"""AERION airsim_ros2_bridge — 메인 노드 (드론 1대 = 프로세스 1개 패턴).

본 모듈은 단일 vehicle에 대한 ROS2 노드 + AirSim RPC client + 토픽 발행자 라이프사이클을 묶음.
멀티드론 환경에서는 이 노드를 N개 프로세스(`ros2 launch ... drone_count:=N`)로 spawn.

설계 요지:
  1. **1 드론 = 1 프로세스 패턴**: AirSim RPC IOLoop 경합 (microsoft/AirSim#2607) 회피.
     한 드론의 RPC 타임아웃이 다른 드론에 영향 0. 노드별 CPU/메모리/로그 격리.
  2. **`ThreadSafeAirSimClient`**: MultiThreadedExecutor에서 여러 콜백이 병렬 호출해도 RLock으로
     RPC 직렬화 (`IOLoop is already running` 회피).
  3. **`_resolve_vehicle_specs`**: vehicle_name(단수)이 비어있지 않으면 단일 vehicle 모드,
     아니면 vehicle_names(복수) fallback. 멀티드론 launch는 단수 인자만 사용.
  4. **세 발행자**: 단일 책임 원칙.
     - CameraPublisher : RGB 카메라 (Phase 2 기본, 320x240 권장)
     - RangePublisher  : 전/좌/우 거리센서 (Phase 2.2에서 추가, 충돌 회피용)
     - DroneController : MAVROS 호환 토픽 + 명령 구독 + AirSim API 직접 제어
  5. **`control_backend` 분기**:
     - `airsim_direct` (default) : SimpleFlight 직접 제어 (Phase 2~4)
     - `px4_mavros` : MAVROS setpoint publisher 통해 PX4 SITL 제어 (Phase 5+)

진입점:
  `ros2 run airsim_ros2_bridge bridge_node` 또는 launch 파일.
  주요 launch: `aerion_single_drone.launch.py`, `aerion_multi_drone.launch.py`,
              `aerion_phase4_formation.launch.py`, `aerion_phase5_px4.launch.py`.
"""

import rclpy
# 2026-05-18 v3: MultiThreadedExecutor → SingleThreadedExecutor 로 변경.
# 이유: AirSim Tornado IOLoop는 thread-safe 안 함. ThreadSafeAirSimClient의 RLock으로도
# IOLoop 자체 재진입은 막을 수 없음 (microsoft/AirSim#2607 잔재). MultiThread에서 카메라/range/pose
# 콜백이 동시 실행 시 'IOLoop is already running' 대량 발생 → setpoint RPC도 fail → 포메이션 흔들림.
# 단일 스레드는 throughput 감소가 있지만 1 드론 = 1 프로세스 패턴이라 N대 전체로는 N개 스레드 = N개 IOLoop.
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import airsim
import threading

# AERION 토픽 발행자 3종 (단일 책임 원칙 — docs/ARCHITECTURE.md 참조)
from airsim_ros2_bridge.camera_publisher import CameraPublisher
from airsim_ros2_bridge.drone_controller import DroneController
from airsim_ros2_bridge.lidar_publisher import LidarPublisher
from airsim_ros2_bridge.range_publisher import RangePublisher


class ThreadSafeAirSimClient:
    """AirSim RPC 호출을 스레드 간 직렬화하여 Tornado IOLoop 재진입 회피.

    배경: AirSim RPC 서버는 rpclib + Tornado 기반. 같은 client 인스턴스를 여러 스레드가
    동시에 호출하면 'IOLoop is already running' 에러 (microsoft/AirSim#2607).
    MultiThreadedExecutor로 여러 콜백이 병렬 실행되어도 RLock으로 RPC 자체는 직렬화.
    """

    def __init__(self, client):
        self._client = client
        self._lock = threading.RLock()

    def __getattr__(self, name):
        attr = getattr(self._client, name)
        if not callable(attr):
            return attr

        def locked_call(*args, **kwargs):
            with self._lock:
                return attr(*args, **kwargs)

        return locked_call


class AirSimBridgeNode(Node):
    def __init__(self):
        super().__init__('airsim_bridge')

        # Parameters
        self.declare_parameter('vehicle_name', '')
        self.declare_parameter('vehicle_index', -1)
        self.declare_parameter('mavros_instance_namespace', '')
        self.declare_parameter('vehicle_names', ['Drone0', 'Drone1'])
        self.declare_parameter('camera_name', 'front_center')
        self.declare_parameter('camera_fps', 30.0)
        self.declare_parameter('airsim_image_channel_order', 'bgr')
        self.declare_parameter('image_compressed', False)
        self.declare_parameter('jpeg_quality', 70)
        # topic_namespace: '__vehicle__'(기본)=/{vehicle_name}/* 레거시, ''/'bare'=bare(/camera/image 등 단일드론 PoC), 그외=해당 ns
        self.declare_parameter('topic_namespace', '__vehicle__')
        self.declare_parameter('enable_camera', False)
        self.declare_parameter('enable_range', False)
        self.declare_parameter('range_sensors', ['Distance_Front', 'Distance_Left', 'Distance_Right'])
        self.declare_parameter('range_publish_rate', 20.0)
        self.declare_parameter('range_field_of_view_rad', 0.035)
        # range_mode: 'range'(레거시 Range) | 'laserscan'(/range/front LaserScan) | 'points'(/range/front/points PointCloud2, 체화지능 3D) | 'both'
        self.declare_parameter('range_mode', 'range')
        self.declare_parameter('enable_lidar', False)
        self.declare_parameter('lidar_name', 'Lidar_Front')
        self.declare_parameter('lidar_publish_rate', 10.0)
        self.declare_parameter('enable_lidar_obstacle', True)
        self.declare_parameter('lidar_obstacle_min_distance', 0.3)
        self.declare_parameter('lidar_obstacle_max_distance', 25.0)
        self.declare_parameter('lidar_obstacle_forward_fov_deg', 90.0)
        self.declare_parameter('lidar_obstacle_vertical_fov_deg', 45.0)
        # enable_ardu_compat: ArduPilot DDS 토픽(/ap/*) 호환 alias 발행 여부.
        # AERION Phase 2~5는 모두 SimpleFlight/PX4 기반이라 false 유지. Phase 7(별도 트랙) ArduPilot 통합 시에만 true.
        self.declare_parameter('enable_ardu_compat', False)
        # ardu_compat_vehicle: enable_ardu_compat=true일 때만 의미. vehicle key 통일 컨벤션(drone1..)에 맞춤.
        self.declare_parameter('ardu_compat_vehicle', 'drone1')
        self.declare_parameter('velocity_control_mode', 'kinematic')
        self.declare_parameter('control_backend', 'px4_mavros')
        self.declare_parameter('controller_pose_rate', 10.0)
        # publish_mavros_state: px4_mavros에서도 AirSim RPC→/mavros/* 상태 발행(MAVROS처럼). 제어 서비스는 미포함.
        self.declare_parameter('publish_mavros_state', False)
        self.declare_parameter('velocity_command_duration', 0.2)
        self.declare_parameter('kinematic_z_ned', -1.0)
        self.declare_parameter('forward_ap_cmd_vel_to_mavros', False)
        self.declare_parameter('local_motion_from_mavros', False)
        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('airsim_port', 41451)
        self.declare_parameter('airsim_timeout_sec', 2.0)
        self.declare_parameter('home_latitude', 37.5665)
        self.declare_parameter('home_longitude', 126.9780)
        self.declare_parameter('home_altitude', 0.0)
        self.declare_parameter('enable_gps_debug_log', False)
        self.declare_parameter('gps_debug_log_rate', 2.0)
        self.declare_parameter('gps_debug_sensor_name', '')

        vehicle_name = self.get_parameter('vehicle_name').get_parameter_value().string_value
        vehicle_index = self.get_parameter('vehicle_index').get_parameter_value().integer_value
        mavros_instance_namespace = (
            self.get_parameter('mavros_instance_namespace').get_parameter_value().string_value
        )
        vehicle_names = self.get_parameter('vehicle_names').get_parameter_value().string_array_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        camera_fps = self.get_parameter('camera_fps').get_parameter_value().double_value
        airsim_image_channel_order = (
            self.get_parameter('airsim_image_channel_order').get_parameter_value().string_value
        )
        image_compressed = self.get_parameter('image_compressed').get_parameter_value().bool_value
        jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        topic_namespace_raw = self.get_parameter('topic_namespace').get_parameter_value().string_value
        if topic_namespace_raw == '__vehicle__':
            topic_ns = None          # 레거시: /{vehicle_name}/*
        elif topic_namespace_raw in ('', 'bare', '/'):
            topic_ns = ''            # bare: /camera/image, /mavros/* ...
        else:
            topic_ns = topic_namespace_raw
        enable_camera = self.get_parameter('enable_camera').get_parameter_value().bool_value
        enable_range = self.get_parameter('enable_range').get_parameter_value().bool_value
        range_sensors = self.get_parameter('range_sensors').get_parameter_value().string_array_value
        range_publish_rate = self.get_parameter('range_publish_rate').get_parameter_value().double_value
        range_field_of_view_rad = self.get_parameter('range_field_of_view_rad').get_parameter_value().double_value
        range_mode = self.get_parameter('range_mode').get_parameter_value().string_value
        enable_lidar = self.get_parameter('enable_lidar').get_parameter_value().bool_value
        lidar_name = self.get_parameter('lidar_name').get_parameter_value().string_value
        lidar_publish_rate = self.get_parameter('lidar_publish_rate').get_parameter_value().double_value
        enable_lidar_obstacle = self.get_parameter('enable_lidar_obstacle').get_parameter_value().bool_value
        lidar_obstacle_min_distance = (
            self.get_parameter('lidar_obstacle_min_distance').get_parameter_value().double_value
        )
        lidar_obstacle_max_distance = (
            self.get_parameter('lidar_obstacle_max_distance').get_parameter_value().double_value
        )
        lidar_obstacle_forward_fov_deg = (
            self.get_parameter('lidar_obstacle_forward_fov_deg').get_parameter_value().double_value
        )
        lidar_obstacle_vertical_fov_deg = (
            self.get_parameter('lidar_obstacle_vertical_fov_deg').get_parameter_value().double_value
        )
        enable_ardu_compat = self.get_parameter('enable_ardu_compat').get_parameter_value().bool_value
        ardu_compat_vehicle = self.get_parameter('ardu_compat_vehicle').get_parameter_value().string_value
        velocity_control_mode = self.get_parameter('velocity_control_mode').get_parameter_value().string_value
        control_backend = self.get_parameter('control_backend').get_parameter_value().string_value
        controller_pose_rate = self.get_parameter('controller_pose_rate').get_parameter_value().double_value
        publish_mavros_state = self.get_parameter('publish_mavros_state').get_parameter_value().bool_value
        velocity_command_duration = (
            self.get_parameter('velocity_command_duration').get_parameter_value().double_value
        )
        kinematic_z_ned = self.get_parameter('kinematic_z_ned').get_parameter_value().double_value
        forward_ap_cmd_vel_to_mavros = (
            self.get_parameter('forward_ap_cmd_vel_to_mavros').get_parameter_value().bool_value
        )
        local_motion_from_mavros = (
            self.get_parameter('local_motion_from_mavros').get_parameter_value().bool_value
        )
        airsim_ip = self.get_parameter('airsim_ip').get_parameter_value().string_value
        airsim_port = self.get_parameter('airsim_port').get_parameter_value().integer_value
        airsim_timeout_sec = self.get_parameter('airsim_timeout_sec').get_parameter_value().double_value
        home_latitude = self.get_parameter('home_latitude').get_parameter_value().double_value
        home_longitude = self.get_parameter('home_longitude').get_parameter_value().double_value
        home_altitude = self.get_parameter('home_altitude').get_parameter_value().double_value
        enable_gps_debug_log = self.get_parameter('enable_gps_debug_log').get_parameter_value().bool_value
        gps_debug_log_rate = self.get_parameter('gps_debug_log_rate').get_parameter_value().double_value
        gps_debug_sensor_name = (
            self.get_parameter('gps_debug_sensor_name').get_parameter_value().string_value
        )

        # Connect to AirSim
        self.get_logger().info(f'Connecting to AirSim at {airsim_ip}:{airsim_port}...')
        raw_client = airsim.MultirotorClient(
            ip=airsim_ip,
            port=airsim_port,
            timeout_value=airsim_timeout_sec,
        )
        self._client = ThreadSafeAirSimClient(raw_client)
        self._client.confirmConnection()
        self.get_logger().info('Connected to AirSim!')

        vehicle_specs = self._resolve_vehicle_specs(
            vehicle_name=vehicle_name,
            vehicle_index=vehicle_index,
            mavros_instance_namespace=mavros_instance_namespace,
            vehicle_names=vehicle_names,
        )

        # Create camera publishers and controllers for each vehicle
        self._camera_publishers = []
        self._lidar_publishers = []
        self._range_publishers = []
        self._drone_controllers = []
        self._gps_debug_vehicle_names = []
        self._gps_debug_sensor_name = gps_debug_sensor_name

        for spec in vehicle_specs:
            vehicle_name = spec['vehicle_name']
            self.get_logger().info(f'Setting up {vehicle_name}...')

            if enable_camera:
                try:
                    cam_pub = CameraPublisher(
                        node=self,
                        client=self._client,
                        vehicle_name=vehicle_name,
                        camera_name=camera_name,
                        publish_rate=camera_fps,
                        image_channel_order=airsim_image_channel_order,
                        publish_compressed=image_compressed,
                        jpeg_quality=jpeg_quality,
                        topic_namespace=topic_ns,
                    )
                    self._camera_publishers.append(cam_pub)
                except Exception as e:
                    self.get_logger().warn(
                        f'[{vehicle_name}] Camera publisher disabled after initialization error: {e}'
                    )

            # Range publisher (Phase 2.2~). 거리센서가 settings.json에 정의 안 됐어도
            # RangePublisher가 _probe_sensors_once에서 경고만 출력하고 노드는 살아있도록 설계됨.
            # 초기화 자체가 실패하는 케이스(드물게)에만 이 try/except로 비활성화.
            if enable_range and range_sensors:
                try:
                    rng_pub = RangePublisher(
                        node=self,
                        client=self._client,
                        vehicle_name=vehicle_name,
                        distance_sensors=list(range_sensors),
                        publish_rate=range_publish_rate,
                        field_of_view_rad=range_field_of_view_rad,
                        topic_namespace=topic_ns,
                        range_mode=range_mode,
                    )
                    self._range_publishers.append(rng_pub)
                except Exception as e:
                    self.get_logger().warn(
                        f'[{vehicle_name}] Range publisher disabled after initialization error: {e}'
                    )

            if enable_lidar:
                try:
                    lidar_pub = LidarPublisher(
                        node=self,
                        client=self._client,
                        vehicle_name=vehicle_name,
                        lidar_name=lidar_name,
                        publish_rate=lidar_publish_rate,
                        enable_obstacle=enable_lidar_obstacle,
                        min_distance=lidar_obstacle_min_distance,
                        max_distance=lidar_obstacle_max_distance,
                        obstacle_forward_fov_deg=lidar_obstacle_forward_fov_deg,
                        obstacle_vertical_fov_deg=lidar_obstacle_vertical_fov_deg,
                        topic_namespace=topic_ns,
                    )
                    self._lidar_publishers.append(lidar_pub)
                except Exception as e:
                    self.get_logger().warn(
                        f'[{vehicle_name}] LiDAR publisher disabled after initialization error: {e}'
                    )

            controller = DroneController(
                node=self,
                client=self._client,
                vehicle_name=vehicle_name,
                enable_ardu_compat=enable_ardu_compat and vehicle_name == ardu_compat_vehicle,
                velocity_control_mode=velocity_control_mode,
                control_backend=control_backend,
                velocity_command_duration=velocity_command_duration,
                kinematic_z_ned=kinematic_z_ned,
                forward_ap_cmd_vel_to_mavros=forward_ap_cmd_vel_to_mavros,
                local_motion_from_mavros=local_motion_from_mavros,
                home_latitude=home_latitude,
                home_longitude=home_longitude,
                home_altitude=home_altitude,
                mavros_instance_namespace=('mavros' if topic_ns == '' else spec['mavros_instance_namespace']),
                pose_publish_rate=controller_pose_rate,
                topic_namespace=topic_ns,
                publish_mavros_state=publish_mavros_state,
            )
            self._drone_controllers.append(controller)
            self._gps_debug_vehicle_names.append(vehicle_name)

        self.get_logger().info(f'Bridge running for {len(vehicle_specs)} vehicle instance(s)')

        self._gps_debug_timer = None
        if enable_gps_debug_log and self._gps_debug_vehicle_names:
            rate = max(0.2, gps_debug_log_rate)
            self._gps_debug_timer = self.create_timer(1.0 / rate, self._publish_gps_debug_log)
            self.get_logger().info(
                f'AirSim GPS debug log enabled for {self._gps_debug_vehicle_names} @ {rate:.1f}Hz'
            )

    @staticmethod
    def _resolve_vehicle_specs(
        vehicle_name: str,
        vehicle_index: int,
        mavros_instance_namespace: str,
        vehicle_names,
    ) -> list[dict[str, str]]:
        if vehicle_name:
            index = vehicle_index if vehicle_index >= 0 else AirSimBridgeNode._index_from_vehicle_name(vehicle_name)
            namespace = mavros_instance_namespace or f'mavros{index}'
            return [
                {
                    'vehicle_name': vehicle_name,
                    'mavros_instance_namespace': namespace,
                }
            ]

        return [
            {
                'vehicle_name': name,
                'mavros_instance_namespace': f'mavros{index}',
            }
            for index, name in enumerate(vehicle_names)
        ]

    @staticmethod
    def _index_from_vehicle_name(vehicle_name: str) -> int:
        digits = ''
        for char in reversed(vehicle_name):
            if not char.isdigit():
                break
            digits = char + digits
        return int(digits) if digits else 0

    def _publish_gps_debug_log(self) -> None:
        for vehicle_name in self._gps_debug_vehicle_names:
            try:
                gps_data = self._client.getGpsData(self._gps_debug_sensor_name, vehicle_name)
                geo = gps_data.gnss.geo_point
                message = f'GPS {vehicle_name}'
                value = f'lat={geo.latitude:.7f} lon={geo.longitude:.7f} alt={geo.altitude:.2f}m'
                self._client.simPrintLogMessage(message, value, 0)
            except Exception as exc:
                self.get_logger().warn(f'[{vehicle_name}] GPS debug log update failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = AirSimBridgeNode()
    # SingleThreadedExecutor: AirSim Tornado IOLoop 재진입 회피 (위 import 주석 참조).
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
