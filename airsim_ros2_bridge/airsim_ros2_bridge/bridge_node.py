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
        self.declare_parameter('enable_camera', False)
        self.declare_parameter('enable_range', False)
        self.declare_parameter('range_sensors', ['Distance_Front', 'Distance_Left', 'Distance_Right'])
        self.declare_parameter('range_publish_rate', 20.0)
        self.declare_parameter('range_field_of_view_rad', 0.035)
        # enable_ardu_compat: ArduPilot DDS 토픽(/ap/*) 호환 alias 발행 여부.
        # AERION Phase 2~5는 모두 SimpleFlight/PX4 기반이라 false 유지. Phase 7(별도 트랙) ArduPilot 통합 시에만 true.
        self.declare_parameter('enable_ardu_compat', False)
        # ardu_compat_vehicle: enable_ardu_compat=true일 때만 의미. vehicle key 통일 컨벤션(drone1..)에 맞춤.
        self.declare_parameter('ardu_compat_vehicle', 'drone1')
        self.declare_parameter('velocity_control_mode', 'kinematic')
        self.declare_parameter('control_backend', 'px4_mavros')
        self.declare_parameter('velocity_command_duration', 0.2)
        self.declare_parameter('kinematic_z_ned', -1.0)
        self.declare_parameter('airsim_ip', '127.0.0.1')
        self.declare_parameter('airsim_port', 41451)
        self.declare_parameter('airsim_timeout_sec', 2.0)
        self.declare_parameter('home_latitude', 37.5665)
        self.declare_parameter('home_longitude', 126.9780)
        self.declare_parameter('home_altitude', 0.0)

        vehicle_name = self.get_parameter('vehicle_name').get_parameter_value().string_value
        vehicle_index = self.get_parameter('vehicle_index').get_parameter_value().integer_value
        mavros_instance_namespace = (
            self.get_parameter('mavros_instance_namespace').get_parameter_value().string_value
        )
        vehicle_names = self.get_parameter('vehicle_names').get_parameter_value().string_array_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        camera_fps = self.get_parameter('camera_fps').get_parameter_value().double_value
        enable_camera = self.get_parameter('enable_camera').get_parameter_value().bool_value
        enable_range = self.get_parameter('enable_range').get_parameter_value().bool_value
        range_sensors = self.get_parameter('range_sensors').get_parameter_value().string_array_value
        range_publish_rate = self.get_parameter('range_publish_rate').get_parameter_value().double_value
        range_field_of_view_rad = self.get_parameter('range_field_of_view_rad').get_parameter_value().double_value
        enable_ardu_compat = self.get_parameter('enable_ardu_compat').get_parameter_value().bool_value
        ardu_compat_vehicle = self.get_parameter('ardu_compat_vehicle').get_parameter_value().string_value
        velocity_control_mode = self.get_parameter('velocity_control_mode').get_parameter_value().string_value
        control_backend = self.get_parameter('control_backend').get_parameter_value().string_value
        velocity_command_duration = (
            self.get_parameter('velocity_command_duration').get_parameter_value().double_value
        )
        kinematic_z_ned = self.get_parameter('kinematic_z_ned').get_parameter_value().double_value
        airsim_ip = self.get_parameter('airsim_ip').get_parameter_value().string_value
        airsim_port = self.get_parameter('airsim_port').get_parameter_value().integer_value
        airsim_timeout_sec = self.get_parameter('airsim_timeout_sec').get_parameter_value().double_value
        home_latitude = self.get_parameter('home_latitude').get_parameter_value().double_value
        home_longitude = self.get_parameter('home_longitude').get_parameter_value().double_value
        home_altitude = self.get_parameter('home_altitude').get_parameter_value().double_value

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
        self._range_publishers = []
        self._drone_controllers = []

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
                    )
                    self._range_publishers.append(rng_pub)
                except Exception as e:
                    self.get_logger().warn(
                        f'[{vehicle_name}] Range publisher disabled after initialization error: {e}'
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
                home_latitude=home_latitude,
                home_longitude=home_longitude,
                home_altitude=home_altitude,
                mavros_instance_namespace=spec['mavros_instance_namespace'],
            )
            self._drone_controllers.append(controller)

        self.get_logger().info(f'Bridge running for {len(vehicle_specs)} vehicle instance(s)')

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
