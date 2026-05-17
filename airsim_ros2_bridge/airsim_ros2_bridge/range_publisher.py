"""AERION Phase 2.2: AirSim Distance 센서를 sensor_msgs/Range로 발행.

설계 배경:
  - AirSim Distance 센서(`SensorType=5`)는 단일 ray-cast (사실상 FOV=0).
    sensor_msgs/Range는 `radiation_type` ULTRASOUND(원뿔 빔 가정) / INFRARED 둘 중 하나만 받으므로
    단일 빔 LiDAR/IR을 모사하는 INFRARED 쪽이 의미상 정확.
  - 전방/좌/우 3개 센서를 충돌 회피 신호로 사용 (Phase 4 포메이션 변경 시 안전망 토픽).
  - 폴링 주기 20Hz: AirSim RPC 50Hz 상한 (microsoft/AirSim#3859) 안에서 카메라(10Hz) +
    Range×3 + DroneController의 pose/imu 폴링이 공존 가능한 안전 마진 값.

토픽/프레임 컨벤션:
  Topic:  /{vehicle_name}/range/{front,left,right}   (vehicle_name=ROS namespace와 동일)
  Frame:  {vehicle_name}_range_{front,left,right}_link
  QoS:    depth=10 RELIABLE (기본). 외부 노출 시 BEST_EFFORT로 재선언 권장 (sensor_data 프로파일).

알려진 함정:
  - Colosseum의 메서드명은 `getDistanceSensorData` (sim 접두사 없음). 본가 Microsoft AirSim의
    `simGetDistanceSensorData`와 다름. AERION 트랙은 Colosseum 사용이므로 본 모듈도 후자.
  - settings.json에 정의되지 않은 센서 이름이 들어오면 RPC가 throw → `_probe_sensors_once`로
    초기 1회 호출해 즉시 경고. 런타임 중에도 5초 throttle로 경고만 (노드 죽이지 않음).
"""

import airsim
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header


# AERION 표준 거리센서 3개. settings.json의 Sensors 키와 정확히 일치해야 함.
DEFAULT_DISTANCE_SENSORS = ('Distance_Front', 'Distance_Left', 'Distance_Right')

# AirSim 센서 이름 → ROS2 토픽 suffix (소문자, image_transport 컨벤션과 정합)
TOPIC_SUFFIX = {
    'Distance_Front': 'front',
    'Distance_Left': 'left',
    'Distance_Right': 'right',
}

# AirSim 센서 이름 → TF frame_id suffix
# REP-105 권장: 각 드론마다 base_link → range_*_link 트리. base_link → range frame 사이
# static TF는 robot_state_publisher가 발행 (이 모듈에서 직접 발행하지 않음).
FRAME_SUFFIX = {
    'Distance_Front': 'range_front_link',
    'Distance_Left': 'range_left_link',
    'Distance_Right': 'range_right_link',
}


class RangePublisher:
    """단일 드론의 AirSim Distance 센서를 sensor_msgs/Range로 발행.

    한 인스턴스 = 한 드론. 멀티드론은 각 브릿지 프로세스가 자기 RangePublisher 인스턴스를 보유
    (1 드론 = 1 프로세스 패턴, AirSim RPC IOLoop 경합 회피 — Phase 3 RPC stagger와 같이 작동).
    """

    def __init__(
        self,
        node: Node,
        client: airsim.MultirotorClient,
        vehicle_name: str,
        distance_sensors=DEFAULT_DISTANCE_SENSORS,
        publish_rate: float = 20.0,
        field_of_view_rad: float = 0.035,
    ):
        self._node = node
        self._client = client
        self._vehicle_name = vehicle_name
        self._distance_sensors = tuple(distance_sensors)
        self._field_of_view = field_of_view_rad
        self._last_error_ns = {name: 0 for name in self._distance_sensors}
        self._callback_group = ReentrantCallbackGroup()

        topic_prefix = f'/{vehicle_name}/range'
        self._publishers = {}
        self._frame_ids = {}
        topics_advertised = []
        for sensor_name in self._distance_sensors:
            suffix = TOPIC_SUFFIX.get(sensor_name, sensor_name.lower())
            frame_suffix = FRAME_SUFFIX.get(sensor_name, f'range_{suffix}_link')
            topic = f'{topic_prefix}/{suffix}'
            self._publishers[sensor_name] = node.create_publisher(Range, topic, 10)
            self._frame_ids[sensor_name] = f'{vehicle_name}_{frame_suffix}'
            topics_advertised.append(topic)

        node.get_logger().info(
            f'[{vehicle_name}] Range publishers: {topics_advertised} @ {publish_rate:.1f}Hz'
        )

        # Sanity probe: 첫 호출 시 settings의 센서 정의 누락이면 즉시 경고
        self._probe_sensors_once()

        self._timer = node.create_timer(
            1.0 / publish_rate,
            self._publish_callback,
            callback_group=self._callback_group,
        )

    def _probe_sensors_once(self):
        for sensor_name in self._distance_sensors:
            try:
                self._client.getDistanceSensorData(sensor_name, self._vehicle_name)
            except Exception as e:
                self._node.get_logger().warn(
                    f'[{self._vehicle_name}] Distance sensor {sensor_name} probe failed '
                    f'(settings.json에 정의되어 있는지 확인): {e}'
                )

    def _publish_callback(self):
        stamp = self._node.get_clock().now().to_msg()
        for sensor_name in self._distance_sensors:
            try:
                data = self._client.getDistanceSensorData(sensor_name, self._vehicle_name)
                if data is None:
                    continue

                msg = Range()
                msg.header = Header()
                msg.header.stamp = stamp
                msg.header.frame_id = self._frame_ids[sensor_name]
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = float(self._field_of_view)
                msg.min_range = float(getattr(data, 'min_distance', 0.2))
                msg.max_range = float(getattr(data, 'max_distance', 40.0))
                msg.range = float(getattr(data, 'distance', msg.max_range))
                self._publishers[sensor_name].publish(msg)

            except Exception as e:
                now_ns = self._node.get_clock().now().nanoseconds
                if now_ns - self._last_error_ns[sensor_name] > 5_000_000_000:
                    self._node.get_logger().warn(
                        f'[{self._vehicle_name}] Range sensor {sensor_name} error: {e}'
                    )
                    self._last_error_ns[sensor_name] = now_ns
