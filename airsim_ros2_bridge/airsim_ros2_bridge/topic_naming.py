"""AERION 토픽 네임스페이스 해석 — ROS 의존 없는 순수 모듈 (단위 테스트 용이).

배경 (2026-06-17 체화지능 연동):
  - 기존 검증 계약은 `/{vehicle_name}/camera/...`, `/{vehicle_name}/range/...` (namespaced).
  - 체화지능(Mac embodied-drone) 쪽은 비네임스페이스 `/camera/image`, `/range/front`를 기대.
  - 모든 publisher가 절대경로 f-string으로 토픽을 만들기 때문에 launch remap/namespace push가
    먹지 않는다(절대경로 면역). 따라서 토픽 prefix 자체를 파라미터로 받아야 한다.

설계:
  - `topic_namespace is None`  → vehicle_name 사용 (기본, 현행 동작 100% 보존).
  - `topic_namespace == ''`     → 비네임스페이스 (`/camera/...`). 단일 드론 embodied 모드.
  - `topic_namespace == 'uav0'` → 임의 네임스페이스 (`/uav0/camera/...`).

주의: 비네임스페이스는 단일 드론에서만 안전하다. 다중 드론이 같은 `/camera/image`를
  같은 ROS graph에 올리면 충돌하므로 ROS_DOMAIN_ID 분리 또는 명시 네임스페이스가 필요하다
  (docs/status_2026-06-17 §15.8).
"""


def resolve_namespace(vehicle_name: str, topic_namespace=None) -> str:
    """토픽 prefix에 쓸 네임스페이스 세그먼트를 반환.

    None → vehicle_name, '' → 비네임스페이스, 그 외 → 해당 문자열.
    """
    if topic_namespace is None:
        return vehicle_name
    return topic_namespace


def sensor_topic_prefix(vehicle_name: str, group: str, topic_namespace=None) -> str:
    """센서 그룹(`camera`/`range`/...)의 토픽 prefix를 만든다.

    >>> sensor_topic_prefix('drone1', 'camera')
    '/drone1/camera'
    >>> sensor_topic_prefix('drone1', 'camera', '')
    '/camera'
    >>> sensor_topic_prefix('drone1', 'range', 'uav0')
    '/uav0/range'
    """
    ns = resolve_namespace(vehicle_name, topic_namespace)
    return f'/{ns}/{group}' if ns else f'/{group}'
