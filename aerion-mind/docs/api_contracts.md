# API 및 데이터 계약

## 1. Mission Plan

```text
POST /api/v1/mission/plan
```

Request:

```json
{
  "intent": "A구역 산악 지역 정찰 임무. 고도 120m 이하 이동 후 목표 지점 사진 촬영.",
  "target_drones": ["drone1"],
  "constraints": {
    "max_altitude_m": 120,
    "max_speed_mps": 8,
    "require_rth": true
  }
}
```

Response:

```json
{
  "status": "success",
  "structured_intent": {},
  "planning_graph": {},
  "safety_report": {"passed": true, "issues": []}
}
```

## 2. Mission Deploy

```text
POST /api/v1/mission/deploy
```

`dispatch_mode`:

- `none`: 계획/검증만 수행
- `send`: Soma A2A blocking send
- `stream`: Soma A2A SSE streaming

## 3. Planning Graph 핵심 필드

```json
{
  "graph_id": "pg_xxx",
  "mission_id": "mission_xxx",
  "intent": {},
  "assigned_drones": ["drone1"],
  "constraints": {},
  "nodes": [
    {
      "id": "WP01",
      "type": "waypoint",
      "action": "navigate_to",
      "command_type": "mission_upload",
      "assigned_to": "drone1",
      "params": {"lat": 37.1, "lon": 127.1, "alt_m": 120, "speed_mps": 8}
    }
  ],
  "edges": [{"from": "START", "to": "WP01", "condition": "success"}]
}
```

## 4. Command Type → Soma 라우팅 권장

| command_type | Soma 처리 |
|---|---|
| `mission_upload` | DDS `/a2a/droneN/mission_command` 또는 MAVROS waypoint push |
| `mode_change` | MAVROS set_mode |
| `direct_command` | takeoff, capture, hover 같은 단발 명령 |
| `mission_control` | RTH, abort, resume, complete |

## 5. Safety Issue

```json
{
  "rule_id": "R014",
  "severity": "critical",
  "message": "Altitude exceeds allowed maximum",
  "node_id": "WP02",
  "detail": {"max_altitude_m": 500}
}
```
