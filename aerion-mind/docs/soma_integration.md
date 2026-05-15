# AERION MIND ↔ Soma 통신 설계 및 호출 방법

## 1. 통신 원칙

Mind는 FCU/MAVROS를 직접 제어하지 않는다. Mind는 Soma Supervisor 또는 A2A Adapter로 **Planning Graph**를 전달한다. Soma는 자체 Supervisor에서 재검증 후 DDS/MAVROS/FCU로 변환한다.

- Mind → Soma: A2A JSON-RPC 2.0
- Soma → Mind: Heartbeat REST, Event Callback REST, 선택적 A2A callback
- 실시간 진행률: SSE `message/send-stream`

## 2. Soma Heartbeat

Soma는 1Hz로 Mind에 자신의 상태와 A2A endpoint를 등록한다.

```bash
curl -X POST http://localhost:8020/api/v1/drones/drone1/heartbeat \
  -H 'Content-Type: application/json' \
  -d '{
    "system_id":"drone1",
    "mode":"GUIDED",
    "armed":true,
    "battery_percent":82,
    "position":{"lat":37.5326,"lon":127.0246,"alt_m":35},
    "heading_deg":45,
    "a2a_url":"http://localhost:8011",
    "role":"leader",
    "status":"ready"
  }'
```

Mind는 `a2a_url`을 우선 사용하고, 없으면 `.env`의 `SOMA_DEFAULT_A2A_URL`을 사용한다.

## 3. Mind → Soma: blocking JSON-RPC 호출

Endpoint:

```text
POST {SOMA_A2A_URL}/a2a/message/send
```

Payload:

```json
{
  "jsonrpc": "2.0",
  "id": "msg_xxx",
  "method": "message/send",
  "params": {
    "message": {
      "message_id": "msg_xxx",
      "role": "mind",
      "type": "mission_upload",
      "parts": [
        {
          "kind": "data",
          "data": {
            "type": "mission_upload",
            "drone_id": "drone1",
            "mission_id": "mission_xxx",
            "graph_id": "pg_xxx",
            "graph": { "nodes": [], "edges": [] },
            "constraints": { "max_altitude_m": 120 }
          }
        }
      ],
      "metadata": {
        "protocol": "AERION-A2A",
        "source": "aerion-mind",
        "target_drone": "drone1"
      }
    }
  }
}
```

Response:

```json
{
  "jsonrpc": "2.0",
  "id": "msg_xxx",
  "result": {
    "status": "received",
    "drone_id": "drone1",
    "mission_id": "mission_xxx",
    "graph_id": "pg_xxx"
  }
}
```

## 4. Mind → Soma: SSE streaming 호출

Endpoint:

```text
POST {SOMA_A2A_URL}/a2a/message/send-stream
```

Soma는 다음 형식으로 진행률을 보낸다.

```text
event: ack
data: {"status":"received","mission_id":"mission_xxx"}

event: progress
data: {"node":"WP01","status":"ok"}

event: done
data: {"status":"completed"}
```

Backend 구현 위치:

```text
backend/app/services/a2a_client.py
backend/app/services/mission_service.py
```

## 5. Frontend에서 실시간 이벤트 보는 방법

Mind는 Soma에서 받은 SSE를 내부 mission event bus에 재발행한다. 프론트엔드는 다음 endpoint를 구독한다.

```text
GET http://localhost:8020/api/v1/mission/{mission_id}/events
```

## 6. 실제 Soma 적용 체크리스트

1. Soma Supervisor에 `/a2a/message/send`와 `/a2a/message/send-stream` 구현
2. Soma heartbeat에 실제 `a2a_url` 포함
3. Planning Graph 수신 후 Soma 내부 Safety Validator 재검증
4. `command_type`에 따라 DDS topic 또는 MAVROS service로 라우팅
5. 임무 진행 상태를 SSE 또는 `/api/v1/soma/events`로 보고
6. 통신 단절 시 Soma는 기존 graph와 local reactive loop로 자율 지속
