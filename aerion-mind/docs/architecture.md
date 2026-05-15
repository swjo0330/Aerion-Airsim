# AERION MIND 개발 아키텍처 설계서

## 1. 구현 범위

이 프로젝트는 AERION 전체 3-Loop 중 Layer 3인 **AERION MIND / Strategic Planning Loop** 구현에 집중한다. MIND는 지휘관의 자연어 임무를 받아 지도/제약 컨텍스트를 주입하고, Planning Graph를 생성하며, Safety Validator를 통과한 그래프만 Soma의 Supervisor/A2A Adapter로 전달한다.

Soma는 실시간 FCU 제어를 담당하므로, Mind는 모터/자세 명령을 직접 생성하지 않는다. Mind의 출력은 `mission_upload`, `mode_change`, `direct_command`, `mission_control` 타입의 고수준 그래프이며 Soma가 이를 DDS/MAVROS 계층으로 변환한다.

## 2. 결론: 프론트엔드는 Next.js/React 권장

PyQt는 단일 GCS 장비에서 폐쇄형 관제툴을 빠르게 만들 때 유리하지만, AERION Mind는 클라우드/서버 Layer 3에 위치하고 다중 드론, 다중 사용자, SSE/WebSocket, 로그 대시보드, 권한관리, 원격 배포가 필요하다. 따라서 기본 프론트는 **Next.js/React**가 적합하다.

권장 구조는 다음과 같다.

- Web Console: Next.js/React
- Backend API: FastAPI
- Streaming: SSE 우선, 추후 WebSocket 확장
- Local-only GCS가 필요할 때: PyQt는 별도 thin client 또는 Electron/Tauri 래퍼로 후순위 구현

## 3. 전체 컴포넌트

```text
Operator Browser
  │
  ▼
Next.js Console : mission input, drone status, planning graph, event stream
  │ REST/SSE
  ▼
FastAPI MIND Server : intent parser, map MCP adapter, graph builder, safety validator
  │ A2A JSON-RPC / SSE
  ▼
Soma Supervisor / A2A Adapter : graph validation, DDS publish, MAVROS/FCU bridge
  │ DDS / MAVLink
  ▼
Autopilot / Drone
```

## 4. Backend 모듈

| 모듈 | 역할 |
|---|---|
| `intent_parser.py` | 자연어 임무에서 task, area, target, altitude, photo_required 추출 |
| `map_mcp.py` | MapBox/MCP 어댑터. 현재는 offline deterministic mock, 실제 MCP 서버로 교체 가능 |
| `planning_graph_builder.py` | Structured Intent + Map Context + Drone State로 Planning Graph 생성 |
| `safety_rules.py` | Rule-first validation. LLM 없이 blocking safety issue 판단 |
| `a2a_client.py` | Soma Supervisor와 `message/send`, `message/send-stream` 통신 |
| `drone_state_cache.py` | 1Hz heartbeat와 stale threshold 관리 |
| `mission_service.py` | 계획/검증/배포 orchestration |

## 5. Safety Gate

Safety Validator는 최소 다음 검사를 수행한다.

1. Graph 비어 있음 여부
2. Node ID 중복
3. Edge 참조 무결성
4. START에서 모든 노드 도달 가능 여부
5. Return-to-home 존재 여부
6. 허용 action whitelist
7. 금지 action blacklist
8. action별 필수 파라미터
9. 고도 상한/하한
10. 속도 상한
11. 좌표 범위
12. geofence 교차
13. 장거리 leg 경고
14. 연결 드론 존재 여부
15. stale heartbeat 여부
16. battery threshold
17. mission duration rough estimate

Blocking severity는 `error`, `critical`이다. `warning`은 운용자에게 표시하지만 dispatch는 막지 않는다.

## 6. Runtime 배치

Docker Compose는 세 서비스를 실행한다.

```text
backend   : FastAPI MIND, port 8020
frontend  : Next.js Console, port 3000
mock-soma : Soma 통신 검증용 A2A endpoint, port 8011
```

실제 Soma와 연결할 때는 `.env`의 `SOMA_DEFAULT_A2A_URL` 또는 각 드론 heartbeat의 `a2a_url`을 실제 Supervisor endpoint로 지정한다.
