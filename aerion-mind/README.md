# AERION MIND Stack

AERION MIND 전략 기획 루프를 FastAPI 백엔드와 Next.js 관제 프론트엔드로 실행하는 Docker 기반 프로젝트입니다.

이 저장소는 첨부된 Gemini 보일러플레이트를 다음 수준으로 확장한 실전형 골격입니다.

- FastAPI 기반 MIND Tactical Server
- 자연어 임무 입력 → 구조화 의도 → Planning Graph 생성 → Safety Validation → Soma A2A Dispatch
- Soma 통신 구간: JSON-RPC 2.0 `message/send`, SSE `message/send-stream`, Heartbeat, Event Callback
- Next.js/React 기반 관제 UI
- Mock Soma 서비스 포함: 실제 Soma가 없어도 전체 흐름 검증 가능

## 빠른 실행

```bash
cp .env.example .env
docker compose up --build
```

접속 주소:

- Frontend: http://localhost:3000
- Backend OpenAPI: http://localhost:8020/docs
- Mock Soma: http://localhost:8011/health

## 로컬 백엔드 실행

```bash
cd backend
python -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8020 --reload
```

## 핵심 API

```bash
curl -s http://localhost:8020/api/v1/health | jq
curl -s http://localhost:8020/api/v1/drones | jq
curl -s -X POST http://localhost:8020/api/v1/mission/deploy \
  -H 'Content-Type: application/json' \
  -d '{"intent":"A구역 산악 지역 정찰 임무. 고도 120m 이하 이동 후 목표 지점 사진 촬영.","target_drones":["drone1"],"dispatch_mode":"stream"}' | jq
```

실시간 이벤트 스트림:

```bash
curl -N http://localhost:8020/api/v1/mission/<mission_id>/events
```

## 문서

- `docs/architecture.md`: 전체 설계 자료
- `docs/frontend_decision.md`: PyQt vs Next.js/React 의사결정
- `docs/soma_integration.md`: Soma 통신 호출 방법
- `docs/api_contracts.md`: API/JSON 계약
- `docs/docker_runbook.md`: Docker 실행/운영 절차
- `docs/migration_notes.md`: Gemini 보일러플레이트에서 확장한 내용
