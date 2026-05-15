# Gemini 보일러플레이트 확장 내역

첨부 보일러플레이트는 다음 파일로 구성된 최소 FastAPI + LangGraph 초안이었다.

- `app/main.py`
- `app/api/routes.py`
- `app/agent/graph.py`
- `app/agent/nodes/n1_context.py`
- `app/agent/nodes/n2_planner.py`
- `app/agent/nodes/n3_validator.py`
- `app/core/safety_rules.py`
- `app/tools/mapbox_mcp.py`

이번 패키지는 보일러플레이트를 다음 방향으로 확장했다.

1. Pydantic 기반 Planning Graph 계약 정식화
2. Mission Plan과 Deploy API 분리
3. DroneStateCache 및 1Hz heartbeat 구현
4. Safety Validator를 20개 이상 rule로 확장
5. A2A JSON-RPC/SSE client 구현
6. Mission Event Bus 구현
7. Mock Soma 서비스 추가
8. Next.js 관제 UI 추가
9. Docker Compose로 원클릭 실행 구성

LangGraph는 현재 Docker 실행 안정성을 위해 필수 dependency에서 제외했다. 실제 LLM/Graph orchestration을 붙일 때는 `mission_service.plan()` 내부의 `parse_user_intent → builder.build → validator.validate` 파이프라인을 LangGraph node로 감싸면 된다.

## Agent compatibility layer

기존 Gemini 보일러플레이트의 `mind_app.ainvoke(initial_state)` 호출 형태를 유지하기 위해 `backend/app/agent/graph.py`에 `MindPipeline`을 추가했다. 현재는 LangGraph dependency 없이 실행되는 lightweight pipeline이며, 실제 LangGraph를 도입할 때 이 파일을 `StateGraph` 기반 구현으로 교체하면 API/service 레이어는 그대로 유지된다.
