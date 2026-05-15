# 프론트엔드 선택: PyQt vs Next.js/React

## 결론

AERION MIND의 기본 프론트엔드는 **Next.js/React**로 구현하는 것이 맞다.

## 판단 기준

| 기준 | PyQt | Next.js/React |
|---|---|---|
| 단일 장비 로컬 GCS | 매우 좋음 | 가능하지만 브라우저 필요 |
| 다중 사용자/원격 접속 | 별도 배포 필요 | 기본적으로 강함 |
| SSE/WebSocket 실시간 로그 | 가능하나 직접 구현량 큼 | 웹 표준으로 쉬움 |
| Docker/Cloud 배포 | GUI/X11 문제 | 컨테이너 친화적 |
| 관제 대시보드 확장 | 위젯 직접 개발 | 차트/지도/권한/라우팅 확장 쉬움 |
| 모바일/태블릿 접근 | 어려움 | 쉬움 |
| 오프라인 폐쇄망 | 가능 | 가능 |

## 권장 전략

1. 1차 구현: Next.js/React 관제 콘솔
2. 2차 구현: 지도/텔레메트리/로그/mission replay 추가
3. Local-only 장비가 필요할 때: PyQt를 별도 thin client로 제작하거나, 웹 콘솔을 Electron/Tauri로 패키징

## 이번 패키지 구현

`frontend/`는 Next.js App Router 기반이다. 현재 기능은 다음이다.

- 자연어 임무 입력
- target drone 선택
- dispatch mode 선택
- Planning Graph JSON 확인
- 드론 heartbeat 테이블
- mission SSE event stream 표시
