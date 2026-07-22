# docs/airsim — 문서 인덱스

AERION AirSim(Colosseum)+PX4 SITL ↔ 체화지능(Mac) 연동 문서. **목적별 디렉터리**로 정리(2026-07-22).

## 📂 구조

| 디렉터리 | 목적 |
|---|---|
| `guide/` | **실행 가이드** — 지금 무엇을 어떻게 돌리는가 (최신 우선 참조) |
| `runbook/` | **런북** — 단계별 실행 절차 |
| `design/` | **설계서** — 아키텍처·결정·검토 (why) |
| `reference/` | **레퍼런스** — 초기 조사·매핑 기록 (역사적) |

## 📄 문서

### guide/ (실행 가이드)
- [`2026-07-22-aerion-embodied-execution-guide.md`](guide/2026-07-22-aerion-embodied-execution-guide.md) — ★ **현행 통합 실행 가이드**. 환경 스펙·경로·포트·아키텍처·명령·트러블슈팅·제어. 전체 파이프라인 `scripts/run_embodied_pipeline.sh` 포함.
- [`2026-07-08-airsim-local-mission-smoke-prep.md`](guide/2026-07-08-airsim-local-mission-smoke-prep.md) — 로컬 미션 스모크 테스트 준비.

### runbook/ (런북)
- [`2026-06-18-embodied-link-runbook.md`](runbook/2026-06-18-embodied-link-runbook.md) — 단일 드론 2채널 링크 실행 런북(원본). 최신 통합본은 guide/ 참조.

### design/ (설계서)
- [`2026-07-22-multi-drone-px4-sysid-design.md`](design/2026-07-22-multi-drone-px4-sysid-design.md) — ★ 멀티 PX4 SITL sysid/포트 배분 + 인스턴스별 초기 GPS 설계·검토.
- [`2026-06-18-embodied-topic-integration-design.md`](design/2026-06-18-embodied-topic-integration-design.md) — 토픽 통합 설계(2채널, gap=0, 3-agent 결과).
- [`2026-06-17-sim-comm-extension-design.md`](design/2026-06-17-sim-comm-extension-design.md) — 연동 현황/설계 권위 문서.

### reference/ (레퍼런스)
- [`2026-06-09-airsim-px4-zenoh-integration.md`](reference/2026-06-09-airsim-px4-zenoh-integration.md) — 초기 zenoh 통합 조사.
- [`2026-06-09-airsim-px4-topic-mapping.md`](reference/2026-06-09-airsim-px4-topic-mapping.md) — 초기 토픽 매핑 조사.

## 🔗 관련 (docs/airsim 밖)
- 실행 스크립트: `scripts/run_embodied_pipeline.sh`(전체), `scripts/test_embodied_link.sh`(단일 진입점), `scripts/run_ue_carla_town10.sh`(UE Town10).
- 프로젝트 규약: 루트 `CLAUDE.md`.
