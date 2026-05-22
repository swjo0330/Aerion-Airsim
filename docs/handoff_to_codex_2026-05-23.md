# AERION Phase 4-Δ — Codex 인수인계 (2026-05-23)

이전 세션 (Claude Code Opus 4.7) 에서 SimpleFlight 3-drone formation 시연을 시도. 여러 fix 누적 후에도 **drone2/3 가 일부 패턴에서 freeze 되는 root cause 가 미해결**. 본 문서로 Codex 가 동일 컨텍스트로 이어받아 진행.

---

## 1. 최종 목표

**CARLA+AirSim + MAVROS + PX4** 또는 **AirSim SimpleFlight** 환경에서 **3대 드론** 포메이션 시뮬레이션. 패턴: TRIANGLE / V3 / COLUMN / DIAMOND3 순환, 패턴 간 부드러운 전환. 사용자가 시각적으로 확인 가능.

현재 단계: **SimpleFlight + AirSim API 직접 제어** path 로 우회 시도 중. 추락은 멈췄지만 일부 vehicle 이 패턴별로 freeze.

---

## 2. 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04 |
| GPU | NVIDIA (Driver 580) |
| UE | 5.6.1 source build at `~/airsim/unreal-engine/` |
| Colosseum | main branch at `~/airsim/Colosseum/`, libAirLib.a 빌드 완료 |
| UE 프로젝트 | `~/airsim/Colosseum/Unreal/Environments/BlocksV2/BlocksV2.uproject` |
| Python | 3.10 (system) + 3.12 (pyenv) — script 실행은 3.10 |
| AirSim Python client | `~/airsim/Colosseum/PythonClient/airsim/` (editable install) |
| PX4 | `~/airsim/PX4-Autopilot/` (none_iris build) — **현재 미사용 (SimpleFlight 우회)** |
| ROS2 | Humble + rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 — **현재 미사용** |

settings.json 경로: `~/Documents/AirSim/settings.json` (UE Play 시점에 로드).

---

## 3. Repo 위치 + branch

- **Repo**: `~/workspace/projects/aerion-airsim`
- **Remote**: `git@github.com:swjo0330/Aerion-Airsim.git` (SSH key `Branchman555` 계정으로 push 가능)
- **Branch**: `branc/airsim-ros2-bridge` (협업자 `wnsgud5813` 와 공동 사용)
- **현재 HEAD**: `f87b443` (push 완료)

협업자가 직접 push 운영, PR 없음. push 명령:
```bash
git push git@github.com:swjo0330/Aerion-Airsim.git branc/airsim-ros2-bridge
```

---

## 4. 핵심 파일

### 4.1 현재 활성 path (SimpleFlight + 직접 Python)

| 파일 | 역할 |
|---|---|
| [`scripts/run_phase4_delta_clean.sh`](../scripts/run_phase4_delta_clean.sh) | orchestrator: settings deploy → UE Stop/Play 안내 → Python 실행 |
| [`scripts/phase4_delta_simple.py`](../scripts/phase4_delta_simple.py) | **핵심 control script**. AirSim API 직접, ROS2 없음 |
| [`settings/sf_3drones_phase4_delta.json`](../settings/sf_3drones_phase4_delta.json) | SimpleFlight 3대 (drone1/2/3, X=0/5/10) |
| [`scripts/run_ue_blocksv2.sh`](../scripts/run_ue_blocksv2.sh) | UE Editor 자동 시작 (BlocksV2.uproject) |

### 4.2 보존된 다른 path (현재 미사용, 참고용)

- PX4 stack: `scripts/run_phase4_delta.sh`, `settings/px4_3drones_phase4_delta.json`, `airsim_ros2_bridge/scripts/mavros_arm_all.py`, `mavros_force_arm.py` — Phase 5 EKF2 fix (`docs/2026-05-21_phase5_ekf2_root_cause.md`) 적용된 path 지만 multi-vehicle arm 안 됨
- ROS2 formation_node stack: `airsim_ros2_bridge/airsim_ros2_bridge/formation_node.py`, `formation_demo.py`, `aerion_phase4_formation.launch.py` — Phase 4-Δ spec/plan 의 실 구현 (drone_controller bridge 의 moveToPositionAsync 호출이 출렁거림 유발, 단편적 fix 안 통함)

### 4.3 Phase 4-Δ 설계 문서 (현재 SF path 와 부분 일치)

- spec: `docs/superpowers/specs/2026-05-22-phase4-delta-3drone-formation-design.md`
- plan: `docs/superpowers/plans/2026-05-22-phase4-delta-formation.md`
- runbook: `docs/phase4_delta_runbook.md`
- Phase 5 EKF2 doc: `docs/2026-05-21_phase5_ekf2_root_cause.md`

---

## 5. 시도 이력 (시간순)

| 시점 | 시도 | 결과 |
|---|---|---|
| 2026-05-21 | PX4 5대 stack + EKF2 fix (commit `85b8e96`) | 1대 isolated console 에서 EKF fusion 동작 검증. multi-vehicle arm 까지 안 가봄 |
| 2026-05-22 brainstorming | Phase 4-Δ spec + plan + 5 tasks 구현 (commits `d503c0f` ~ `3b77d2d`) | formation_node + bridge + launch + tests + docs 완성. push 완료 |
| 2026-05-22 PX4 runner 시도 | `run_phase4_delta.sh` 로 PX4 3대 + mavros + arm | arm 모두 `result=1 (TEMPORARILY_REJECTED)`. settings.json 의 EKF2_MAG_TYPE 등 새 param 이 Colosseum→PX4 PARAM_SET 채널로 전달 안 됨. force-arm magic 도 거부. |
| 2026-05-22 SF backend | `run_phase4_delta_sf.sh` + bridge airsim_direct backend | bridge 의 moveToPositionAsync 호출이 매 20Hz setpoint 마다 P-controller reset → 출렁거림. velocity/rate-limit fix 도 충분치 않음 |
| 2026-05-22 Clean Python | `phase4_delta_simple.py` (ROS2 우회) | 첫 시도부터 다양한 실패 — SteppableClock hang, world↔local 변환 잘못, 추락 등 |
| 2026-05-23 SteppableClock 제거 | settings 에서 ClockType 제거 (commit `7ea53be`) | SteppableClock 이 PX4 lockstep 전용임 확인. SimpleFlight 는 default ScalableClock. 그 후 첫 패턴은 정상, 두 번째부터 일부 vehicle freeze |
| 2026-05-23 enableApi reassert | 매 명령 직전 `enableApiControl(True)` (commit `20940b0`) | drone2/3 가 호출 직후 추락 — controller disrupt |
| 2026-05-23 world↔local 변환 추가 | (commit `8bbe3a0`) | **잘못된 가설**. 웹 검색으로 moveToPositionAsync 가 world NED 임 확인. revert (`ef2c3cf`) |
| 2026-05-23 hover 버그 우회 | hold 동안 1Hz target 재발행 + enableApi 재호출 제거 (commit `f87b443`) | **추락 멈춤 ✓**. 그러나 drone2/3 가 여전히 일부 패턴에서 freeze (아래 §6 좌표 분석) |

---

## 6. 현재 상태 — 좌표 분석 (commit f87b443 시연 결과)

사용자 직전 시연의 정확한 좌표:

| 단계 | drone1 도달 | drone2 도달 | drone3 도달 |
|---|---|---|---|
| Step 2 hover | (0, 0, -4.95) ✓ | (5.30, -0, -4.69) ✓ | (10.24, +0, -4.58) ✓ |
| **P1 TRIANGLE** target → 도달 | (0, 0, -5) → (0, 0, -4.94) ✓ | (-1.70, -1, -5) → **(-1.77, -1.01, -4.84) ✓** | (-1.70, +1, -5) → **(-1.75, +1.01, -4.68) ✓** |
| **P2 V3** target → 도달 | (0, 0, -5) → (0, 0, -4.94) ✓ | (-1.50, -1.50, -5) → **(-1.85, -1.02, -4.85) ✗ P1 위치 그대로** | (-1.50, +1.50, -5) → **(-1.80, +1.01, -4.69) ✗ P1 위치** |
| **P3 COLUMN** target → 도달 | (0, 0, -5) → (0, 0, -4.94) ✓ | (-2, 0, -5) → **(-1.84, -1.02, -4.85) ✗ 여전히 P1** | (-4, 0, -5) → **(-4.07, -0.03, -4.84) ✓ 다시 동작** |
| **P4 DIAMOND3** target → 도달 | (0, 0, -6) → **(0, 0, -4.94) ✗ z 안 변함** | (-1.5, -1, -5) → (-1.84, -1.02, -4.85) ✗ | (-1.5, +1, -5) → **(-1.46, +1.02, -5.00) ✓** |
| **P5 TRIANGLE** target → 도달 | (0, 0, -5) → (0, 0, -4.94) ✓ | (-1.70, -1, -5) → (-1.84, -1.02, -4.85) ✗ | (-1.70, +1, -5) → (-1.42, +1.03, -5.01) ✓ |

**패턴**:
- **drone1**: xy 모든 패턴 도달. z 는 P4 DIAMOND3 의 -6 target 못 도달 (-4.94 유지) — 별 이슈
- **drone2**: P1 도달 후 **모든 후속 패턴에서 (-1.84, -1.02, -4.85) 영구 freeze**
- **drone3**: P1 도달, P2 freeze, P3 부터 다시 동작 (intermittent)

= **drone2 가 완전 freeze, drone3 가 intermittent freeze**. 1Hz stream 재발행 + enableApi 안 재호출 fix 가 충분치 않음. 추락은 멈췄지만 multi-vehicle 명령 처리 자체에 SimpleFlight 측 한계.

---

## 7. 미해결 root cause (가설)

검색으로 확인된 알려진 issues + 좌표 패턴이 시사하는 것:

1. **AirSim Issue #1538**: RPC 서버 `pimpl_->server.async_run(4)` 4-thread 한계. 3대 < 4 이라 thread 자체는 충분하지만 동시 명령 처리 race condition 가능. (`microsoft/AirSim` upstream 에서 `262cb28` 으로 fix 되었지만 Colosseum fork 미반영)
2. **AirSim Issue #991**: SimpleFlight hover() 버그 — buggy hover. 우리는 stream 으로 우회
3. **AirSim Issue #4421**: moveToPositionAsync 완료 후 shaky — 우리는 stream 으로 일부 우회
4. **알려지지 않은**: SimpleFlight multi-vehicle 환경에서 **첫 명령 처리 후 일부 vehicle 의 SimpleFlight controller 가 후속 moveToPositionAsync 명령을 silent ignore**. `.join()` 은 정상 complete 응답하지만 실제 controller 가 처리 안 함. drone1 spawn=(0,0) 이라 항상 동작, drone2/3 (spawn 더 멀리) 가 affected.

좌표 패턴이 **stream 자체는 발행되지만 SimpleFlight 가 무시** 를 시사. `.join()` 이 즉시 complete 하면 (도달 판정) → drone 이 이전 위치에 머무름.

---

## 8. 다음 시도 후보 (Codex 가 진행 권장)

### 8.1 단기 우회 (시연 우선)

**(A) `simSetVehiclePose` 텔레포트 시연** (Recommended for 빠른 시연)
- moveToPositionAsync 대신 `client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=v)` 로 직접 위치 설정
- physics 우회 (텔레포트), 비행 dynamics 손실
- 모든 vehicle 항상 정확히 도달 (controller 의존 없음)
- 시연/screenshot 용으로 적합
- 코드 위치: `scripts/phase4_delta_simple.py` 의 패턴 루프에서 moveToPositionAsync 호출을 simSetVehiclePose 로 교체

**(B) vehicle 별 별 client + multi-thread**
- `airsim.MultirotorClient()` 인스턴스를 vehicle 별로 분리 + 각자 thread 에서 명령
- RPC 호출이 vehicle 별 독립 → SimpleFlight controller race 회피 가능
- 코드 변경 큼 (전체 재구조)

### 8.2 장기 — 진짜 root cause 진단

**(C) Colosseum source 의 multi-vehicle SimpleFlight 명령 처리 코드 분석**
- 위치: `~/airsim/Colosseum/AirLib/include/vehicles/multirotor/firmwares/simple_flight/SimpleFlightApi.hpp` + 관련
- moveToPosition 호출이 internal queue 또는 controller state 에서 drone1 우선 처리 + drone2/3 무시되는 코드 path 검색
- 시간 소요 (하루~), 가능하면 fix PR 가능

**(D) Issue #1538 의 `async_run(num_vehicles)` patch 이식**
- Colosseum 의 `RpcLibServerBase.cpp:260` 를 `pimpl_->server.async_run(num_vehicles)` 로 변경 + recompile
- 4-thread 한계 해소. 그러나 SimpleFlight controller race 가 그것 때문인지 미확정

### 8.3 환경 변경

**(E) PX4 stack 으로 회귀 + Phase 5 EKF2 fix 추가 진단**
- PX4 multi-vehicle 의 동시 명령 처리는 PX4 firmware 가 담당 (AirSim RPC race 회피)
- 다만 Colosseum settings.json Parameters → PX4 PARAM_SET 채널 미동작 문제 남음. Colosseum source 디버깅 필요

---

## 9. 즉시 시작 명령 (Codex 가 이어받을 때)

```bash
cd ~/workspace/projects/aerion-airsim
git pull                                       # f87b443 + handoff doc

# UE Editor 시작 (별 터미널)
bash scripts/run_ue_blocksv2.sh                # BlocksV2.uproject 자동 open
# GUI 에서 ▶ Play 누름

# 현재 SF path 재현 (drone2/3 freeze 증상 재현)
bash scripts/run_phase4_delta_clean.sh         # f87b443 의 동작 그대로

# Option A 권장 시도 — simSetVehiclePose 텔레포트
# (scripts/phase4_delta_simple.py 의 패턴 루프 moveToPositionAsync 를
#  simSetVehiclePose 로 교체 + Step 2 의 hover 도 같은 방식. 별 새 파일
#  scripts/phase4_delta_teleport.py 만드는 게 더 안전 — 기존 SF path 보존)
```

---

## 10. 사용자 메타 정보

- 사용자: 김영훈 (seongwon.jo@moreh.io)
- 사용자 메모리 위치: `~/.claude/projects/-home-clrobur-workspace-projects/memory/MEMORY.md` (Claude Code 의 auto-memory)
- 협업 패턴 메모리: 한국어, 결론 먼저, 짧고 밀도 있게, 명령어 복붙 가능, 위험작업 백업 우선
- 현재 frustration 상태 — 단편적 fix 시도 누적이 원인. **새 접근은 검증된 패턴 (공식 예제 / 검색된 fix) 우선 + 추측 fix 최소화** 권장.

---

## 11. 관련 commits (push 완료)

```
f87b443 fix(sim): drop enableApi reassert + 1Hz target re-issue during hold (SimpleFlight hover bug workaround)
ef2c3cf fix(sim): revert world↔local conversion — moveToPositionAsync is world NED per AirSim docs
8bbe3a0 fix(sim): world↔local frame conversion for multi-vehicle moveToPosition (real root cause)  [잘못된 가설, ef2c3cf 가 revert]
20940b0 fix(sim): re-assert enableApiControl(True) before every pattern move (drone2/3 freeze fix)  [추락 부작용 발생]
3b68b5d fix(sim): drop --lookahead from wrapper (removed in 7ea53be rewrite)
7ea53be fix(sim): rewrite simple runner — AirSim official pattern, drop ClockType SteppableClock
a02975b fix(sim): reacquire only re-asserts API control — no armDisarm/cancel (drop fix)
57bc9ac fix(sim): clean runner — lazy reacquire + join timeout + fixed yaw
6239e6f feat(sim): clean slate Phase 4-Δ runner — single Python, AirSim API direct
1197c6c feat(sim): SimpleFlight backend runner — PX4 stack 우회
...
3b77d2d docs(fix): phase4-Δ final review (Phase 4-Δ ROS2 stack 의 design + 5 task 구현 완료)
85b8e96 fix(px4): remove EKF2_AID_MASK/GPS_CHECK that disabled EKF2 fusion (Phase 5)
```

---

## 12. 검색 결과 references (Codex 가 더 깊은 진단 시 참고)

- [AirSim multi_agent_drone.py](https://github.com/microsoft/AirSim/blob/main/PythonClient/multirotor/multi_agent_drone.py) — moveToPositionAsync 의 좌표가 world NED 임 확인 source
- [AirSim Multi-Vehicle docs](https://microsoft.github.io/AirSim/multi_vehicle/) — multi-vehicle 기본
- [Issue #1538 — RPC 4-thread 한계](https://github.com/microsoft/AirSim/issues/1538) — fix `262cb28` upstream merged
- [Issue #991 — SimpleFlight hover() bug](https://github.com/microsoft/AirSim/issues/991)
- [Issue #4421 — Multirotor shaky after moveToPosition](https://github.com/microsoft/AirSim/issues/4421)
- [Issue #1956 — Follow-leader pattern](https://github.com/microsoft/AirSim/issues/1956) — leader-follow 코드 참조

---

**Codex 시작 추천**: `scripts/phase4_delta_simple.py` 의 `moveToPositionAsync` 호출을 `simSetVehiclePose` 로 교체한 별 파일 `scripts/phase4_delta_teleport.py` 작성 → 즉시 시연 동작 → 그 후 root cause 진단 (Option C/D) 별도 진행.
