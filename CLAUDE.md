## Airsim Colosseum + UE 5.6 + PX4/APM SITL + ROS2 시뮬레이션 환경

**설계문서:** `docs/superpowers/specs/2026-04-07-colosseum-px4-sitl-ros2-design.md`

## 환경 정보
- OS: Ubuntu 22.04.5 LTS
- UE 5.6.1: `/home/clrobur/airsim/unreal-engine/` (설치완료)
- GPU: NVIDIA (Driver 580, CUDA 13.0)
- Colosseum: `main` 브랜치 (UE 5.6 지원) -- 빌드완료 (libAirLib.a 생성됨)
- 컴파일러: clang-18 설치완료 (Ubuntu clang 18.1.8)
- Python: 3.12.13 (CPython, uv 관리) -- `.venv/` 에 설치완료 (uv 0.10.12)
- cmake: 설치됨 (`/usr/bin/cmake`)
- uv: 설치됨 (`/home/clrobur/.local/bin/uv`) -- Python 가상환경/패키지 관리에 활용 가능

## 구축 작업 항목

### Phase 1: 기반 설치
- [x] 1.1 clang-18, libc++-18-dev 설치 (Ubuntu clang 18.1.8 확인됨)
- [x] 1.2 Python 3.12+ 설치 (uv venv, CPython 3.12.13 확인됨)
- [x] 1.3 Colosseum clone → setup.sh → build.sh (libAirLib.a 생성 확인됨)
- [x] 1.4 PX4-Autopilot clone → `make px4_sitl_default none_iris` (px4 바이너리 확인됨)
- [x] 1.5 airsim Python 패키지 설치 (airsim 1.8.1, editable install 확인됨)

### Phase 2: PX4 SITL 2대 시뮬레이션 검증
- [x] 2.1 settings.json 구성 (px4_dual.json 생성 확인됨, launch 스크립트 2개 존재)
- [x] 2.2 BlocksV2.uproject EngineAssociation → "5.6" 확인됨
- [x] 2.3 UE 에디터 + BlocksV2 실행 — C++ 빌드 성공, GUI 정상 표시
- [x] 2.4 PX4 SITL 2개 인스턴스 기동 — TCP 4560/4561 연결 확인 (PX4_SIM_MODEL=none_iris 필수)
- [x] 2.5 드론 스폰 + PX4 연결 확인 — 2대 스폰, API 제어/이륙/이동 성공
- [ ] 2.6 원격 MAVROS에서 MAVLink 수신 확인 (LocalHostIp 변경 후 테스트 필요)

### Phase 3: Python ROS2 브릿지
- [x] 3.1 Python 브릿지 노드 구현 (rclpy + airsim client) — colcon 빌드 완료
- [x] 3.2 /camera/image 토픽 발행 — 1280x720 RGB 정상 발행 확인 (10fps)
- [x] 3.3 /camera/camera_info 토픽 발행 — K=[641,0,640,0,641,360,0,0,1], plumb_bob 확인
- [x] 3.4 드론 제어 토픽 구독 → airsim API 연동 — cmd_vel, cmd_pos 수신 대기 확인
- [ ] 3.5 원격 embodied-drone 노드와 통합 테스트 (ROS2 DDS 네트워크 설정 필요)

### Phase 4: ArduPilot(APM) SITL 구축
- [x] 4.1 settings.json에서 VehicleType → "ArduCopter" 전환 — apm_dual.json 생성 완료
- [x] 4.2 ArduPilot 설치 + 빌드 완료 (~/ardupilot, ArduCopter SITL)
- [x] 4.3 APM SITL ↔ Colosseum 연결 성공 — 바이너리 직접 실행 + --serial0 udpclient
- [ ] 4.4 원격 MAVROS와 MAVLink 연동 확인

### Phase 5: 도시 환경 + 장애물 배치
- [ ] 5.1 UE 마켓플레이스 도시 에셋 설치
- [ ] 5.2 새 UE 프로젝트에 AirSim 플러그인 + 도시 맵 구성
- [ ] 5.3 장애물 배치
- [ ] 5.4 전체 파이프라인 도시 환경에서 검증

## 핵심 결정사항
- MAVLink 전송: settings.json `ControlPortRemote`로 직접 UDP 전송 (14555/14556)
- ROS2 브릿지: Python 기반 (rclpy + airsim client), C++ ros2 패키지 빌드 안함
- 환경: BlocksV2로 파이프라인 검증 우선 → 추후 도시 에셋 교체
- SITL 순서: PX4 먼저 → APM 추후 (혼합 비권장, ClockType 충돌)

## 포트 매핑

### PX4 SITL 2대
| 용도 | Drone0 | Drone1 |
|------|--------|--------|
| SITL↔Colosseum TCP | 4560 | 4561 |
| MAVLink→MAVROS UDP | 14555 | 14556 |
| MAV_SYS_ID | 1 | 2 |

### APM SITL 2대 (Phase 4)
| 용도 | Drone0 (instance 0) | Drone1 (instance 1) |
|------|--------|--------|
| Sensor UDP (UdpPort) | 9003 | 9013 |
| Control UDP (ControlPortLocal) | 9002 | 9012 |
| MAVLink→MAVROS UDP (--out) | 14555 | 14556 |
| 참고: ArduPilot --instance N은 포트에 N*10 오프셋 적용 | | |

## 참고 문서
- Colosseum repo: https://github.com/CodexLabsLLC/Colosseum
- Colosseum 문서: https://codexlabsllc.github.io/Colosseum/
- Settings: https://codexlabsllc.github.io/Colosseum/settings/
- PX4 SITL: https://codexlabsllc.github.io/Colosseum/px4_sitl/
- PX4 Multi Vehicle: https://codexlabsllc.github.io/Colosseum/px4_multi_vehicle/
- Image API: https://codexlabsllc.github.io/Colosseum/image_apis/
- API docs: https://codexlabsllc.github.io/Colosseum/apis/
- Python 예제: https://github.com/CodexLabsLLC/Colosseum/tree/main/PythonClient/multirotor
- 파이썬은 3.12 이상을 사용

---

## 원본 메모 (기존 작성 내용)

## Airsim Colosseum Unreal engine 5.6 또는 5.3 구축

## https://github.com/CodexLabsLLC/Colosseum

- git : https://github.com/CodexLabsLLC/Colosseum.git
- unreal engine 5.6 : https://dev.epicgames.com/documentation/ko-kr/unreal-engine/linux-development-quickstart-for-unreal-engine?application_version=5.6
- Engine/Binaries/Linux/UnrealEditor 실행 시 언리얼 엔진 실행됨.


나는 sitl + px4 2대 + airsim colosseum의 5.6/5.3 언리얼엔진 시뮬레이션 환경을 우분투 22.04 여기서 구축해야돼.

우선 이 목표를 위한 구체적인 계획을 세우고, 자료조사를 해서 최적의 계획과 단계별로 구축을 할 수 있도록 해. 맨 아래 링크들을 다 방문하고 자료 및 문서 읽어 파악한 후 전문가적인 구체적이고 상세한 계획을 세워 보고해야해.


우선 나는 mavros로 별도의 컴퓨터에서 ros2 node들을 embodied-drone이라고 체화지능 엣지단을 구현했어. 여기는 본래 ardupilot + gazabo 로 구축되어있어.
나는 mavros단은 살리고 이 ardupilot sitl + gazebo는 중지할거야,
본래 /camera/image topic 으로 시뮬레이션하는 카메라 이미지 데이터가 들어오거든, 그외 카메라 렌즈스펙 파라미터 정보도 카메라 토픽들로 들어오고,

너가 알야야 할것은 궁극적 우리의 목표는 아래와 같아. 

1. airsim colosseum을 unreal engine 5.6 스펙이라고하니 이 우분투 22.04에서 unreal engine 5.6 및 기타 필요 설치+설정을 하는 것.

2. 상대 ros2 mavros node 컴퓨터로 udp 14555 port에 colosseum 안에서 시뮬레이션되는 sitl px4의 mavlink data를 드론별 송신하는 채널 1개를 설정/구축하는 것 2대면 14555, 14556 등이 되겠지, 아마 설정파일에 설정가능한 것으로 알아


3. 아래 기능들을 수행하기위해 제일 강력하고 편하게 구축하는 방법을 결정 (https://github.com/CodexLabsLLC/Colosseum/tree/main/ros2/src 이 패키지를 빌드할지, 파이썬 브릿지하나 구현해서 ros2 측에 토픽으로 송신할지 등, 상대 ros2측은 건들지 않는다는 전제)

- 실시간 드론의 카메라 스펙, 렌즈 등 기본 필요 카메라 파라미터를 받고 기존 ros2 /camera관련 info topic에 매핑해서 송신

- 실시간 드론의 카메라 이미지데이터를 추출해서 실시간으로 ros2 /camera/image topic으로 송신 (gazebo 에서 쏘는 것처럼)

- ros2 topic으로 드론 제어가능하도록 브릿지 또는 방안 구성

- 언리얼 엔진위의 airsim 콜로세움에서 도시 세계를 만들고 내가 원하는 장애물들을 배치하는 것


자료를 조사해서 현재 필요한 정보와 모르는 문제를 해결하는 서브에이전트를 두고,

진행에 따라 문서화와 통신 구조들을 검토/기록 업데이트하는 서브에이전트를 둬.

콜로세움 구축하기 (최신아님, Ue 5.2이나 5.6도 비슷할 거라 예상)
- https://www.countinglab.co.uk/post/mastering-drone-simulation-a-step-by-step-guide-to-using-microsoft-s-airsim
- VS 2022 또는 그외 컴파일러 들이 필요할 것으로 예상, (자료조사 필요)

콜로세움 일반 문서
- https://codexlabsllc.github.io/Colosseum/
- https://codexlabsllc.github.io/Colosseum/apis/
- https://codexlabsllc.github.io/Colosseum/settings/


콜로세움을 위한 px4 설정
- https://codexlabsllc.github.io/Colosseum/px4_setup/

콜로세움 px4 SITL
- https://codexlabsllc.github.io/Colosseum/px4_sitl/

콜로세움 px4 multi vehicle sim
- https://codexlabsllc.github.io/Colosseum/px4_multi_vehicle/



이미지 API
- https://codexlabsllc.github.io/Colosseum/image_apis/



콜로세움 릴리즈
- https://github.com/CodexLabsLLC/Colosseum/releases

- 파이썬은 3.12 이상을 사용,

기타 카메라 및 드론 제어 예제들 (파이썬로 가능하면 모두 파이썬기반 구축)

- https://github.com/CodexLabsLLC/Colosseum/blob/main/PythonClient/multirotor/hello_drone.py
- https://github.com/CodexLabsLLC/Colosseum/blob/main/PythonClient/computer_vision/cv_mode.py
- https://github.com/CodexLabsLLC/Colosseum/tree/main/PythonClient/multirotors

<!-- FABLIZE:BEGIN — run Opus like Fable (always-on router). Verified procedures only. Install/update: fablize setup.sh -->
## Operating mode (always on — auto-route by task signal)

Apply what the task signals; with no signal, baseline only. Read each pack only when needed. Routing: smallest matching discipline only, overlap only when genuinely multi-category, mimic observable behavior only.

- **[always]** Lead with the outcome · stay within the requested scope (no incidental refactors) · ground completion claims in this session's tool results · confirm before destructive or hard-to-reverse actions.
- **[2+ sequential stories]** Run `python3 /home/clrobur/.claude/plugins/cache/fablize/fablize/2.0.0/scripts/goals.py`: create → next → checkpoint (with evidence) → final verification gate (no completion without `--verify-cmd` and `--verify-evidence`). Run from the repo root; state in `./.fablize/` (resume with `status`). Skip for single-step tasks.
- **[debugging / test failure / unknown cause / review]** Follow `/home/clrobur/.claude/plugins/cache/fablize/fablize/2.0.0/packs/investigation-protocol.txt`: reproduce first → 3+ competing hypotheses → evidence per hypothesis → full causal chain → verify before/after → report rejected hypotheses.
- **[render/executable artifact: HTML, SVG, game, UI, chart]** Follow `/home/clrobur/.claude/plugins/cache/fablize/fablize/2.0.0/packs/verification-grounding-pack.txt` grounding loop: run it in the real renderer → observe the output → fix what you see → re-run. A static check is not observation.
- **[hard or ambiguous task]** Adaptive thinking scales with difficulty automatically. To go higher, recommend `/effort xhigh` to the user. Depth (capability) cannot be raised: if stuck 2+ times or out-of-spec discovery is needed, report the limit honestly and escalate.
<!-- FABLIZE:END -->

<!-- AUTOCAPTURE:BEGIN — 자율 지식 축적 규칙 (페이블식). fablize 블록 밖이므로 setup.sh가 건드리지 않음. 손으로 편집 가능 -->
## 자율 지식 축적 규칙 (always on — 실시간·무인)

작업·대화 도중 **지속 가치가 있는 지식이 발견되면, 묻지 말고 실시간으로** 알맞은 저장소에 기록한다. 사용자가 신경 쓰지 않아도 지식이 자동 축적·동기화되게 하는 것이 목적. (memoir Stop 훅의 자동 캡처와 *별개로* 작동하는 행동 규율)

**1) 포착 신호 (이게 보이면 캡처 검토):**
- **결정** — "X로 간다, 이유는 Y" (설계·기술 선택)
- **제약/관례 발견** — "반드시 …", "…하면 안 됨", 포트/버전/경로 규칙
- **교훈/함정** — 무언가 깨졌다가 원인+해결을 찾았을 때
- **사용자 교정/피드백** — 내 방식에 대한 지적·승인
- **사용자 선호/금지** — 개인 선호·정체성 (⚠️ 민감)
- **마일스톤/상태 변화** — 장기 작업의 단계 전환·핸드오프 시점

**2) 라우팅 (어디에 쓰나):**
| 성격 | 위치 |
|---|---|
| 이 작업 한정·휘발성 | **기록 안 함** (노이즈 방지) |
| 세션 간 사실·선호 | 파일메모리 카드(`memory/<slug>.md`) + `MEMORY.md` 인덱스 1줄 |
| 구조화 기술사실 | memoir (Stop 자동; 즉시 필요하면 `/memoir:remember`) |
| 모두가 따를 프로젝트 룰 | **이 CLAUDE.md** (해당 섹션에 추가) |
| 서사형 진행·하우투·핸드오프 | `docs/` (예: `docs/fable/`, `docs/airsim/`) |

**3) 자율성 정책 (공격적 실시간):**
- **비민감**(결정·교훈·제약·상태·교정) → **즉시 무인 기록**, 응답에 `📝 기록: …` 한 줄만 남김.
- **민감**(개인 선호·금지·정체성) → **먼저 통지 → 승인 후** 저장.
- **중복 금지**: 저장 전 기존 항목 확인, 새로 만들기보다 **갱신** 우선. 틀린 기억은 삭제.
- **노이즈 금지**: 레포/깃/CLAUDE.md가 이미 담은 내용은 저장하지 않는다(무엇이 *비자명*했는지만 남김).
- **링크**: 파일메모리 카드끼리 `[[slug]]`로 연결.
<!-- AUTOCAPTURE:END -->
