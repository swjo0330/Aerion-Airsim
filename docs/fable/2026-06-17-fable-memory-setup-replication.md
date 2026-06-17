# 페이블식 자율 기억 셋업 — 재현 가이드 (Ubuntu / aerion-airsim 기준)

> **2026-06-17 작성, 동일자 실측 검증 후 전면 개정(v2).**
> 본 프로젝트(`aerion-airsim`)의 "페이블식 자율 기억 셋업" —
> **fablize 오퍼레이팅 모드 + memoir 스토어 + 파일 메모리(MEMORY.md) + 프로젝트 규칙(CLAUDE.md)** —
> 을 다른 컴퓨터의 Claude Code에 재현하기 위한 가이드.
>
> ⚠️ 개정 사유: 초판(v1)은 macOS 경로/`~/.remember` 스토어/fablize v5 등
> **이 머신의 실제 구성과 다른 내용**이 다수였음. 본 v2는 이 머신(Ubuntu 22.04, 사용자 `clrobur`)에서
> 실제 플러그인 파일과 경로를 검증해 작성함.

---

## 0. 이 머신의 실제 환경 (기준값)

| 항목 | 값 |
|---|---|
| OS | Ubuntu 22.04 (Linux), 사용자 `clrobur`, 홈 `/home/clrobur` |
| 프로젝트 경로 | `/home/clrobur/workspace/projects/aerion-airsim` |
| git 원격 | `git@github.com:swjo0330/Aerion-Airsim.git` (SSH) |
| fablize | **v2.0.0** (`~/.claude/plugins/cache/fablize/fablize/2.0.0`) |
| memoir | **v0.3.0** (`~/.claude/plugins/cache/memoir/memoir/0.3.0`) |

> macOS의 `pbcopy`/`/Users/...` 대신 Linux에서는 `xclip -selection clipboard`(X11) 또는
> `wl-copy`(Wayland)를 쓰거나, 그냥 에디터로 직접 편집한다.

**현재 이 머신의 셋업 상태(실측):**

| 구성요소 | 상태 | 비고 |
|---|---|---|
| fablize 오퍼레이팅 블록 (CLAUDE.md 내 `<!-- FABLIZE -->`) | ❌ 미설정 | `/fablize:setup` 필요 |
| `~/.fablize/progress.json` | ❌ 없음 | setup 시 생성 |
| memoir 스토어 `~/.memoir/...` | ❌ 미초기화 | `/memoir:onboard` 필요 |
| 파일 메모리 `.../memory/MEMORY.md` | ❌ 없음 | 첫 기억 저장 시 생성 |
| 전역 `~/.claude/CLAUDE.md` | ❌ 없음 | 전역 규칙 미사용 |
| 프로젝트 `CLAUDE.md` | ✅ 존재 | git에 포함됨 |

즉, **이 머신도 아직 fablize/memoir 초기화 전**이다. 아래 절차는 이 머신을 포함한 모든 머신에 동일 적용된다.

---

## 1. 셋업의 4개 구성요소

| # | 구성요소 | 실제 위치 | git clone으로 따라오나 | 설명 |
|---|---|---|---|---|
| 1 | **CLAUDE.md** (프로젝트 규칙 + fablize 블록) | `aerion-airsim/CLAUDE.md` (로컬) / `~/.claude/CLAUDE.md` (전역) | 프로젝트분만 ✅ | 프로젝트 규칙은 git 포함. fablize 블록은 `/fablize:setup`이 주입 |
| 2 | **파일 메모리** (선호/정체성/서사 인덱스) | `~/.claude/projects/-home-clrobur-workspace-projects-aerion-airsim/memory/MEMORY.md` + 개별 `.md` | ❌ | 세션 간 기억. 머신별 고유, 수동 rsync로 이전 |
| 3 | **memoir 스토어** (구조화 기술사실) | `~/.memoir/-home-clrobur-workspace-projects-aerion-airsim/` | ❌ | **git 기반 프로젝트별 스토어**. `derive-store-path.sh`가 프로젝트 경로로 디렉토리명 산출 |
| 4 | **docs/** (서사형 기록) | `aerion-airsim/docs/` (예: `docs/fable/`, `docs/airsim/`, `docs/superpowers/specs/`) | ✅ | git에 포함, clone 직후 자동 |

> **memoir 스토어 경로는 자동 계산된다.** 직접 만들지 말 것:
> ```bash
> bash ~/.claude/plugins/cache/memoir/memoir/0.3.0/scripts/derive-store-path.sh
> # → /home/clrobur/.memoir/-home-clrobur-workspace-projects-aerion-airsim
> ```
> 프로젝트 절대경로의 `/`를 `-`로 치환한 이름이다. 머신마다 경로가 다르면 디렉토리명도 달라진다.

---

## 2. 타 머신 셋업 절차 (3단계)

### 2.1 단계 1 — 플러그인 설치 및 프로젝트 복제

```bash
# (1) Claude Code에서 fablize + memoir 플러그인 설치 (UI 또는 /plugin)
#     /plugin marketplace add fivetaku/fablize  → /plugin install fablize@fablize
#     /plugin marketplace add zhangfengcdt/memoir → /plugin install memoir@memoir
#     (academic-research-skills 등 추가 스킬은 선택)

# (2) git clone (SSH 키가 등록되어 있어야 함)
mkdir -p ~/workspace/projects
cd ~/workspace/projects
git clone git@github.com:swjo0330/Aerion-Airsim.git aerion-airsim
cd aerion-airsim
```

> - 플러그인 설치 후 **`/reload-plugins`** 또는 Claude Code 재시작으로 적용.
> - fablize의 UserPromptSubmit 라우터 훅은 `hooks.json`에 의해 **설치 시 자동 등록**된다 (settings.json 직접 수정 불필요).
> - HTTPS를 쓰려면 `https://github.com/swjo0330/Aerion-Airsim.git` 사용.

---

### 2.2 단계 2 — fablize 오퍼레이팅 모드 활성화

Claude Code에서:

```
/fablize:setup
```

**`/fablize:setup`이 실제로 하는 일 (v2.0.0):**

1. ✅ 설치 범위를 한 번 묻는다 — **Local(이 프로젝트만, 권장) / Global(모든 프로젝트) / Cancel**
2. ✅ `setup.sh` 실행:
   - 대상 CLAUDE.md 백업 (`CLAUDE.md.fablize-bak.<ts>`)
   - `<!-- FABLIZE:BEGIN ... FABLIZE:END -->` 오퍼레이팅 블록을 멱등 주입
     (Local = `프로젝트/CLAUDE.md`, Global = `~/.claude/CLAUDE.md`)
   - `~/.fablize/progress.json` 기록 (재실행 방지)
   - `gh`로 fablize repo star (로그인 안 됐거나 이미 star면 조용히 건너뜀)
3. ❌ **memoir 초기화는 하지 않는다** — 그건 단계 3의 `/memoir:onboard` 몫.
4. ❌ **자동기록 기능을 켜지 않는다** — fablize는 *작업 규율 라우터*(결과 우선·다단계 goals.py·조사 프로토콜·렌더 검증)일 뿐.

---

### 2.3 단계 3 — memoir 스토어 초기화

Claude Code에서:

```
/memoir:onboard
```

- `~/.memoir/<프로젝트경로>/`에 git 기반 스토어를 생성하고 코드베이스 개요를 기록한다.
- 이후 세션 종료(Stop) 훅으로 자동 기억 캡처가 동작한다.
- 검증: `/memoir:status`

---

### 2.4 단계 4 (선택) — 기존 기억 이전

경험까지 그대로 재현하려면 원본 머신의 기억을 복사한다. **경로는 머신마다 다르므로 실제 경로로 치환**할 것.

```bash
# 원본 머신 → 신규 머신 (예: scp/rsync over ssh)

# (a) 파일 메모리
rsync -avz \
  ~/.claude/projects/<원본-인코딩경로>/memory/ \
  <신규>:~/.claude/projects/<신규-인코딩경로>/memory/

# (b) memoir 스토어 (git 기반이므로 디렉토리 통째로)
rsync -avz \
  ~/.memoir/<원본-인코딩경로>/ \
  <신규>:~/.memoir/<신규-인코딩경로>/
```

> 인코딩경로는 양쪽 머신에서 각각 `derive-store-path.sh`로 확인한다(홈 경로가 다르면 값도 다름).
> memoir 스토어는 git 저장소이므로, 더 안전하게는 원본에서 별도 원격에 push → 신규에서 clone 하는 방식도 가능.

---

## 3. 구성요소별 동기화 규칙

### 3.1 CLAUDE.md
- 프로젝트 규칙(`aerion-airsim/CLAUDE.md`)은 **git으로 동기화** — clone 시 자동.
- fablize 블록은 **각 머신에서 `/fablize:setup`으로 주입** (수동 복사 불필요, 멱등).
- 전역 규칙이 필요하면 `~/.claude/CLAUDE.md`에 따로 작성 — 프로젝트 규칙이 전역을 덮어쓴다.

### 3.2 파일 메모리 (MEMORY.md + 개별 .md)
- 프로젝트별 고유 인덱스. 새 머신은 **빈 상태로 시작 가능**.
- 이전하려면 `memory/` 디렉토리 전체를 rsync. `MEMORY.md`는 한 줄 인덱스이므로 병합 시 중복만 제거.

### 3.3 memoir 스토어
- `~/.memoir/<프로젝트경로>/` — **git 기반**. 개별 파일을 손으로 만들지 말 것.
- 이전: 디렉토리 rsync 또는 git push/clone.
- 검증: `/memoir:status` (스토어 경로·브랜치·기억 수 표시).

### 3.4 docs/
- `git clone` 시 모두 따라옴. 이후 `git pull`/`push`로 동기화.
- 확인: `git -C ~/workspace/projects/aerion-airsim log --oneline -- docs/ | head`

---

## 4. 재현 완료 확인 체크리스트

```bash
# (1) 파일/디렉토리 존재
ls -la ~/workspace/projects/aerion-airsim/docs/
bash ~/.claude/plugins/cache/memoir/memoir/0.3.0/scripts/derive-store-path.sh   # memoir 스토어 경로
ls -la "$(bash ~/.claude/plugins/cache/memoir/memoir/0.3.0/scripts/derive-store-path.sh)"
ls -la ~/.fablize/progress.json
grep -c 'FABLIZE' ~/workspace/projects/aerion-airsim/CLAUDE.md   # ≥1 이면 블록 주입됨

# (2) Claude Code 내부 검증
/memoir:status          # memoir 스토어 검증
/memoir:recall          # 기억 조회

# (3) 프로젝트 규칙 로드
#   프로젝트를 열면 CLAUDE.md가 자동 로드되고, fablize 블록이 오퍼레이팅 모드로 적용됨
```

---

## 5. 문제 해결

**Q: memoir 스토어가 `~/.remember/`에 있지 않나?**
A: 아니다. 이 셋업의 memoir(v0.3.0)는 `~/.memoir/<프로젝트경로>/`에 **git 기반**으로 저장한다.
`~/.remember/`(remember.md/today/recent/...)는 본 셋업과 무관하며 이 머신에 존재하지 않는다.

**Q: `/fablize:setup`이 기억까지 켜주나?**
A: 아니다. fablize는 작업 규율 라우터다. 기억은 **memoir(자동 캡처) + 파일 메모리 프로토콜**이 담당한다. 둘은 별개 단계로 설치/초기화한다.

**Q: CLAUDE.md를 새로 쓰고 싶으면?**
A: 프로젝트 규칙은 `aerion-airsim/CLAUDE.md`(git 제공)를 편집. fablize 블록은 `/fablize:setup`이 멱등 관리하므로 손대지 말 것.

**Q: 오프라인 환경에서는?**
A: 파일 메모리·memoir 스토어·CLAUDE.md는 USB/rsync로 복사. git은 bundle 파일로 이전. 단, fablize/memoir **플러그인 설치 자체는 온라인 필요**.

---

## 6. 자동 기억 동작 (참고)

이 셋업에서 세션 간 지속 기억은 **두 경로**로 쌓인다 — fablize가 아니라 아래 두 시스템이 담당한다:

1. **memoir** — 세션 종료(Stop) 훅이 구조화 사실을 `~/.memoir/<proj>` 스토어에 자동 캡처.
2. **파일 메모리 프로토콜** — `memory/` 디렉토리에 `user`/`feedback`/`project`/`reference` 4유형으로 기록하고
   `MEMORY.md`에 한 줄 인덱스를 남김. 사용자 선호/금지(민감 항목)는 **먼저 통지 후 승인**, 나머지는 자동.

fablize 오퍼레이팅 블록은 위 기록을 **트리거하지 않는다.** 결과 우선·범위 준수·완료 근거 제시·다단계 작업의 goals.py 검증 게이트 등 *실행 규율*만 담당한다.

---

## 7. 자율 기록 층 (페이블식 실시간 캡처) — v3 추가

> **2026-06-17 추가.** 기본 셋업(1~6절)은 *세션 종료 시* memoir 자동 캡처 + *수동* 파일메모리다.
> 페이블처럼 **작업 도중 중요한 룰/맥락/결정을 스스로 판단해 실시간 기록**하는 경험까지 원하면 이 층을 추가한다.
> 이 층은 **모델 기능이 아니라 "상시 로드 규칙 + 강화 훅"**으로 구현된다.

**3층 구조:**

| 층 | 메커니즘 | 자동성 |
|---|---|---|
| L1 실시간 캡처 | `CLAUDE.md`의 `<!-- AUTOCAPTURE -->` 규칙 (매 세션 로드) | 행동 규율 (판단 기반) |
| L2 강화 백스톱 | Stop 훅 `.claude/hooks/autocapture-reminder.sh` (`asyncRewake`, **세션당 1회**) | 훅 자동 |
| L3 구조화 저장 | memoir Stop 훅 (1~6절에서 이미 설치) | 훅 자동 |

**구성 파일 (모두 git 포함 — clone 시 따라옴):**
- `CLAUDE.md` 내 `<!-- AUTOCAPTURE:BEGIN ... END -->` 섹션 — 포착 신호 6종(결정·제약·교훈·교정·선호·마일스톤), 라우팅 표, 자율성 정책(비민감=즉시 무인 기록 / 민감=통지 후). **fablize 블록 바깥**이라 `setup.sh`가 건드리지 않음.
- `.claude/hooks/autocapture-reminder.sh` — 세션당 1회 점검 신호. `session_id` 기준 `/tmp` sentinel로 루프 방지. **`jq` 비의존**(sed 파싱) — 이 머신엔 jq 미설치였음.
- `.claude/settings.json` — Stop 훅 등록. 경로는 `$CLAUDE_PROJECT_DIR`(이식성).

**타 머신 활성화:**
1. `git clone` → 위 3개 파일 자동 포함.
2. **`/hooks`를 한 번 열거나 Claude Code 재시작** — `.claude/`가 세션 시작 시 없었으면 설정 감시기가 Stop 훅을 늦게 읽는다. (L1·L3는 즉시 유효, L2 훅만 이 단계 필요)
3. 검증: `echo '{"session_id":"x"}' | bash .claude/hooks/autocapture-reminder.sh` → 첫 호출 exit 2 + 점검문, 재호출 exit 0.

**끄기:** `/hooks`에서 비활성화, 또는 `CLAUDE.md`의 `<!-- AUTOCAPTURE -->` 섹션 삭제.

**한계:** L1은 모델 판단 기반이라 100% 무인은 아니다(페이블도 본질 동일). L2 훅이 누락을 잡는 안전망.

---

**문서 버전:** 2026-06-17-v3 (자율 기록 층 추가)
**검증 기준 머신:** Ubuntu 22.04, 사용자 `clrobur`, fablize 2.0.0 / memoir 0.3.0
**v2 대비 추가:** 7절 자율 기록 층(L1 CLAUDE.md AUTOCAPTURE 규칙 + L2 Stop 강화 훅 + L3 memoir), jq 비의존 훅, `$CLAUDE_PROJECT_DIR` 이식 경로
**v1 대비 정정:** 경로(macOS→Linux), 저장소(AERION→Aerion-Airsim/SSH), memoir 스토어(`~/.remember`→`~/.memoir`), fablize(v5→v2, 동작 범위), 자동기억 주체(fablize→memoir+파일메모리)
