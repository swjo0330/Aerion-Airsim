#!/usr/bin/env bash
# 자율 지식 축적 규칙 — Stop 강화 훅 (페이블식).
# 세션당 1회만, 비동기로 모델을 깨워 "기록거리 점검"을 시킨다.
#   - 세션당 1회: session_id 기준 sentinel 로 중복/루프 방지.
#   - 비차단: asyncRewake(async) 라서 Stop 을 동기 차단하지 않음.
#   - exit 2 = (asyncRewake 계약) 모델 재기상 + stdout 을 system-reminder 로 주입.
#   - exit 0 = 조용히 종료 (이미 점검함).
# memoir 의 Stop 훅(플러그인 hooks.json)과는 별개로 동작 — 중복/간섭 없음.

input=$(cat 2>/dev/null)
# jq 없이 session_id 추출 (이 호스트엔 jq 미설치). 실패 시 nosession 폴백.
sid=$(printf '%s' "$input" | sed -n 's/.*"session_id"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' | head -1)
[ -z "$sid" ] && sid="nosession"
sentinel="/tmp/claude-autocapture-${sid}.done"

# 이미 이 세션에서 점검 신호를 줬으면 조용히 종료 (루프 방지).
[ -f "$sentinel" ] && exit 0
touch "$sentinel" 2>/dev/null

cat <<'MSG'
[자율 지식 축적 규칙 점검] 이번 세션에서 아래 중 아직 기록하지 않은 것이 있는지 1회 점검하라:
- 결정(이유 포함) · 제약/관례 · 교훈/함정 · 사용자 교정/피드백 · 마일스톤/상태 변화
있으면 지금 알맞은 곳에 기록(비민감=즉시, 민감=통지 후): 파일메모리(memory/<slug>.md + MEMORY.md) / docs/ / CLAUDE.md / memoir.
없으면 "기록할 것 없음"만 남기고 종료하라. (이 점검은 세션당 1회만 발생)
MSG
exit 2
