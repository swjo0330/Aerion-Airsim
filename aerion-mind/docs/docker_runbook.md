# Docker 실행/운영 가이드

## 1. 실행

```bash
cp .env.example .env
docker compose up --build
```

## 2. 접속

- Frontend: http://localhost:3000
- Backend: http://localhost:8020/docs
- Mock Soma: http://localhost:8011/health

## 3. 로그

```bash
docker compose logs -f backend
docker compose logs -f mock-soma
docker compose logs -f frontend
```

## 4. 실제 Soma 연결

`.env` 또는 docker compose environment에서 다음 값을 변경한다.

```env
SOMA_DEFAULT_A2A_URL=http://<soma-supervisor-ip>:8011
```

여러 드론을 운용할 때는 각 드론 heartbeat가 자신의 `a2a_url`을 보내도록 한다.

## 5. 흔한 문제

### 프론트에서 API 연결 실패

브라우저 기준 주소가 필요하므로 `NEXT_PUBLIC_API_BASE_URL=http://localhost:8020/api/v1`인지 확인한다.

### deploy가 accepted인데 failed event 발생

Mind는 비동기 dispatch를 수행한다. accepted는 safety validation을 통과하고 dispatch task가 생성됐다는 의미다. Soma endpoint 불가 시 event stream에 failed가 뜬다.

### drone stale

`STALE_DRONE_SECONDS` 기본값은 30초다. 실제 Soma heartbeat가 1Hz로 들어오는지 확인한다.
