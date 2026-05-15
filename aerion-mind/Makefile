up:
	docker compose up --build

down:
	docker compose down --remove-orphans

logs:
	docker compose logs -f

backend-test:
	cd backend && pytest -q

zip:
	cd .. && zip -r AERION_MIND_Stack.zip aerion-mind-stack -x '*/.venv/*' '*/node_modules/*' '*/__pycache__/*'
