FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY app ./app

EXPOSE 8020
CMD ["sh", "-c", "uvicorn app.main:app --host ${MIND_HOST:-0.0.0.0} --port ${MIND_PORT:-8020}"]
