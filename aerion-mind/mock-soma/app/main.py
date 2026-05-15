from __future__ import annotations

import asyncio
import json
import os
from datetime import datetime, timezone
from typing import Any

import httpx
from fastapi import FastAPI
from fastapi.responses import StreamingResponse

app = FastAPI(title="AERION Mock Soma", version="1.0.0")

MIND_BASE_URL = os.getenv("MIND_BASE_URL", "http://localhost:8020").rstrip("/")
SOMA_PUBLIC_A2A_URL = os.getenv("SOMA_PUBLIC_A2A_URL", "http://localhost:8011").rstrip("/")
DRONE_ID = os.getenv("DRONE_ID", "drone1")


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sse(event: str, data: dict[str, Any]) -> str:
    return f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False)}\n\n"


def extract_graph(payload: dict[str, Any]) -> dict[str, Any]:
    message = payload.get("params", {}).get("message", {})
    for part in message.get("parts", []):
        data = part.get("data") or {}
        if data.get("graph"):
            return data["graph"]
    return {}


async def heartbeat_loop():
    await asyncio.sleep(2)
    while True:
        payload = {
            "system_id": DRONE_ID,
            "mode": "GUIDED",
            "armed": True,
            "battery_percent": 82,
            "position": {"lat": 37.5326, "lon": 127.0246, "alt_m": 35},
            "heading_deg": 45,
            "a2a_url": SOMA_PUBLIC_A2A_URL,
            "role": "leader",
            "status": "ready",
        }
        try:
            async with httpx.AsyncClient(timeout=3.0) as client:
                await client.post(f"{MIND_BASE_URL}/api/v1/drones/{DRONE_ID}/heartbeat", json=payload)
        except Exception:
            pass
        await asyncio.sleep(1)


@app.on_event("startup")
async def startup():
    asyncio.create_task(heartbeat_loop())


@app.get("/health")
async def health():
    return {"status": "ok", "service": "mock-soma", "drone_id": DRONE_ID, "timestamp": iso_now()}


@app.post("/a2a/message/send")
async def a2a_send(payload: dict[str, Any]):
    graph = extract_graph(payload)
    return {
        "jsonrpc": "2.0",
        "id": payload.get("id"),
        "result": {
            "status": "received",
            "drone_id": DRONE_ID,
            "mission_id": graph.get("mission_id"),
            "graph_id": graph.get("graph_id"),
            "timestamp": iso_now(),
        },
    }


@app.post("/a2a/message/send-stream")
async def a2a_send_stream(payload: dict[str, Any]):
    graph = extract_graph(payload)
    mission_id = graph.get("mission_id")
    graph_id = graph.get("graph_id")
    nodes = graph.get("nodes", [])[:8]

    async def gen():
        yield sse("ack", {"status": "received", "drone_id": DRONE_ID, "mission_id": mission_id, "graph_id": graph_id, "timestamp": iso_now()})
        for idx, node in enumerate(nodes, start=1):
            await asyncio.sleep(0.35)
            yield sse("progress", {"node": node.get("id"), "action": node.get("action"), "idx": idx, "total": len(nodes), "status": "ok"})
        await asyncio.sleep(0.2)
        yield sse("done", {"status": "completed", "drone_id": DRONE_ID, "mission_id": mission_id, "graph_id": graph_id, "timestamp": iso_now()})

    return StreamingResponse(gen(), media_type="text/event-stream")
