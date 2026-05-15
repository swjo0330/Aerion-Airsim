from __future__ import annotations

import json
from collections.abc import AsyncIterator
from typing import Any

import httpx

from app.core.config import settings
from app.domain.models import A2AMessage, A2APart, PlanningGraph


class A2AClient:
    def __init__(self, timeout_s: float | None = None):
        self.timeout_s = timeout_s or settings.dispatch_timeout_seconds

    def _build_message(self, graph: PlanningGraph, drone_id: str) -> A2AMessage:
        return A2AMessage(
            role="mind",
            type="mission_upload",
            parts=[
                A2APart(
                    kind="data",
                    data={
                        "type": "mission_upload",
                        "drone_id": drone_id,
                        "mission_id": graph.mission_id,
                        "graph_id": graph.graph_id,
                        "graph": graph.model_dump(mode="json"),
                        "constraints": graph.constraints.model_dump(mode="json"),
                    },
                )
            ],
            metadata={"protocol": "AERION-A2A", "source": "aerion-mind", "target_drone": drone_id},
        )

    def _jsonrpc(self, method: str, message: A2AMessage) -> dict[str, Any]:
        return {
            "jsonrpc": "2.0",
            "id": message.message_id,
            "method": method,
            "params": {"message": message.model_dump(mode="json")},
        }

    async def send(self, base_url: str, graph: PlanningGraph, drone_id: str) -> dict[str, Any]:
        message = self._build_message(graph, drone_id)
        payload = self._jsonrpc("message/send", message)
        url = base_url.rstrip("/") + "/a2a/message/send"
        async with httpx.AsyncClient(timeout=self.timeout_s) as client:
            response = await client.post(url, json=payload)
            response.raise_for_status()
            return response.json()

    async def send_stream(self, base_url: str, graph: PlanningGraph, drone_id: str) -> AsyncIterator[dict[str, Any]]:
        message = self._build_message(graph, drone_id)
        payload = self._jsonrpc("message/send-stream", message)
        url = base_url.rstrip("/") + "/a2a/message/send-stream"
        async with httpx.AsyncClient(timeout=httpx.Timeout(self.timeout_s, read=None)) as client:
            async with client.stream("POST", url, json=payload) as response:
                response.raise_for_status()
                event_name = "message"
                async for raw_line in response.aiter_lines():
                    line = raw_line.strip()
                    if not line:
                        continue
                    if line.startswith("event:"):
                        event_name = line.split(":", 1)[1].strip()
                    elif line.startswith("data:"):
                        data_text = line.split(":", 1)[1].strip()
                        try:
                            data = json.loads(data_text)
                        except json.JSONDecodeError:
                            data = {"raw": data_text}
                        yield {"event": event_name, "data": data}
                        event_name = "message"
