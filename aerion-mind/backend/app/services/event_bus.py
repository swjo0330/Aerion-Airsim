from __future__ import annotations

import asyncio
from collections import defaultdict
from typing import Any


class MissionEventBus:
    def __init__(self):
        self._queues: dict[str, list[asyncio.Queue[dict[str, Any]]]] = defaultdict(list)

    async def publish(self, mission_id: str, event: dict[str, Any]) -> None:
        queues = list(self._queues.get(mission_id, []))
        for queue in queues:
            await queue.put(event)

    async def subscribe(self, mission_id: str):
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self._queues[mission_id].append(queue)
        try:
            while True:
                event = await queue.get()
                yield event
        finally:
            self._queues[mission_id].remove(queue)


mission_event_bus = MissionEventBus()
