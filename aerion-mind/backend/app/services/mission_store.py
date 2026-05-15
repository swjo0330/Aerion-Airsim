from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from threading import RLock
from typing import Any

from app.domain.models import PlanningGraph, SafetyReport


@dataclass
class MissionRecord:
    mission_id: str
    graph_id: str
    status: str
    graph: PlanningGraph
    safety_report: SafetyReport
    events: list[dict[str, Any]] = field(default_factory=list)
    created_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    updated_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))


class MissionStore:
    def __init__(self):
        self._items: dict[str, MissionRecord] = {}
        self._lock = RLock()

    def create(self, graph: PlanningGraph, safety_report: SafetyReport, status: str = "planned") -> MissionRecord:
        rec = MissionRecord(
            mission_id=graph.mission_id,
            graph_id=graph.graph_id,
            status=status,
            graph=graph,
            safety_report=safety_report,
        )
        with self._lock:
            self._items[graph.mission_id] = rec
        return rec

    def get(self, mission_id: str) -> MissionRecord | None:
        with self._lock:
            return self._items.get(mission_id)

    def append_event(self, mission_id: str, event: dict[str, Any]) -> None:
        with self._lock:
            rec = self._items.get(mission_id)
            if rec is None:
                return
            rec.events.append(event)
            rec.updated_at = datetime.now(timezone.utc)
            if event.get("event") in {"done", "completed"}:
                rec.status = "completed"
            elif event.get("event") in {"failed", "error"}:
                rec.status = "failed"
            elif event.get("event") in {"ack", "progress", "dispatch"}:
                rec.status = "running"

    def all(self) -> list[MissionRecord]:
        with self._lock:
            return list(self._items.values())


mission_store = MissionStore()
