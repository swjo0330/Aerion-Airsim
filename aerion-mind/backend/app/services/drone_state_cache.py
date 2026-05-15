from __future__ import annotations

from datetime import datetime, timezone
from threading import RLock

from app.core.config import settings
from app.domain.models import DroneHeartbeat, DroneState


class DroneStateCache:
    def __init__(self):
        self._items: dict[str, DroneHeartbeat] = {}
        self._lock = RLock()

    def upsert(self, heartbeat: DroneHeartbeat) -> DroneState:
        now = datetime.now(timezone.utc)
        if heartbeat.last_heartbeat is None:
            heartbeat.last_heartbeat = now
        with self._lock:
            self._items[heartbeat.system_id] = heartbeat
        return self.get(heartbeat.system_id)  # type: ignore[return-value]

    def get(self, system_id: str) -> DroneState | None:
        with self._lock:
            hb = self._items.get(system_id)
        if hb is None:
            return None
        return self._to_state(hb)

    def all(self) -> list[DroneState]:
        with self._lock:
            items = list(self._items.values())
        return [self._to_state(hb) for hb in items]

    def alive(self) -> list[DroneState]:
        return [drone for drone in self.all() if not drone.stale and drone.status != "offline"]

    def remove(self, system_id: str) -> bool:
        with self._lock:
            existed = system_id in self._items
            self._items.pop(system_id, None)
        return existed

    def _to_state(self, hb: DroneHeartbeat) -> DroneState:
        now = datetime.now(timezone.utc)
        last = hb.last_heartbeat or now
        if last.tzinfo is None:
            last = last.replace(tzinfo=timezone.utc)
        age = max((now - last).total_seconds(), 0.0)
        stale = age > settings.stale_drone_seconds
        return DroneState(**hb.model_dump(), stale=stale, age_seconds=age)


drone_state_cache = DroneStateCache()
