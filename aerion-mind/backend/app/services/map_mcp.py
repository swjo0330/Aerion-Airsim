from __future__ import annotations

from typing import Any

from app.domain.models import Coordinate
from app.services.geo import BoundingBox, offset_coordinate


class MapBoxMCPClient:
    """Offline-first MapBox MCP adapter.

    운영 환경에서는 이 클래스를 실제 MCP 클라이언트로 교체하면 됩니다.
    현재 구현은 Docker 데모가 외부 API 없이 동작하도록 known tactical zones를 사용합니다.
    """

    def __init__(self, api_key: str | None = None):
        self.api_key = api_key
        self.tactical_zones: dict[str, dict[str, Any]] = {
            "A구역": {"lat": 37.5326, "lon": 127.0246, "radius_m": 500, "label": "A구역"},
            "B구역": {"lat": 37.5660, "lon": 126.9784, "radius_m": 450, "label": "B구역"},
            "mountain_region": {"lat": 37.5400, "lon": 127.0100, "radius_m": 700, "label": "mountain_region"},
        }
        self.known_geofences = [
            {
                "id": "NFZ-01",
                "name": "restricted-core",
                "bbox": BoundingBox(min_lat=37.5480, max_lat=37.5520, min_lon=127.0400, max_lon=127.0450),
            }
        ]

    def resolve_location(self, intent_text: str, target_hint: str | None = None) -> dict[str, Any]:
        for zone_name, zone in self.tactical_zones.items():
            if zone_name in intent_text or zone_name == target_hint:
                center = Coordinate(lat=zone["lat"], lon=zone["lon"], alt_m=None)
                return {"label": zone_name, "center": center, "radius_m": zone["radius_m"]}
        zone = self.tactical_zones["A구역"]
        return {"label": "A구역", "center": Coordinate(lat=zone["lat"], lon=zone["lon"]), "radius_m": zone["radius_m"]}

    def get_geofences_in_area(self, center: Coordinate, radius_m: float) -> list[dict[str, Any]]:
        return self.known_geofences

    def build_lawnmower_waypoints(self, center: Coordinate, radius_m: float, altitude_m: float, lanes: int = 3) -> list[Coordinate]:
        half = min(max(radius_m, 120.0), 800.0) / 2
        lane_gap = (2 * half) / max(lanes - 1, 1)
        points: list[Coordinate] = []
        for i in range(lanes):
            north = -half + i * lane_gap
            if i % 2 == 0:
                points.append(offset_coordinate(center, north, -half, altitude_m))
                points.append(offset_coordinate(center, north, half, altitude_m))
            else:
                points.append(offset_coordinate(center, north, half, altitude_m))
                points.append(offset_coordinate(center, north, -half, altitude_m))
        return points
