from __future__ import annotations

import math
from dataclasses import dataclass

from app.domain.models import Coordinate

EARTH_RADIUS_M = 6371000.0


def haversine_m(a: Coordinate, b: Coordinate) -> float:
    lat1 = math.radians(a.lat)
    lat2 = math.radians(b.lat)
    dlat = math.radians(b.lat - a.lat)
    dlon = math.radians(b.lon - a.lon)
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(h))


def offset_coordinate(origin: Coordinate, north_m: float, east_m: float, alt_m: float | None = None) -> Coordinate:
    dlat = north_m / 111111.0
    lon_scale = 111111.0 * math.cos(math.radians(origin.lat))
    if abs(lon_scale) < 1e-6:
        lon_scale = 1e-6
    dlon = east_m / lon_scale
    return Coordinate(lat=origin.lat + dlat, lon=origin.lon + dlon, alt_m=alt_m if alt_m is not None else origin.alt_m)


@dataclass(frozen=True)
class BoundingBox:
    min_lat: float
    max_lat: float
    min_lon: float
    max_lon: float

    def contains(self, point: Coordinate, margin_deg: float = 0.0) -> bool:
        return (
            self.min_lat - margin_deg <= point.lat <= self.max_lat + margin_deg
            and self.min_lon - margin_deg <= point.lon <= self.max_lon + margin_deg
        )
