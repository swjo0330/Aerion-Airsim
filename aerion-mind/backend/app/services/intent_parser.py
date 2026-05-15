from __future__ import annotations

import re

from app.domain.models import MissionConstraint, MissionTask, StructuredIntent


_ALTITUDE_PATTERNS = [
    re.compile(r"고도\s*(\d+(?:\.\d+)?)\s*m\s*(?:이하|미만|까지)?"),
    re.compile(r"altitude\s*(?:<=|under|below)?\s*(\d+(?:\.\d+)?)\s*m", re.IGNORECASE),
]


def parse_user_intent(text: str, constraints: MissionConstraint | None = None) -> StructuredIntent:
    normalized = text.strip()
    task = MissionTask.reconnaissance
    photo_required = False
    area = "mountain_region" if "산" in normalized or "mountain" in normalized.lower() else "generic_area"
    target_label = "A구역"

    if "B구역" in normalized:
        target_label = "B구역"
    elif "A구역" in normalized:
        target_label = "A구역"

    if "촬영" in normalized or "사진" in normalized or "photo" in normalized.lower():
        photo_required = True
        task = MissionTask.capture_photo
    elif "순찰" in normalized or "patrol" in normalized.lower():
        task = MissionTask.patrol
    elif "점검" in normalized or "inspect" in normalized.lower():
        task = MissionTask.inspect

    altitude = constraints.max_altitude_m if constraints else 120.0
    for pattern in _ALTITUDE_PATTERNS:
        match = pattern.search(normalized)
        if match:
            altitude = float(match.group(1))
            break

    resolved_constraints = constraints or MissionConstraint(max_altitude_m=altitude)
    resolved_constraints.max_altitude_m = altitude

    return StructuredIntent(
        task=task,
        raw_text=normalized,
        target_label=target_label,
        area=area,
        target="waypoint_or_aoi",
        altitude_max_m=altitude,
        photo_required=photo_required,
        constraints=resolved_constraints,
    )
