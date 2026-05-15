from __future__ import annotations

from typing import Any, TypedDict


class MindState(TypedDict, total=False):
    user_intent: str
    target_drones: list[str]
    constraints: dict[str, Any]
    structured_intent: Any
    geo_context: dict[str, Any]
    draft_pg: Any
    safety_report: Any
    safety_errors: list[str]
    final_pg: Any
    revision_count: int
