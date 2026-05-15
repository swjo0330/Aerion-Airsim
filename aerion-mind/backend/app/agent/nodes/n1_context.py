from __future__ import annotations

from app.agent.state import MindState
from app.domain.models import MissionConstraint
from app.services.intent_parser import parse_user_intent
from app.services.map_mcp import MapBoxMCPClient


def node_gather_context(state: MindState) -> MindState:
    constraints = None
    if state.get("constraints"):
        constraints = MissionConstraint(**state["constraints"])
    structured = parse_user_intent(state.get("user_intent", ""), constraints)
    map_client = MapBoxMCPClient()
    location = map_client.resolve_location(structured.raw_text, structured.target_label)
    geo_context = {
        "resolved_target": {
            "label": location["label"],
            "center": location["center"].model_dump(),
            "radius_m": location["radius_m"],
        },
        "restricted_zones": [
            {"id": item["id"], "name": item["name"], "bbox": item["bbox"].__dict__}
            for item in map_client.get_geofences_in_area(location["center"], location["radius_m"])
        ],
    }
    return {**state, "structured_intent": structured, "geo_context": geo_context}
