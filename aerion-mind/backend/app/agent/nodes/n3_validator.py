from __future__ import annotations

from app.agent.state import MindState
from app.core.safety_rules import PlanningGraphValidator
from app.services.drone_state_cache import drone_state_cache


def node_safety_validator(state: MindState) -> MindState:
    graph = state.get("draft_pg")
    if graph is None:
        return {**state, "safety_errors": ["draft_pg is missing"], "revision_count": state.get("revision_count", 0) + 1}
    validator = PlanningGraphValidator(drone_state_cache.all(), state.get("geo_context", {}))
    report = validator.validate(graph, require_connected_drones=False)
    errors = [issue.message for issue in report.blocking_issues]
    output = {**state, "safety_report": report, "safety_errors": errors}
    if report.passed:
        output["final_pg"] = graph
    else:
        output["revision_count"] = state.get("revision_count", 0) + 1
    return output
