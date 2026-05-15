from __future__ import annotations

from app.agent.state import MindState
from app.services.planning_graph_builder import PlanningGraphBuilder


def node_generate_pg(state: MindState) -> MindState:
    structured = state["structured_intent"]
    target_drones = state.get("target_drones") or ["drone1"]
    graph, geo_context = PlanningGraphBuilder().build(structured, target_drones)
    return {**state, "draft_pg": graph, "geo_context": geo_context}
