from __future__ import annotations

from app.agent.state import MindState
from app.agent.nodes.n1_context import node_gather_context
from app.agent.nodes.n2_planner import node_generate_pg
from app.agent.nodes.n3_validator import node_safety_validator


class MindPipeline:
    """LangGraph-compatible lightweight pipeline.

    Gemini boilerplate used `mind_app.ainvoke(initial_state)`. This class keeps the same
    call shape without requiring LangGraph during Docker bootstrap. Replace this class
    with a real LangGraph `StateGraph` when the LLM planner is attached.
    """

    async def ainvoke(self, initial_state: MindState) -> MindState:
        state = node_gather_context(initial_state)
        max_revisions = 3
        while True:
            state = node_generate_pg(state)
            state = node_safety_validator(state)
            if not state.get("safety_errors"):
                return state
            if state.get("revision_count", 0) >= max_revisions:
                return state


mind_app = MindPipeline()
