from __future__ import annotations

import asyncio
from typing import Any

from app.core.config import settings
from app.core.safety_rules import PlanningGraphValidator
from app.core.time import iso_now
from app.domain.models import (
    DispatchMode,
    DispatchResult,
    MissionConstraint,
    MissionDeployResponse,
    MissionPlanResponse,
    PlanningGraph,
    SafetyReport,
    SafetySeverity,
)
from app.services.a2a_client import A2AClient
from app.services.drone_state_cache import drone_state_cache
from app.services.event_bus import mission_event_bus
from app.services.intent_parser import parse_user_intent
from app.services.map_mcp import MapBoxMCPClient
from app.services.mission_store import mission_store
from app.services.planning_graph_builder import PlanningGraphBuilder


class MissionService:
    def __init__(self):
        self.map_client = MapBoxMCPClient()
        self.builder = PlanningGraphBuilder(self.map_client)
        self.a2a_client = A2AClient()

    def plan(self, text: str, target_drones: list[str], constraints: MissionConstraint | None, require_connected_drones: bool) -> MissionPlanResponse:
        structured_intent = parse_user_intent(text, constraints)
        graph, geo_context = self.builder.build(structured_intent, target_drones)
        validator = PlanningGraphValidator(drone_state_cache.all(), geo_context)
        safety_report = validator.validate(graph, require_connected_drones=require_connected_drones)
        status = "success" if safety_report.passed else "failed"
        if safety_report.passed:
            mission_store.create(graph, safety_report, status="planned")
        return MissionPlanResponse(
            status=status,
            structured_intent=structured_intent,
            planning_graph=graph if safety_report.passed else graph,
            safety_report=safety_report,
        )

    async def deploy(self, text: str, target_drones: list[str], constraints: MissionConstraint | None, dispatch_mode: DispatchMode) -> MissionDeployResponse:
        plan_result = self.plan(text, target_drones, constraints, require_connected_drones=True)
        graph = plan_result.planning_graph
        safety_report = plan_result.safety_report
        if not graph or not safety_report.passed:
            return MissionDeployResponse(status="failed", safety_report=safety_report, planning_graph=graph)

        rec = mission_store.get(graph.mission_id) or mission_store.create(graph, safety_report, status="planned")
        dispatch_results: list[DispatchResult] = []
        if dispatch_mode == DispatchMode.none:
            dispatch_results = [DispatchResult(drone_id=d, status="skipped", detail={"reason": "dispatch_mode=none"}) for d in graph.assigned_drones]
            return MissionDeployResponse(
                status="accepted",
                mission_id=graph.mission_id,
                graph_id=graph.graph_id,
                safety_report=safety_report,
                dispatch_results=dispatch_results,
                planning_graph=graph,
            )

        for drone_id in graph.assigned_drones:
            drone = drone_state_cache.get(drone_id)
            endpoint = (drone.a2a_url if drone and drone.a2a_url else settings.soma_default_a2a_url).rstrip("/")
            dispatch_results.append(DispatchResult(drone_id=drone_id, status="queued", endpoint=endpoint))

        await self._publish_and_store(graph.mission_id, {"event": "dispatch", "data": {"graph_id": graph.graph_id, "drones": graph.assigned_drones}, "timestamp": iso_now()})
        for result in dispatch_results:
            asyncio.create_task(self._dispatch_to_soma(graph, result.drone_id, result.endpoint or settings.soma_default_a2a_url, dispatch_mode))

        rec.status = "dispatching"
        return MissionDeployResponse(
            status="accepted",
            mission_id=graph.mission_id,
            graph_id=graph.graph_id,
            safety_report=safety_report,
            dispatch_results=dispatch_results,
            planning_graph=graph,
        )

    async def _dispatch_to_soma(self, graph: PlanningGraph, drone_id: str, endpoint: str, dispatch_mode: DispatchMode) -> None:
        try:
            if dispatch_mode == DispatchMode.send:
                response = await self.a2a_client.send(endpoint, graph, drone_id)
                await self._publish_and_store(
                    graph.mission_id,
                    {"event": "sent", "drone_id": drone_id, "data": response, "timestamp": iso_now()},
                )
                return

            async for event in self.a2a_client.send_stream(endpoint, graph, drone_id):
                await self._publish_and_store(
                    graph.mission_id,
                    {"event": event.get("event", "message"), "drone_id": drone_id, "data": event.get("data", {}), "timestamp": iso_now()},
                )
        except Exception as exc:
            await self._publish_and_store(
                graph.mission_id,
                {"event": "failed", "drone_id": drone_id, "data": {"error": str(exc), "endpoint": endpoint}, "timestamp": iso_now()},
            )

    async def _publish_and_store(self, mission_id: str, event: dict[str, Any]) -> None:
        mission_store.append_event(mission_id, event)
        await mission_event_bus.publish(mission_id, event)

    def failed_safety_report(self, message: str) -> SafetyReport:
        from app.domain.models import SafetyIssue

        return SafetyReport(
            passed=False,
            issues=[SafetyIssue(rule_id="MIND", severity=SafetySeverity.error, message=message)],
        )


mission_service = MissionService()
