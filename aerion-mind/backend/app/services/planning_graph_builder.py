from __future__ import annotations

from uuid import uuid4

from app.domain.models import CommandType, MissionConstraint, PlanningEdge, PlanningGraph, PlanningNode, StructuredIntent
from app.services.map_mcp import MapBoxMCPClient


class PlanningGraphBuilder:
    def __init__(self, map_client: MapBoxMCPClient | None = None):
        self.map_client = map_client or MapBoxMCPClient()

    def build(self, intent: StructuredIntent, target_drones: list[str]) -> tuple[PlanningGraph, dict]:
        location = self.map_client.resolve_location(intent.raw_text, intent.target_label)
        center = location["center"]
        radius_m = float(location["radius_m"])
        altitude_m = min(intent.altitude_max_m, intent.constraints.max_altitude_m)
        waypoints = self.map_client.build_lawnmower_waypoints(center, radius_m, altitude_m, lanes=3)

        assigned = target_drones or ["drone1"]
        mission_id = f"mission_{uuid4().hex[:12]}"
        graph_id = f"pg_{uuid4().hex[:12]}"
        drone_id = assigned[0]

        nodes: list[PlanningNode] = [
            PlanningNode(
                id="START",
                type="start",
                action="start_mission",
                command_type=CommandType.mission_control,
                assigned_to=drone_id,
                params={"mission_id": mission_id},
            ),
            PlanningNode(
                id="MODE_GUIDED",
                type="control",
                action="set_mode",
                command_type=CommandType.mode_change,
                assigned_to=drone_id,
                params={"mode": "GUIDED"},
            ),
            PlanningNode(
                id="TAKEOFF",
                type="action",
                action="takeoff",
                command_type=CommandType.direct_command,
                assigned_to=drone_id,
                params={"target_alt_m": altitude_m},
            ),
        ]

        previous_id = "TAKEOFF"
        edges: list[PlanningEdge] = [
            PlanningEdge(**{"from": "START", "to": "MODE_GUIDED", "condition": "accepted"}),
            PlanningEdge(**{"from": "MODE_GUIDED", "to": "TAKEOFF", "condition": "mode_ok"}),
        ]

        for idx, wp in enumerate(waypoints, start=1):
            node_id = f"WP{idx:02d}"
            nodes.append(
                PlanningNode(
                    id=node_id,
                    type="waypoint",
                    action="navigate_to",
                    command_type=CommandType.mission_upload,
                    assigned_to=drone_id,
                    params={
                        "lat": wp.lat,
                        "lon": wp.lon,
                        "alt_m": wp.alt_m,
                        "speed_mps": min(intent.constraints.max_speed_mps, 8.0),
                        "acceptance_radius_m": 5,
                    },
                )
            )
            edges.append(PlanningEdge(**{"from": previous_id, "to": node_id, "condition": "success"}))
            previous_id = node_id

        if intent.photo_required:
            nodes.append(
                PlanningNode(
                    id="CAPTURE_PHOTO",
                    type="action",
                    action="capture_photo",
                    command_type=CommandType.direct_command,
                    assigned_to=drone_id,
                    params={"target": intent.target_label, "min_quality": "standard"},
                )
            )
            edges.append(PlanningEdge(**{"from": previous_id, "to": "CAPTURE_PHOTO", "condition": "arrived"}))
            previous_id = "CAPTURE_PHOTO"

        nodes.extend(
            [
                PlanningNode(
                    id="RETURN_HOME",
                    type="control",
                    action="return_to_home",
                    command_type=CommandType.mission_control,
                    assigned_to=drone_id,
                    params={"reason": "mission_complete"},
                ),
                PlanningNode(
                    id="GOAL",
                    type="terminal",
                    action="complete_mission",
                    command_type=CommandType.mission_control,
                    assigned_to=drone_id,
                    params={"mission_id": mission_id},
                ),
            ]
        )
        edges.append(PlanningEdge(**{"from": previous_id, "to": "RETURN_HOME", "condition": "success"}))
        edges.append(PlanningEdge(**{"from": "RETURN_HOME", "to": "GOAL", "condition": "landed_or_rth_confirmed"}))

        constraints: MissionConstraint = intent.constraints
        graph = PlanningGraph(
            graph_id=graph_id,
            mission_id=mission_id,
            intent=intent,
            assigned_drones=assigned,
            constraints=constraints,
            nodes=nodes,
            edges=edges,
            metadata={
                "planner": "aerion-mind-deterministic-builder",
                "map_context_label": location["label"],
                "map_context_radius_m": radius_m,
                "geo_context": {
                    "center": center.model_dump(),
                    "restricted_zones": [
                        {"id": g["id"], "name": g["name"], "bbox": g["bbox"].__dict__}
                        for g in self.map_client.get_geofences_in_area(center, radius_m)
                    ],
                },
            },
        )
        geo_context = graph.metadata["geo_context"]
        return graph, geo_context
