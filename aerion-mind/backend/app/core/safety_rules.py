from __future__ import annotations

from collections import defaultdict, deque
from typing import Any

from app.core.config import settings
from app.domain.models import Coordinate, DroneState, PlanningGraph, SafetyIssue, SafetyReport, SafetySeverity
from app.services.geo import BoundingBox, haversine_m


ALLOWED_ACTIONS = {
    "start_mission",
    "set_mode",
    "takeoff",
    "navigate_to",
    "capture_photo",
    "return_to_home",
    "complete_mission",
    "hold_hover",
    "abort_mission",
}

FORBIDDEN_ACTIONS = {
    "motor_pwm",
    "set_attitude_raw",
    "fire_payload",
    "weapon_release",
    "disable_failsafe",
}


class PlanningGraphValidator:
    """Rule-first safety validator for AERION MIND.

    The validator intentionally avoids LLM calls. Blocking issues are `error` or `critical`.
    Warnings are returned to the operator but do not block dispatch.
    """

    def __init__(self, drone_states: list[DroneState], geo_context: dict[str, Any] | None = None):
        self.drone_states = {d.system_id: d for d in drone_states}
        self.geo_context = geo_context or {}
        self.issues: list[SafetyIssue] = []

    def validate(self, graph: PlanningGraph, require_connected_drones: bool = True) -> SafetyReport:
        self.issues = []
        self._check_graph_non_empty(graph)
        self._check_unique_node_ids(graph)
        self._check_edges_reference_nodes(graph)
        self._check_connectivity(graph)
        self._check_required_terminal(graph)
        self._check_allowed_actions(graph)
        self._check_action_params(graph)
        self._check_physical_constraints(graph)
        self._check_geofence(graph)
        self._check_drones(graph, require_connected_drones=require_connected_drones)
        self._check_mission_duration(graph)
        blocking = [i for i in self.issues if i.severity in {SafetySeverity.error, SafetySeverity.critical}]
        return SafetyReport(passed=len(blocking) == 0, issues=self.issues)

    def _issue(self, rule_id: str, severity: SafetySeverity, message: str, node_id: str | None = None, **detail: Any) -> None:
        self.issues.append(SafetyIssue(rule_id=rule_id, severity=severity, message=message, node_id=node_id, detail=detail))

    def _check_graph_non_empty(self, graph: PlanningGraph) -> None:
        if not graph.nodes:
            self._issue("R001", SafetySeverity.critical, "Planning graph must contain at least one node.")
        if not graph.edges:
            self._issue("R002", SafetySeverity.error, "Planning graph must contain sequence edges.")

    def _check_unique_node_ids(self, graph: PlanningGraph) -> None:
        seen: set[str] = set()
        for node in graph.nodes:
            if node.id in seen:
                self._issue("R003", SafetySeverity.critical, f"Duplicate node id: {node.id}", node.id)
            seen.add(node.id)

    def _check_edges_reference_nodes(self, graph: PlanningGraph) -> None:
        ids = {node.id for node in graph.nodes}
        for edge in graph.edges:
            if edge.source not in ids or edge.target not in ids:
                self._issue("R004", SafetySeverity.critical, "Edge references missing node.", None, source=edge.source, target=edge.target)

    def _check_connectivity(self, graph: PlanningGraph) -> None:
        ids = {node.id for node in graph.nodes}
        if "START" not in ids:
            self._issue("R005", SafetySeverity.error, "START node is required.")
            return
        adjacency: dict[str, list[str]] = defaultdict(list)
        for edge in graph.edges:
            adjacency[edge.source].append(edge.target)
        reachable = set(["START"])
        queue: deque[str] = deque(["START"])
        while queue:
            cur = queue.popleft()
            for nxt in adjacency.get(cur, []):
                if nxt not in reachable:
                    reachable.add(nxt)
                    queue.append(nxt)
        unreachable = ids - reachable
        for node_id in sorted(unreachable):
            self._issue("R006", SafetySeverity.error, f"Node is unreachable from START: {node_id}", node_id)

    def _check_required_terminal(self, graph: PlanningGraph) -> None:
        actions = {node.action for node in graph.nodes}
        if graph.constraints.require_rth and "return_to_home" not in actions:
            self._issue("R007", SafetySeverity.error, "return_to_home terminal/control node is required.")
        if "complete_mission" not in actions:
            self._issue("R008", SafetySeverity.warning, "complete_mission GOAL node is recommended.")

    def _check_allowed_actions(self, graph: PlanningGraph) -> None:
        for node in graph.nodes:
            if node.action in FORBIDDEN_ACTIONS:
                self._issue("R009", SafetySeverity.critical, f"Forbidden action is not allowed: {node.action}", node.id)
            elif node.action not in ALLOWED_ACTIONS:
                self._issue("R010", SafetySeverity.error, f"Unknown action: {node.action}", node.id)

    def _check_action_params(self, graph: PlanningGraph) -> None:
        for node in graph.nodes:
            params = node.params
            if node.action == "set_mode" and not params.get("mode"):
                self._issue("R011", SafetySeverity.error, "set_mode requires mode parameter.", node.id)
            if node.action == "takeoff" and params.get("target_alt_m") is None:
                self._issue("R012", SafetySeverity.error, "takeoff requires target_alt_m.", node.id)
            if node.action == "navigate_to":
                missing = [k for k in ["lat", "lon", "alt_m"] if params.get(k) is None]
                if missing:
                    self._issue("R013", SafetySeverity.error, f"navigate_to missing parameters: {missing}", node.id)

    def _check_physical_constraints(self, graph: PlanningGraph) -> None:
        max_alt = min(graph.constraints.max_altitude_m, settings.max_altitude_m)
        min_alt = max(graph.constraints.min_altitude_m, settings.min_altitude_m)
        max_speed = min(graph.constraints.max_speed_mps, settings.max_speed_mps)
        previous_wp: Coordinate | None = None
        for node in graph.nodes:
            params = node.params
            alt = params.get("alt_m") if params.get("alt_m") is not None else params.get("target_alt_m")
            if alt is not None:
                if float(alt) > max_alt:
                    self._issue("R014", SafetySeverity.critical, f"Altitude exceeds allowed maximum: {alt}m > {max_alt}m", node.id)
                if node.action in {"takeoff", "navigate_to"} and float(alt) < min_alt:
                    self._issue("R015", SafetySeverity.error, f"Altitude below minimum safe altitude: {alt}m < {min_alt}m", node.id)
            speed = params.get("speed_mps")
            if speed is not None and float(speed) > max_speed:
                self._issue("R016", SafetySeverity.error, f"Speed exceeds allowed maximum: {speed}m/s > {max_speed}m/s", node.id)
            if node.action == "navigate_to":
                try:
                    current = Coordinate(lat=float(params["lat"]), lon=float(params["lon"]), alt_m=float(params["alt_m"]))
                except Exception as exc:
                    self._issue("R017", SafetySeverity.error, f"Invalid coordinate: {exc}", node.id)
                    continue
                if previous_wp is not None:
                    distance = haversine_m(previous_wp, current)
                    if distance > settings.max_leg_distance_m:
                        self._issue("R018", SafetySeverity.warning, f"Long waypoint leg: {distance:.1f}m", node.id, distance_m=distance)
                previous_wp = current

    def _check_geofence(self, graph: PlanningGraph) -> None:
        restricted = self.geo_context.get("restricted_zones", [])
        geofences: list[tuple[str, BoundingBox]] = []
        for item in restricted:
            bbox = item.get("bbox") if isinstance(item, dict) else None
            if isinstance(bbox, dict):
                geofences.append((item.get("id", "unknown"), BoundingBox(**bbox)))
        if not geofences:
            return
        margin_deg = graph.constraints.geofence_margin_m / 111111.0
        for node in graph.nodes:
            if node.action != "navigate_to":
                continue
            params = node.params
            point = Coordinate(lat=float(params["lat"]), lon=float(params["lon"]), alt_m=float(params["alt_m"]))
            for geofence_id, bbox in geofences:
                if bbox.contains(point, margin_deg=margin_deg):
                    self._issue("R019", SafetySeverity.critical, f"Waypoint intersects restricted geofence {geofence_id}.", node.id, geofence_id=geofence_id)

    def _check_drones(self, graph: PlanningGraph, require_connected_drones: bool) -> None:
        if not graph.assigned_drones:
            self._issue("R020", SafetySeverity.error, "At least one assigned drone is required.")
            return
        if not require_connected_drones:
            return
        for drone_id in graph.assigned_drones:
            drone = self.drone_states.get(drone_id)
            if drone is None:
                self._issue("R021", SafetySeverity.error, f"Assigned drone is not connected: {drone_id}")
                continue
            if drone.stale:
                self._issue("R022", SafetySeverity.error, f"Assigned drone heartbeat is stale: {drone_id}", detail={"age_seconds": drone.age_seconds})
            if drone.battery_percent < settings.min_battery_percent:
                self._issue("R023", SafetySeverity.error, f"Battery below minimum threshold: {drone_id}", detail={"battery_percent": drone.battery_percent})

    def _check_mission_duration(self, graph: PlanningGraph) -> None:
        waypoint_count = sum(1 for node in graph.nodes if node.action == "navigate_to")
        rough_minutes = max(1.0, waypoint_count * 0.5)
        if rough_minutes > graph.constraints.max_mission_minutes:
            self._issue("R024", SafetySeverity.warning, "Mission may exceed max_mission_minutes.", detail={"estimated_minutes": rough_minutes})
