from __future__ import annotations

from datetime import datetime, timezone
from enum import Enum
from typing import Any, Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class MissionTask(str, Enum):
    reconnaissance = "reconnaissance"
    capture_photo = "capture_photo"
    patrol = "patrol"
    inspect = "inspect"
    fallback = "fallback"


class CommandType(str, Enum):
    mission_upload = "mission_upload"
    mode_change = "mode_change"
    direct_command = "direct_command"
    mission_control = "mission_control"


class DispatchMode(str, Enum):
    none = "none"
    send = "send"
    stream = "stream"


class Coordinate(BaseModel):
    lat: float
    lon: float
    alt_m: float | None = None

    @field_validator("lat")
    @classmethod
    def _lat_range(cls, value: float) -> float:
        if value < -90 or value > 90:
            raise ValueError("lat must be in [-90, 90]")
        return value

    @field_validator("lon")
    @classmethod
    def _lon_range(cls, value: float) -> float:
        if value < -180 or value > 180:
            raise ValueError("lon must be in [-180, 180]")
        return value


class MissionConstraint(BaseModel):
    max_altitude_m: float = Field(default=120.0, gt=0)
    min_altitude_m: float = Field(default=10.0, ge=0)
    max_speed_mps: float = Field(default=8.0, gt=0)
    geofence_margin_m: float = Field(default=20.0, ge=0)
    require_rth: bool = True
    allow_fallback_land: bool = False
    max_mission_minutes: int = Field(default=20, gt=0)


class StructuredIntent(BaseModel):
    task: MissionTask = MissionTask.reconnaissance
    raw_text: str
    target_label: str = "A구역"
    area: str = "mountain_region"
    target: str = "waypoint_or_aoi"
    altitude_max_m: float = 120.0
    photo_required: bool = False
    constraints: MissionConstraint = Field(default_factory=MissionConstraint)


class DroneHeartbeat(BaseModel):
    system_id: str
    last_heartbeat: datetime | None = None
    mode: str = "UNKNOWN"
    armed: bool = False
    battery_percent: float = Field(default=100.0, ge=0, le=100)
    position: Coordinate | None = None
    heading_deg: float | None = Field(default=None, ge=0, le=360)
    a2a_url: str | None = None
    role: Literal["leader", "leaf", "standby"] = "leaf"
    status: Literal["ready", "busy", "degraded", "offline"] = "ready"


class DroneState(DroneHeartbeat):
    stale: bool = False
    age_seconds: float = 0.0


class PlanningNode(BaseModel):
    id: str
    type: Literal["start", "waypoint", "condition", "action", "control", "terminal"]
    action: str
    command_type: CommandType = CommandType.mission_upload
    assigned_to: str | None = None
    params: dict[str, Any] = Field(default_factory=dict)


class PlanningEdge(BaseModel):
    source: str = Field(alias="from")
    target: str = Field(alias="to")
    condition: str = "success"

    model_config = ConfigDict(populate_by_name=True)


class PlanningGraph(BaseModel):
    graph_id: str = Field(default_factory=lambda: f"pg_{uuid4().hex[:12]}")
    mission_id: str = Field(default_factory=lambda: f"mission_{uuid4().hex[:12]}")
    version: str = "1.0"
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
    intent: StructuredIntent
    assigned_drones: list[str] = Field(default_factory=list)
    constraints: MissionConstraint = Field(default_factory=MissionConstraint)
    nodes: list[PlanningNode]
    edges: list[PlanningEdge]
    metadata: dict[str, Any] = Field(default_factory=dict)


class SafetySeverity(str, Enum):
    info = "info"
    warning = "warning"
    error = "error"
    critical = "critical"


class SafetyIssue(BaseModel):
    rule_id: str
    severity: SafetySeverity
    message: str
    node_id: str | None = None
    detail: dict[str, Any] = Field(default_factory=dict)


class SafetyReport(BaseModel):
    passed: bool
    issues: list[SafetyIssue] = Field(default_factory=list)

    @property
    def blocking_issues(self) -> list[SafetyIssue]:
        return [i for i in self.issues if i.severity in {SafetySeverity.error, SafetySeverity.critical}]


class MissionPlanRequest(BaseModel):
    intent: str
    target_drones: list[str] = Field(default_factory=list)
    constraints: MissionConstraint | None = None


class MissionPlanResponse(BaseModel):
    status: Literal["success", "failed"]
    structured_intent: StructuredIntent | None = None
    planning_graph: PlanningGraph | None = None
    safety_report: SafetyReport


class MissionDeployRequest(MissionPlanRequest):
    dispatch_mode: DispatchMode = DispatchMode.stream


class DispatchResult(BaseModel):
    drone_id: str
    status: Literal["queued", "sent", "streaming", "completed", "failed", "skipped"]
    endpoint: str | None = None
    detail: dict[str, Any] = Field(default_factory=dict)


class MissionDeployResponse(BaseModel):
    status: Literal["accepted", "failed"]
    mission_id: str | None = None
    graph_id: str | None = None
    safety_report: SafetyReport
    dispatch_results: list[DispatchResult] = Field(default_factory=list)
    planning_graph: PlanningGraph | None = None


class SomaEvent(BaseModel):
    mission_id: str | None = None
    graph_id: str | None = None
    drone_id: str
    event: str
    data: dict[str, Any] = Field(default_factory=dict)
    timestamp: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class A2APart(BaseModel):
    kind: Literal["text", "data"] = "data"
    text: str | None = None
    data: dict[str, Any] | None = None


class A2AMessage(BaseModel):
    message_id: str = Field(default_factory=lambda: f"msg_{uuid4().hex[:12]}")
    role: Literal["mind", "soma", "supervisor"] = "mind"
    type: str = "mission_upload"
    parts: list[A2APart]
    metadata: dict[str, Any] = Field(default_factory=dict)


class A2AJsonRpcRequest(BaseModel):
    jsonrpc: Literal["2.0"] = "2.0"
    id: str = Field(default_factory=lambda: uuid4().hex)
    method: str = "message/send"
    params: dict[str, Any]


class A2AJsonRpcResponse(BaseModel):
    jsonrpc: Literal["2.0"] = "2.0"
    id: str | None = None
    result: dict[str, Any] | None = None
    error: dict[str, Any] | None = None

    @model_validator(mode="after")
    def _result_or_error(self) -> "A2AJsonRpcResponse":
        if self.result is None and self.error is None:
            raise ValueError("result or error must be present")
        return self
