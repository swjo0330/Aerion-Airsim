from __future__ import annotations

import asyncio
import json
from typing import Any

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from app.core.time import iso_now
from app.domain.models import DroneHeartbeat, MissionDeployRequest, MissionPlanRequest, SomaEvent
from app.services.drone_state_cache import drone_state_cache
from app.services.event_bus import mission_event_bus
from app.services.mission_service import mission_service
from app.services.mission_store import mission_store

router = APIRouter()


def _sse(event: str, data: dict[str, Any]) -> str:
    return f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False)}\n\n"


@router.get("/health")
async def health():
    return {"status": "ok", "service": "aerion-mind", "timestamp": iso_now()}


@router.get("/drones")
async def list_drones():
    return {"items": [drone.model_dump(mode="json") for drone in drone_state_cache.all()]}


@router.post("/drones/{drone_id}/heartbeat")
async def upsert_heartbeat(drone_id: str, heartbeat: DroneHeartbeat):
    if heartbeat.system_id != drone_id:
        raise HTTPException(status_code=400, detail="path drone_id and heartbeat.system_id mismatch")
    state = drone_state_cache.upsert(heartbeat)
    return {"status": "ok", "drone": state.model_dump(mode="json")}


@router.delete("/drones/{drone_id}")
async def delete_drone(drone_id: str):
    return {"removed": drone_state_cache.remove(drone_id)}


@router.post("/mission/plan")
async def plan_mission(request: MissionPlanRequest):
    response = mission_service.plan(
        request.intent,
        request.target_drones,
        request.constraints,
        require_connected_drones=False,
    )
    return response.model_dump(mode="json")


@router.post("/mission/deploy")
async def deploy_mission(request: MissionDeployRequest):
    response = await mission_service.deploy(
        request.intent,
        request.target_drones,
        request.constraints,
        request.dispatch_mode,
    )
    return response.model_dump(mode="json")


@router.get("/mission")
async def list_missions():
    return {
        "items": [
            {
                "mission_id": rec.mission_id,
                "graph_id": rec.graph_id,
                "status": rec.status,
                "created_at": rec.created_at.isoformat(),
                "updated_at": rec.updated_at.isoformat(),
                "event_count": len(rec.events),
            }
            for rec in mission_store.all()
        ]
    }


@router.get("/mission/{mission_id}")
async def get_mission(mission_id: str):
    rec = mission_store.get(mission_id)
    if rec is None:
        raise HTTPException(status_code=404, detail="mission not found")
    return {
        "mission_id": rec.mission_id,
        "graph_id": rec.graph_id,
        "status": rec.status,
        "graph": rec.graph.model_dump(mode="json"),
        "safety_report": rec.safety_report.model_dump(mode="json"),
        "events": rec.events,
    }


@router.get("/mission/{mission_id}/events")
async def mission_events(mission_id: str):
    if mission_store.get(mission_id) is None:
        raise HTTPException(status_code=404, detail="mission not found")

    async def gen():
        yield _sse("connected", {"mission_id": mission_id, "timestamp": iso_now()})
        async for event in mission_event_bus.subscribe(mission_id):
            yield _sse(event.get("event", "message"), event)

    return StreamingResponse(gen(), media_type="text/event-stream")


@router.post("/soma/events")
async def receive_soma_event(event: SomaEvent):
    data = event.model_dump(mode="json")
    mission_id = event.mission_id
    if mission_id:
        mission_store.append_event(mission_id, {"event": event.event, "drone_id": event.drone_id, "data": event.data, "timestamp": data["timestamp"]})
        await mission_event_bus.publish(mission_id, {"event": event.event, "drone_id": event.drone_id, "data": event.data, "timestamp": data["timestamp"]})
    return {"status": "accepted"}


@router.post("/a2a/message/send")
async def inbound_a2a_message(payload: dict[str, Any]):
    # Soma/Supervisor -> Mind callback path. This is intentionally generic.
    method = payload.get("method")
    params = payload.get("params", {})
    message = params.get("message", {})
    metadata = message.get("metadata", {}) if isinstance(message, dict) else {}
    mission_id = metadata.get("mission_id") or metadata.get("graph_id")
    if mission_id and mission_store.get(mission_id):
        mission_store.append_event(mission_id, {"event": method or "a2a", "data": payload, "timestamp": iso_now()})
        await mission_event_bus.publish(mission_id, {"event": method or "a2a", "data": payload, "timestamp": iso_now()})
    return {"jsonrpc": "2.0", "id": payload.get("id"), "result": {"status": "received"}}
