from fastapi.testclient import TestClient

from app.main import app
from app.domain.models import Coordinate, DroneHeartbeat

client = TestClient(app)


def test_plan_without_connected_drone():
    response = client.post(
        "/api/v1/mission/plan",
        json={"intent": "A구역 산악 지역 정찰 임무. 고도 120m 이하 이동 후 목표 지점 사진 촬영.", "target_drones": ["drone1"]},
    )
    assert response.status_code == 200
    body = response.json()
    assert body["status"] in {"success", "failed"}
    assert body["planning_graph"]["nodes"]


def test_heartbeat_and_deploy_stream_accepted_even_if_soma_unreachable():
    hb = DroneHeartbeat(
        system_id="drone1",
        mode="GUIDED",
        armed=True,
        battery_percent=88,
        position=Coordinate(lat=37.5326, lon=127.0246, alt_m=30),
        a2a_url="http://127.0.0.1:9",
    )
    response = client.post("/api/v1/drones/drone1/heartbeat", json=hb.model_dump(mode="json"))
    assert response.status_code == 200
    response = client.post(
        "/api/v1/mission/deploy",
        json={"intent": "A구역 정찰. 고도 100m 이하.", "target_drones": ["drone1"], "dispatch_mode": "stream"},
    )
    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "accepted"
    assert body["mission_id"]
