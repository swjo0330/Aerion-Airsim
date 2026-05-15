import os
from dataclasses import dataclass


def _float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _int(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None or raw == "":
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def _csv(name: str, default: str) -> list[str]:
    raw = os.getenv(name, default)
    return [item.strip() for item in raw.split(",") if item.strip()]


@dataclass(frozen=True)
class Settings:
    mind_host: str = os.getenv("MIND_HOST", "0.0.0.0")
    mind_port: int = _int("MIND_PORT", 8020)
    cors_origins: list[str] = None  # type: ignore[assignment]
    max_altitude_m: float = _float("MAX_ALTITUDE_M", 500.0)
    min_altitude_m: float = _float("MIN_ALTITUDE_M", 5.0)
    max_speed_mps: float = _float("MAX_SPEED_MPS", 15.0)
    max_leg_distance_m: float = _float("MAX_LEG_DISTANCE_M", 2000.0)
    stale_drone_seconds: int = _int("STALE_DRONE_SECONDS", 30)
    min_battery_percent: float = _float("MIN_BATTERY_PERCENT", 20.0)
    soma_default_a2a_url: str = os.getenv("SOMA_DEFAULT_A2A_URL", "http://localhost:8011")
    dispatch_timeout_seconds: float = _float("DISPATCH_TIMEOUT_SECONDS", 15.0)
    default_dispatch_mode: str = os.getenv("DISPATCH_MODE", "stream")

    def __post_init__(self):
        object.__setattr__(self, "cors_origins", _csv("CORS_ORIGINS", "http://localhost:3000"))


settings = Settings()
