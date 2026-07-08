#!/usr/bin/env python3
"""Spawn and drive CARLA vehicles/walkers through the Traffic Manager."""

from __future__ import annotations

import argparse
import logging
import random
import signal
import sys
import time
from typing import Any

import carla


def _get_blueprints(world: carla.World, pattern: str, generation: str) -> list[Any]:
    blueprints = list(world.get_blueprint_library().filter(pattern))
    if generation.lower() == "all" or len(blueprints) <= 1:
        return blueprints
    try:
        wanted = int(generation)
    except ValueError:
        return []
    return [bp for bp in blueprints if bp.has_attribute("generation") and int(bp.get_attribute("generation")) == wanted]


def _spawn_vehicles(
    client: carla.Client,
    world: carla.World,
    traffic_manager: Any,
    count: int,
    filter_pattern: str,
    generation: str,
    safe_only: bool,
    speed_diff: float,
) -> list[int]:
    if count <= 0:
        return []

    blueprints = _get_blueprints(world, filter_pattern, generation)
    if safe_only:
        blueprints = [
            bp
            for bp in blueprints
            if bp.has_attribute("base_type") and bp.get_attribute("base_type").as_string() == "car"
        ]
    if not blueprints:
        raise RuntimeError(f"No vehicle blueprints matched {filter_pattern!r}")

    spawn_points = list(world.get_map().get_spawn_points())
    random.shuffle(spawn_points)
    count = min(count, len(spawn_points))

    batch = []
    for transform in spawn_points[:count]:
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute("color"):
            blueprint.set_attribute("color", random.choice(blueprint.get_attribute("color").recommended_values))
        if blueprint.has_attribute("driver_id"):
            blueprint.set_attribute("driver_id", random.choice(blueprint.get_attribute("driver_id").recommended_values))
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "autopilot")
        batch.append(
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetAutopilot(carla.command.FutureActor, True, traffic_manager.get_port())
            )
        )

    actor_ids: list[int] = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logging.warning("vehicle spawn failed: %s", response.error)
        else:
            actor_ids.append(response.actor_id)

    for vehicle in world.get_actors(actor_ids):
        traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_diff)
        traffic_manager.auto_lane_change(vehicle, True)

    return actor_ids


def _enable_existing_vehicle_autopilot(
    world: carla.World,
    traffic_manager: Any,
    speed_diff: float,
) -> int:
    enabled = 0
    for actor in world.get_actors().filter("vehicle.*"):
        actor.set_autopilot(True, traffic_manager.get_port())
        traffic_manager.vehicle_percentage_speed_difference(actor, speed_diff)
        traffic_manager.auto_lane_change(actor, True)
        enabled += 1
    return enabled


def _spawn_walkers(
    client: carla.Client,
    world: carla.World,
    count: int,
    filter_pattern: str,
    generation: str,
    running_fraction: float,
    crossing_fraction: float,
) -> tuple[list[int], list[int]]:
    if count <= 0:
        return [], []

    blueprints = _get_blueprints(world, filter_pattern, generation)
    if not blueprints:
        raise RuntimeError(f"No walker blueprints matched {filter_pattern!r}")

    walker_transforms = []
    for _ in range(count * 3):
        if len(walker_transforms) >= count:
            break
        location = world.get_random_location_from_navigation()
        if location is None:
            continue
        location.z += 2.0
        walker_transforms.append(carla.Transform(location))

    walker_speeds: list[float] = []
    batch = []
    for transform in walker_transforms:
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute("is_invincible"):
            blueprint.set_attribute("is_invincible", "false")
        speed = 1.4
        if blueprint.has_attribute("speed"):
            values = blueprint.get_attribute("speed").recommended_values
            speed = float(values[2] if random.random() < running_fraction and len(values) > 2 else values[1])
        walker_speeds.append(speed)
        batch.append(carla.command.SpawnActor(blueprint, transform))

    walker_ids: list[int] = []
    valid_speeds: list[float] = []
    for index, response in enumerate(client.apply_batch_sync(batch, True)):
        if response.error:
            logging.warning("walker spawn failed: %s", response.error)
        else:
            walker_ids.append(response.actor_id)
            valid_speeds.append(walker_speeds[index])

    controller_bp = world.get_blueprint_library().find("controller.ai.walker")
    batch = [carla.command.SpawnActor(controller_bp, carla.Transform(), walker_id) for walker_id in walker_ids]
    controller_ids: list[int] = []
    for response in client.apply_batch_sync(batch, True):
        if response.error:
            logging.warning("walker controller spawn failed: %s", response.error)
        else:
            controller_ids.append(response.actor_id)

    world.set_pedestrians_cross_factor(crossing_fraction)
    controllers = world.get_actors(controller_ids)
    for controller, speed in zip(controllers, valid_speeds):
        controller.start()
        destination = world.get_random_location_from_navigation()
        if destination is not None:
            controller.go_to_location(destination)
        controller.set_max_speed(speed)

    return walker_ids, controller_ids


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--vehicles", type=int, default=20)
    parser.add_argument("--walkers", type=int, default=20)
    parser.add_argument("--filterv", default="vehicle.*")
    parser.add_argument("--filterw", default="walker.pedestrian.*")
    parser.add_argument("--generationv", default="All")
    parser.add_argument("--generationw", default="All")
    parser.add_argument("--safe", action="store_true", default=True)
    parser.add_argument("--speed-diff", type=float, default=-15.0, help="Negative means faster than speed limit")
    parser.add_argument("--running-fraction", type=float, default=0.08)
    parser.add_argument("--crossing-fraction", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--enable-existing", action="store_true", default=True)
    parser.add_argument("--keep-existing-only", action="store_true")
    parser.add_argument("--no-cleanup", action="store_true", help="Leave actors alive when this script exits")
    args = parser.parse_args()

    logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
    random.seed(args.seed)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    traffic_manager = client.get_trafficmanager(args.tm_port)
    traffic_manager.set_global_distance_to_leading_vehicle(3.0)
    traffic_manager.set_random_device_seed(args.seed)
    traffic_manager.set_synchronous_mode(False)

    spawned_vehicle_ids: list[int] = []
    spawned_walker_ids: list[int] = []
    spawned_controller_ids: list[int] = []
    stop = False

    def _stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        if args.enable_existing:
            existing = _enable_existing_vehicle_autopilot(world, traffic_manager, args.speed_diff)
            logging.info("enabled Traffic Manager autopilot for %d existing vehicle(s)", existing)

        if not args.keep_existing_only:
            spawned_vehicle_ids = _spawn_vehicles(
                client,
                world,
                traffic_manager,
                args.vehicles,
                args.filterv,
                args.generationv,
                args.safe,
                args.speed_diff,
            )
            spawned_walker_ids, spawned_controller_ids = _spawn_walkers(
                client,
                world,
                args.walkers,
                args.filterw,
                args.generationw,
                args.running_fraction,
                args.crossing_fraction,
            )

        logging.info(
            "traffic running: spawned %d vehicle(s), %d walker(s), %d walker controller(s)",
            len(spawned_vehicle_ids),
            len(spawned_walker_ids),
            len(spawned_controller_ids),
        )
        while not stop:
            time.sleep(0.5)
    finally:
        if args.no_cleanup:
            logging.info("leaving spawned traffic alive")
        else:
            for controller in world.get_actors(spawned_controller_ids):
                controller.stop()
            destroy_ids = spawned_controller_ids + spawned_walker_ids + spawned_vehicle_ids
            if destroy_ids:
                logging.info("destroying %d spawned actor(s)", len(destroy_ids))
                client.apply_batch([carla.command.DestroyActor(actor_id) for actor_id in destroy_ids])

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
