#!/usr/bin/env python3
"""Render a lightweight 2D overview from a CARLA OpenDRIVE (.xodr) map."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt


DEFAULT_XODR = (
    Path.home()
    / "workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr"
)
DEFAULT_OUTPUT_DIR = Path("docs/assets/maps")


def _float_attr(element: ET.Element, name: str, default: float = 0.0) -> float:
    value = element.attrib.get(name)
    return default if value is None else float(value)


def _sample_line(x: float, y: float, hdg: float, length: float, step_m: float) -> list[tuple[float, float]]:
    count = max(2, int(math.ceil(length / step_m)) + 1)
    return [
        (x + math.cos(hdg) * length * i / (count - 1), y + math.sin(hdg) * length * i / (count - 1))
        for i in range(count)
    ]


def _sample_arc(
    x: float,
    y: float,
    hdg: float,
    length: float,
    curvature: float,
    step_m: float,
) -> list[tuple[float, float]]:
    count = max(2, int(math.ceil(length / step_m)) + 1)
    points: list[tuple[float, float]] = []
    for i in range(count):
        s = length * i / (count - 1)
        if abs(curvature) < 1e-9:
            points.append((x + math.cos(hdg) * s, y + math.sin(hdg) * s))
            continue
        points.append(
            (
                x + (math.sin(hdg + curvature * s) - math.sin(hdg)) / curvature,
                y - (math.cos(hdg + curvature * s) - math.cos(hdg)) / curvature,
            )
        )
    return points


def _sample_geometry(geometry: ET.Element, step_m: float) -> list[tuple[float, float]]:
    x = _float_attr(geometry, "x")
    y = _float_attr(geometry, "y")
    hdg = _float_attr(geometry, "hdg")
    length = _float_attr(geometry, "length")

    arc = geometry.find("arc")
    if arc is not None:
        return _sample_arc(x, y, hdg, length, _float_attr(arc, "curvature"), step_m)
    return _sample_line(x, y, hdg, length, step_m)


def _read_roads(xodr_path: Path, step_m: float) -> tuple[list[dict], dict[str, float]]:
    root = ET.parse(xodr_path).getroot()
    header = root.find("header")
    bounds = {}
    if header is not None:
        for key in ("west", "east", "south", "north"):
            if key in header.attrib:
                bounds[key] = float(header.attrib[key])

    roads: list[dict] = []
    for road in root.findall("road"):
        plan_view = road.find("planView")
        if plan_view is None:
            continue
        points: list[tuple[float, float]] = []
        for geometry in plan_view.findall("geometry"):
            segment = _sample_geometry(geometry, step_m)
            if points and segment:
                segment = segment[1:]
            points.extend(segment)
        if len(points) >= 2:
            roads.append(
                {
                    "id": road.attrib.get("id", ""),
                    "name": road.attrib.get("name", ""),
                    "junction": road.attrib.get("junction", "-1"),
                    "points": points,
                }
            )
    return roads, bounds


def _draw_map(
    roads: list[dict],
    bounds: dict[str, float],
    output: Path,
    title: str,
    road_width: float,
    show_ids: bool,
    dpi: int,
) -> None:
    fig, ax = plt.subplots(figsize=(12, 9), dpi=dpi)
    ax.set_facecolor("#f7f7f4")

    for road in roads:
        xs = [point[0] for point in road["points"]]
        ys = [point[1] for point in road["points"]]
        is_junction = road["junction"] != "-1"
        ax.plot(xs, ys, color="#d8d8d0", linewidth=road_width + 2.0, solid_capstyle="round", zorder=1)
        ax.plot(xs, ys, color="#4d5156" if not is_junction else "#626a72", linewidth=road_width, solid_capstyle="round", zorder=2)
        ax.plot(xs, ys, color="#f2c94c", linewidth=0.8, alpha=0.9, zorder=3)

        if show_ids:
            mid = road["points"][len(road["points"]) // 2]
            label = road["name"] or f"Road {road['id']}"
            ax.text(mid[0], mid[1], label, fontsize=6, color="#202124", ha="center", va="center", zorder=4)

    if {"west", "east", "south", "north"}.issubset(bounds):
        pad = max(bounds["east"] - bounds["west"], bounds["north"] - bounds["south"]) * 0.04
        ax.set_xlim(bounds["west"] - pad, bounds["east"] + pad)
        ax.set_ylim(bounds["south"] - pad, bounds["north"] + pad)

    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.set_xlabel("OpenDRIVE X / Easting (m)")
    ax.set_ylabel("OpenDRIVE Y / Northing (m)")
    ax.grid(True, color="#d0d0cc", linewidth=0.4, alpha=0.6)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xodr", type=Path, default=DEFAULT_XODR, help="Input OpenDRIVE .xodr map")
    parser.add_argument("--output", type=Path, default=None, help="Output PNG/SVG path")
    parser.add_argument("--step-m", type=float, default=1.5, help="Sampling distance along road reference lines")
    parser.add_argument("--road-width", type=float, default=4.5, help="Rendered road line width")
    parser.add_argument("--show-ids", action="store_true", help="Draw road labels")
    parser.add_argument("--dpi", type=int, default=220, help="PNG output DPI")
    args = parser.parse_args()

    xodr_path = args.xodr.expanduser().resolve()
    if not xodr_path.exists():
        raise FileNotFoundError(f"OpenDRIVE map not found: {xodr_path}")

    output = args.output
    if output is None:
        output = DEFAULT_OUTPUT_DIR / f"{xodr_path.stem}_2d_map.png"
    output = output.expanduser().resolve()

    roads, bounds = _read_roads(xodr_path, args.step_m)
    if not roads:
        raise RuntimeError(f"No roads found in {xodr_path}")

    _draw_map(
        roads=roads,
        bounds=bounds,
        output=output,
        title=f"{xodr_path.stem} 2D OpenDRIVE Map",
        road_width=args.road_width,
        show_ids=args.show_ids,
        dpi=args.dpi,
    )
    print(f"Wrote {output} ({len(roads)} roads)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
