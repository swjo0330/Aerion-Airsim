#!/usr/bin/env python3
"""Build a standalone browser mission editor from a CARLA OpenDRIVE map."""

from __future__ import annotations

import argparse
import base64
import html
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


DEFAULT_XODR = (
    Path.home()
    / "workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr"
)
DEFAULT_OUTPUT = Path("recordings/maps/mission_editor.html")


def _float_attr(element: ET.Element, name: str, default: float = 0.0) -> float:
    value = element.attrib.get(name)
    return default if value is None else float(value)


def _sample_line(x: float, y: float, hdg: float, length: float, step_m: float) -> list[list[float]]:
    count = max(2, int(math.ceil(length / step_m)) + 1)
    return [
        [x + math.cos(hdg) * length * i / (count - 1), y + math.sin(hdg) * length * i / (count - 1)]
        for i in range(count)
    ]


def _sample_arc(
    x: float,
    y: float,
    hdg: float,
    length: float,
    curvature: float,
    step_m: float,
) -> list[list[float]]:
    count = max(2, int(math.ceil(length / step_m)) + 1)
    points: list[list[float]] = []
    for i in range(count):
        s = length * i / (count - 1)
        if abs(curvature) < 1e-9:
            points.append([x + math.cos(hdg) * s, y + math.sin(hdg) * s])
            continue
        points.append(
            [
                x + (math.sin(hdg + curvature * s) - math.sin(hdg)) / curvature,
                y - (math.cos(hdg + curvature * s) - math.cos(hdg)) / curvature,
            ]
        )
    return points


def _sample_geometry(geometry: ET.Element, step_m: float) -> list[list[float]]:
    x = _float_attr(geometry, "x")
    y = _float_attr(geometry, "y")
    hdg = _float_attr(geometry, "hdg")
    length = _float_attr(geometry, "length")
    arc = geometry.find("arc")
    if arc is not None:
        return _sample_arc(x, y, hdg, length, _float_attr(arc, "curvature"), step_m)
    return _sample_line(x, y, hdg, length, step_m)


def read_xodr(path: Path, step_m: float) -> dict:
    root = ET.parse(path).getroot()
    header = root.find("header")
    bounds: dict[str, float] = {}
    if header is not None:
        for key in ("west", "east", "south", "north"):
            if key in header.attrib:
                bounds[key] = float(header.attrib[key])

    roads = []
    for road in root.findall("road"):
        plan_view = road.find("planView")
        if plan_view is None:
            continue
        points: list[list[float]] = []
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

    if not roads:
        raise RuntimeError(f"No roads found in {path}")
    if not {"west", "east", "south", "north"}.issubset(bounds):
        xs = [point[0] for road in roads for point in road["points"]]
        ys = [point[1] for road in roads for point in road["points"]]
        bounds = {"west": min(xs), "east": max(xs), "south": min(ys), "north": max(ys)}

    pad = max(bounds["east"] - bounds["west"], bounds["north"] - bounds["south"]) * 0.04
    bounds = {
        "west": bounds["west"] - pad,
        "east": bounds["east"] + pad,
        "south": bounds["south"] - pad,
        "north": bounds["north"] + pad,
    }
    return {"map": path.stem, "bounds": bounds, "roads": roads}


def read_heightmap(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    required = {"schema", "bounds", "resolution_m", "width", "height", "values"}
    missing = required - set(data)
    if missing:
        raise ValueError(f"heightmap missing keys {sorted(missing)}: {path}")
    if data["schema"] != "aerion_heightmap_v1":
        raise ValueError(f"unsupported heightmap schema: {data.get('schema')}")
    return data


def read_background_image(path: Path, bounds: dict | None = None) -> dict:
    suffix = path.suffix.lower()
    if suffix == ".png":
        mime = "image/png"
    elif suffix == ".webp":
        mime = "image/webp"
    else:
        mime = "image/jpeg"
    encoded = base64.b64encode(path.read_bytes()).decode("ascii")
    return {
        "src": f"data:{mime};base64,{encoded}",
        "name": path.name,
        "bounds": bounds,
    }


def read_alignment(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != "aerion_topview_alignment_v1":
        raise ValueError(f"unsupported alignment schema: {data.get('schema')}")
    if data.get("method") not in ("affine", "similarity"):
        raise ValueError(f"unsupported alignment method: {data.get('method')}")
    if "pixel_to_map" not in data or "map_to_pixel" not in data:
        raise ValueError(f"alignment missing affine matrices: {path}")
    return data


def read_heightmap_alignment(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != "aerion_heightmap_alignment_v1":
        raise ValueError(f"unsupported heightmap alignment schema: {data.get('schema')}")
    if "heightmap_map_to_pixel" not in data or "pixel_to_heightmap_map" not in data:
        raise ValueError(f"heightmap alignment missing affine matrices: {path}")
    return data


HTML_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>AERION Mission Editor</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f4f5f1;
      --panel: #ffffff;
      --ink: #202124;
      --muted: #5f6368;
      --line: #d7d9d2;
      --accent: #1b7f5c;
      --danger: #c2413a;
      --warn: #b7791f;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: var(--ink);
      background: var(--bg);
      display: grid;
      grid-template-columns: minmax(0, 1fr) 360px;
      height: 100vh;
      overflow: hidden;
    }
    main { position: relative; min-width: 0; }
    canvas {
      width: 100%;
      height: 100%;
      display: block;
      background: #eceee7;
      cursor: crosshair;
    }
    aside {
      border-left: 1px solid var(--line);
      background: var(--panel);
      padding: 16px;
      overflow: auto;
    }
    h1 { font-size: 18px; margin: 0 0 14px; }
    h2 { font-size: 13px; margin: 18px 0 8px; color: var(--muted); text-transform: uppercase; letter-spacing: 0; }
    label { display: grid; gap: 5px; margin: 10px 0; font-size: 12px; color: var(--muted); }
    input, textarea, button {
      font: inherit;
      border-radius: 6px;
      border: 1px solid var(--line);
    }
    input, textarea {
      width: 100%;
      padding: 8px;
      color: var(--ink);
      background: #fff;
    }
    textarea {
      min-height: 116px;
      resize: vertical;
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
    }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 10px; }
    .checkline {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .checkline input {
      width: auto;
      margin: 0;
    }
    .compact-buttons {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      margin-top: 10px;
    }
    button {
      padding: 8px 10px;
      background: #f8f9f7;
      color: var(--ink);
      cursor: pointer;
    }
    button.primary { background: var(--accent); border-color: var(--accent); color: white; }
    button.danger { color: var(--danger); }
    button.active { outline: 2px solid var(--accent); outline-offset: 1px; }
    .status {
      padding: 9px;
      border: 1px solid var(--line);
      background: #fbfcfa;
      border-radius: 6px;
      font-size: 12px;
      line-height: 1.45;
      white-space: pre-wrap;
    }
    table { width: 100%; border-collapse: collapse; font-size: 12px; }
    td, th { border-bottom: 1px solid var(--line); padding: 5px 4px; text-align: right; }
    th:first-child, td:first-child { text-align: left; }
    tr.selected { background: #edf7f2; }
    .hint {
      position: absolute;
      left: 12px;
      bottom: 12px;
      color: var(--muted);
      background: rgba(255, 255, 255, 0.88);
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 7px 9px;
      font-size: 12px;
    }
    @media (max-width: 900px) {
      body { grid-template-columns: 1fr; grid-template-rows: minmax(0, 1fr) 45vh; }
      aside { border-left: 0; border-top: 1px solid var(--line); }
    }
  </style>
</head>
<body>
  <main>
    <canvas id="map"></canvas>
    <div class="hint">Click: add waypoint · Drag: pan · Wheel: zoom · Backspace: undo</div>
  </main>
  <aside>
    <h1>AERION Mission Editor</h1>
    <label>Mission name <input id="name" value="clicked_mission" /></label>
    <div class="row">
      <label>Clearance m <input id="clearance" type="number" value="8" step="1" /></label>
      <label>Default alt m <input id="defaultAlt" type="number" value="20" step="1" /></label>
    </div>
    <h2>Altitude</h2>
    <div class="row">
      <label>Next alt m <input id="currentAlt" type="number" value="20" step="1" /></label>
      <label>Step m <input id="altStep" type="number" value="5" step="1" /></label>
    </div>
    <label class="checkline"><input id="autoClearance" type="checkbox" checked /> Keep clearance</label>
    <div class="compact-buttons">
      <button id="altDown">- Step</button>
      <button id="altSafe">Safe</button>
      <button id="altUp">+ Step</button>
    </div>
    <div class="row">
      <label>Resample m <input id="spacing" type="number" value="8" step="1" /></label>
      <label>Max segment m <input id="maxSegment" type="number" value="12" step="1" /></label>
    </div>
    <div class="row">
      <label>Home map X <input id="originEast" type="number" value="__HOME_MAP_X__" step="1" /></label>
      <label>Home map Y <input id="originNorth" type="number" value="__HOME_MAP_Y__" step="1" /></label>
    </div>
    <label>Export axes
      <select id="axisTransform">
        <option value="airsim_ned_xy" selected>AirSim NED: map X→N, map Y→E</option>
        <option value="visual_screen">Visual Screen: screen ↑→N, screen →→E</option>
        <option value="map_enu">Map ENU: E→E, N→N</option>
        <option value="swap_xy">Swap XY: X→N, Y→E</option>
        <option value="rotate_ccw_90">Rotate CCW 90: E→N, N→-E</option>
        <option value="rotate_cw_90">Rotate CW 90: E→-N, N→E</option>
        <option value="invert_north">Invert North: E→E, N→-N</option>
        <option value="invert_east">Invert East: E→-E, N→N</option>
      </select>
    </label>
    <h2>Layers</h2>
    <div class="buttons">
      <button id="presetPhoto">Photo</button>
      <button id="presetSafety">Safety</button>
    </div>
    <div class="row">
      <label>Image <input id="imageOpacity" type="range" min="0" max="1" value="0.72" step="0.05" /></label>
      <label>Height <input id="heightOpacity" type="range" min="0" max="1" value="0.00" step="0.05" /></label>
    </div>
    <div class="row">
      <label>Min scan m <input id="heightMin" type="number" value="2" step="0.5" /></label>
      <label>Scan cells <input id="showScanCells" type="checkbox" checked /></label>
    </div>
    <div class="row">
      <label>H flip X <input id="heightFlipX" type="checkbox" /></label>
      <label>H flip Y <input id="heightFlipY" type="checkbox" /></label>
    </div>
    <div class="buttons">
      <button id="heightRotate180">H 180</button>
      <button id="heightReset">H Reset</button>
    </div>
    <h2>Obstacle Layer</h2>
    <textarea id="obstacles">[]</textarea>
    <div class="buttons">
      <button id="applyObstacles">Apply Layer</button>
      <button id="undo" class="danger">Undo</button>
      <button id="clear" class="danger">Clear</button>
      <button id="validate">Validate</button>
      <button id="originFirst">Home=First</button>
      <button id="downloadCsv">CSV</button>
      <button id="downloadJson" class="primary">Mission JSON</button>
    </div>
    <h2>Calibration</h2>
    <div class="buttons">
      <button id="calibPixel">Image Point</button>
      <button id="calibMap">Map Point</button>
      <button id="calibUndo" class="danger">Undo Pair</button>
      <button id="calibDownload" class="primary">Control JSON</button>
    </div>
    <div class="status" id="calibrationStatus"></div>
    <h2>Status</h2>
    <div class="status" id="status"></div>
    <h2>Waypoints</h2>
    <table>
      <thead><tr><th>#</th><th>E</th><th>N</th><th>Up</th></tr></thead>
      <tbody id="waypoints"></tbody>
    </table>
  </aside>
  <script id="map-data" type="application/json">__MAP_DATA__</script>
  <script>
    const mapData = JSON.parse(document.getElementById('map-data').textContent);
    const canvas = document.getElementById('map');
    const ctx = canvas.getContext('2d');
    const el = id => document.getElementById(id);
    const backgroundImage = mapData.backgroundImage ? new Image() : null;
    if (backgroundImage) {
      backgroundImage.onload = () => draw();
      backgroundImage.src = mapData.backgroundImage.src;
    }
    const state = {
      waypoints: [],
      obstacles: [],
      zoom: 1,
      panX: 0,
      panY: 0,
      dragging: false,
      dragStart: null,
      panStart: null,
      lastMouse: null,
      clickMode: 'mission',
      pendingPixel: null,
      controlPoints: [],
      selectedWaypoint: null,
      heightmapFlipX: Boolean(mapData.display?.heightmap_flip_x),
      heightmapFlipY: Boolean(mapData.display?.heightmap_flip_y),
      heightmapRotate180: Boolean(mapData.display?.heightmap_rotate_180),
    };

    function numberValue(id, fallback = 0) {
      const value = Number(el(id).value);
      return Number.isFinite(value) ? value : fallback;
    }

    function formatMeters(value) {
      if (!Number.isFinite(value)) return '0';
      const rounded = Math.round(value * 100) / 100;
      if (Number.isInteger(rounded)) return String(rounded);
      return rounded.toFixed(2).replace(/0+$/, '').replace(/\.$/, '');
    }

    function metersToScreenDirect(x, y) {
      const b = mapData.bounds;
      const spanX = b.east - b.west;
      const spanY = b.north - b.south;
      const scale = Math.min(canvas.width / spanX, canvas.height / spanY) * state.zoom;
      const cx = (canvas.width - spanX * scale) / 2 + state.panX;
      const cy = (canvas.height - spanY * scale) / 2 + state.panY;
      const sx = cx + (x - b.west) * scale;
      const sy = cy + (b.north - y) * scale;
      return [sx, mapData.display?.flip_y ? canvas.height - sy : sy];
    }

    function screenToMetersDirect(px, py) {
      const b = mapData.bounds;
      const spanX = b.east - b.west;
      const spanY = b.north - b.south;
      const scale = Math.min(canvas.width / spanX, canvas.height / spanY) * state.zoom;
      const cx = (canvas.width - spanX * scale) / 2 + state.panX;
      const cy = (canvas.height - spanY * scale) / 2 + state.panY;
      const sourceY = mapData.display?.flip_y ? canvas.height - py : py;
      return [b.west + (px - cx) / scale, b.north - (sourceY - cy) / scale];
    }

    function hasAlignedBackgroundImage() {
      return Boolean(mapData.alignment && backgroundImage && backgroundImage.complete && backgroundImage.naturalWidth);
    }

    function imagePixelToScreen(px, py) {
      if (!(backgroundImage && backgroundImage.naturalWidth)) return null;
      const b = mapData.backgroundImage?.bounds || (mapData.heightmap ? mapData.heightmap.bounds : mapData.bounds);
      const mapX = b.west + (px / backgroundImage.naturalWidth) * (b.east - b.west);
      const mapY = b.north - (py / backgroundImage.naturalHeight) * (b.north - b.south);
      return metersToScreenDirect(mapX, mapY);
    }

    function screenToImagePixelDirect(px, py) {
      if (!(backgroundImage && backgroundImage.naturalWidth)) return null;
      const [mapX, mapY] = screenToMetersDirect(px, py);
      const b = mapData.backgroundImage?.bounds || (mapData.heightmap ? mapData.heightmap.bounds : mapData.bounds);
      return [
        ((mapX - b.west) / (b.east - b.west)) * backgroundImage.naturalWidth,
        ((b.north - mapY) / (b.north - b.south)) * backgroundImage.naturalHeight,
      ];
    }

    function mapLayerPixelToDisplayPixel(px, py) {
      if (!(backgroundImage && backgroundImage.naturalWidth)) return [px, py];
      let x = px;
      let y = py;
      if (mapData.display?.map_rotate_180) {
        x = backgroundImage.naturalWidth - x;
        y = backgroundImage.naturalHeight - y;
      }
      if (mapData.display?.map_layer_flip_x) {
        x = backgroundImage.naturalWidth - x;
      }
      return [x, y];
    }

    function rotateMapPoint180(x, y) {
      if (!mapData.display?.map_rotate_180) return [x, y];
      const b = mapData.bounds;
      return [b.west + b.east - x, b.south + b.north - y];
    }

    function metersToScreen(x, y) {
      if (hasAlignedBackgroundImage()) {
        // Alignment files are captured in the displayed image orientation, so
        // map_rotate_180 must not be applied again on this path.
        const pixel = mapToImagePixel(x, y);
        const screen = pixel ? imagePixelToScreen(pixel[0], pixel[1]) : null;
        if (screen) return screen;
      }
      const [rx, ry] = rotateMapPoint180(x, y);
      return metersToScreenDirect(rx, ry);
    }

    function screenToMeters(px, py) {
      if (hasAlignedBackgroundImage()) {
        const pixel = screenToImagePixelDirect(px, py);
        const map = pixel ? imagePixelToMap(pixel[0], pixel[1]) : null;
        if (map) return map;
      }
      const [x, y] = screenToMetersDirect(px, py);
      return rotateMapPoint180(x, y);
    }

    function applyAffine(matrix, x, y) {
      return [
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2],
      ];
    }

    function imagePixelToMap(px, py) {
      const alignment = mapData.alignment;
      if (alignment && alignment.pixel_to_map) return applyAffine(alignment.pixel_to_map, px, py);
      if (!(backgroundImage && backgroundImage.naturalWidth)) return null;
      const b = mapData.backgroundImage?.bounds || (mapData.heightmap ? mapData.heightmap.bounds : mapData.bounds);
      return [
        b.west + (px / backgroundImage.naturalWidth) * (b.east - b.west),
        b.north - (py / backgroundImage.naturalHeight) * (b.north - b.south),
      ];
    }

    function mapToImagePixel(mapX, mapY) {
      const alignment = mapData.alignment;
      if (alignment && alignment.map_to_pixel) return applyAffine(alignment.map_to_pixel, mapX, mapY);
      if (!(backgroundImage && backgroundImage.naturalWidth)) return null;
      const b = mapData.backgroundImage?.bounds || (mapData.heightmap ? mapData.heightmap.bounds : mapData.bounds);
      return [
        ((mapX - b.west) / (b.east - b.west)) * backgroundImage.naturalWidth,
        ((b.north - mapY) / (b.north - b.south)) * backgroundImage.naturalHeight,
      ];
    }

    function mapToDisplayImagePixel(mapX, mapY) {
      const pixel = mapToImagePixel(mapX, mapY);
      // With topview alignment, pixel coordinates are already in display-space.
      if (mapData.alignment) return pixel;
      return pixel ? mapLayerPixelToDisplayPixel(pixel[0], pixel[1]) : null;
    }

    function heightmapMapToImagePixel(x, y) {
      const alignment = mapData.heightmapAlignment;
      if (alignment && alignment.heightmap_map_to_pixel) {
        return applyAffine(alignment.heightmap_map_to_pixel, x, y);
      }
      return mapToDisplayImagePixel(x, y);
    }

    function imagePixelToHeightmapMap(px, py) {
      const alignment = mapData.heightmapAlignment;
      if (alignment && alignment.pixel_to_heightmap_map) {
        return applyAffine(alignment.pixel_to_heightmap_map, px, py);
      }
      return imagePixelToMap(px, py);
    }

    function heightmapMapToScreen(x, y) {
      const pixel = heightmapMapToImagePixel(x, y);
      return pixel ? imagePixelToScreen(pixel[0], pixel[1]) : null;
    }

    function screenToImagePixel(px, py) {
      if (mapData.alignment && backgroundImage && backgroundImage.complete && backgroundImage.naturalWidth) {
        return screenToImagePixelDirect(px, py);
      }
      const [mapX, mapY] = screenToMeters(px, py);
      return mapToImagePixel(mapX, mapY);
    }

    function drawAlignedBackground(opacity = 1) {
      if (!(backgroundImage && backgroundImage.complete && backgroundImage.naturalWidth)) return false;
      const alignment = mapData.alignment;
      if (!(alignment && (alignment.method === 'affine' || alignment.method === 'similarity'))) return false;
      const matrix = alignment.pixel_to_map;
      const p0 = metersToScreen(matrix[0][2], matrix[1][2]);
      const px = metersToScreen(matrix[0][0] + matrix[0][2], matrix[1][0] + matrix[1][2]);
      const py = metersToScreen(matrix[0][1] + matrix[0][2], matrix[1][1] + matrix[1][2]);
      ctx.save();
      ctx.imageSmoothingEnabled = true;
      ctx.globalAlpha = opacity;
      ctx.setTransform(px[0] - p0[0], px[1] - p0[1], py[0] - p0[0], py[1] - p0[1], p0[0], p0[1]);
      ctx.drawImage(backgroundImage, 0, 0);
      ctx.restore();
      return true;
    }

    function heightmapHeightAt(x, y) {
      const hm = mapData.heightmap;
      if (!hm) return null;
      const b = hm.bounds;
      const flipHeightmapX = state.heightmapFlipX || state.heightmapRotate180;
      const flipHeightmapY = state.heightmapFlipY || state.heightmapRotate180;
      let sampleX = x;
      let sampleY = y;
      if (mapData.heightmapAlignment) {
        const displayPixel = mapToDisplayImagePixel(x, y);
        const heightmapPoint = displayPixel ? imagePixelToHeightmapMap(displayPixel[0], displayPixel[1]) : null;
        if (!heightmapPoint) return null;
        sampleX = heightmapPoint[0];
        sampleY = heightmapPoint[1];
      }
      sampleX = flipHeightmapX ? b.west + b.east - sampleX : sampleX;
      sampleY = flipHeightmapY ? b.south + b.north - sampleY : sampleY;
      const col = Math.floor((sampleX - b.west) / hm.resolution_m);
      const row = Math.floor((b.north - sampleY) / hm.resolution_m);
      if (col < 0 || row < 0 || col >= hm.width || row >= hm.height) return null;
      const value = hm.values[row * hm.width + col];
      return Number.isFinite(value) ? value : null;
    }

    function obstacleLayerHeightAt(x, y) {
      let height = 0;
      for (const ob of state.obstacles) {
        if (ob.type === 'rect') {
          const inside = x >= ob.x && x <= ob.x + ob.w && y >= ob.y && y <= ob.y + ob.h;
          if (inside) height = Math.max(height, Number(ob.height || 0));
        } else if (ob.type === 'circle') {
          const d = Math.hypot(x - ob.x, y - ob.y);
          if (d <= Number(ob.r || 0)) height = Math.max(height, Number(ob.height || 0));
        }
      }
      return height;
    }

    function obstacleHeightAt(x, y) {
      const scanned = heightmapHeightAt(x, y);
      return Math.max(scanned == null ? 0 : scanned, obstacleLayerHeightAt(x, y));
    }

    function safeAltitude(x, y) {
      return obstacleHeightAt(x, y) + numberValue('clearance');
    }

    function recommendedAltitude(x, y) {
      return Math.max(numberValue('defaultAlt'), safeAltitude(x, y));
    }

    function nextWaypointAltitude(x, y) {
      const planned = numberValue('currentAlt', numberValue('defaultAlt'));
      return el('autoClearance').checked ? Math.max(planned, safeAltitude(x, y)) : planned;
    }

    function adjustNextAltitude(delta) {
      el('currentAlt').value = (numberValue('currentAlt', numberValue('defaultAlt')) + delta).toFixed(1);
      refresh();
    }

    function setSafeNextAltitude() {
      const samplePoint = state.selectedWaypoint != null
        ? state.waypoints[state.selectedWaypoint]
        : state.waypoints[state.waypoints.length - 1];
      if (samplePoint) {
        el('currentAlt').value = recommendedAltitude(samplePoint.east, samplePoint.north).toFixed(1);
      } else if (state.lastMouse) {
        const [east, north] = screenToMeters(state.lastMouse[0], state.lastMouse[1]);
        el('currentAlt').value = recommendedAltitude(east, north).toFixed(1);
      } else {
        el('currentAlt').value = numberValue('defaultAlt').toFixed(1);
      }
      refresh();
    }

    function applyAltitudeToSelected() {
      if (state.selectedWaypoint == null) return;
      const point = state.waypoints[state.selectedWaypoint];
      if (!point) return;
      point.up = nextWaypointAltitude(point.east, point.north);
      refresh();
    }

    function removeLastWaypoint() {
      state.waypoints.pop();
      if (state.selectedWaypoint != null && state.selectedWaypoint >= state.waypoints.length) {
        state.selectedWaypoint = state.waypoints.length ? state.waypoints.length - 1 : null;
      }
      refresh();
    }

    function origin() {
      return {
        east: Number(el('originEast').value || 0),
        north: Number(el('originNorth').value || 0),
      };
    }

    function axisTransformName() {
      return el('axisTransform') ? el('axisTransform').value : 'map_enu';
    }

    function transformLocalAxes(east, north) {
      switch (axisTransformName()) {
        case 'airsim_ned_xy':
        case 'swap_xy':
          return {east: north, north: east};
        case 'visual_screen':
          return {east, north: mapData.display?.flip_y ? -north : north};
        case 'rotate_ccw_90':
          return {east: -north, north: east};
        case 'rotate_cw_90':
          return {east: north, north: -east};
        case 'invert_north':
          return {east, north: -north};
        case 'invert_east':
          return {east: -east, north};
        default:
          return {east, north};
      }
    }

    function gpsOrigin() {
      const origin = mapData.gpsOrigin || {};
      return {
        latitude: Number(origin.latitude ?? origin.Latitude ?? 37.5665),
        longitude: Number(origin.longitude ?? origin.Longitude ?? 126.9780),
        altitude: Number(origin.altitude ?? origin.Altitude ?? 38),
      };
    }

    function toLocal(point) {
      const o = origin();
      const local = transformLocalAxes(point.east - o.east, point.north - o.north);
      return {east: local.east, north: local.north, up: point.up};
    }

    function localToGps(point) {
      const earthRadiusM = 6378137.0;
      const origin = gpsOrigin();
      const local = toLocal(point);
      const latRad = origin.latitude * Math.PI / 180;
      return {
        latitude: origin.latitude + (local.north / earthRadiusM) * 180 / Math.PI,
        longitude: origin.longitude + (local.east / (earthRadiusM * Math.cos(latRad))) * 180 / Math.PI,
        altitude: local.up,
      };
    }

    function resample(points, spacing) {
      if (points.length < 2 || spacing <= 0) return points.map(p => ({...p}));
      const out = [{...points[0]}];
      let carry = 0;
      for (let i = 1; i < points.length; i++) {
        let start = {...points[i - 1]};
        const target = points[i];
        let length = Math.hypot(target.east - start.east, target.north - start.north, target.up - start.up);
        while (carry + length >= spacing && length > 1e-9) {
          const ratio = (spacing - carry) / length;
          const point = {
            east: start.east + (target.east - start.east) * ratio,
            north: start.north + (target.north - start.north) * ratio,
            up: start.up + (target.up - start.up) * ratio,
          };
          out.push(point);
          start = point;
          length = Math.hypot(target.east - start.east, target.north - start.north, target.up - start.up);
          carry = 0;
        }
        carry += length;
      }
      const last = points[points.length - 1];
      const prev = out[out.length - 1];
      if (Math.hypot(last.east - prev.east, last.north - prev.north, last.up - prev.up) > 1e-6) out.push({...last});
      return out;
    }

    function validate(points = resample(state.waypoints, Number(el('spacing').value || 0))) {
      const warnings = [];
      const errors = [];
      const clearance = Number(el('clearance').value || 0);
      const maxSegment = Number(el('maxSegment').value || 0);
      let unknownHeightmapSamples = 0;
      let checkedHeightmapSamples = 0;
      if (points.length < 2) errors.push('mission needs at least 2 waypoints');
      for (let i = 0; i < points.length; i++) {
        const p = points[i];
        if (mapData.heightmap) {
          checkedHeightmapSamples += 1;
          if (heightmapHeightAt(p.east, p.north) == null) unknownHeightmapSamples += 1;
        }
        const required = obstacleHeightAt(p.east, p.north) + clearance;
        if (p.up < required) errors.push(`waypoint ${i + 1} needs ${required.toFixed(1)}m clearance altitude`);
      }
      for (let i = 1; i < points.length; i++) {
        const a = points[i - 1], b = points[i];
        const len = Math.hypot(b.east - a.east, b.north - a.north, b.up - a.up);
        if (maxSegment > 0 && len > maxSegment) warnings.push(`segment ${i}->${i + 1} is ${len.toFixed(1)}m`);
        const samples = Math.max(2, Math.ceil(len / 2));
        for (let j = 0; j <= samples; j++) {
          const t = j / samples;
          const x = a.east + (b.east - a.east) * t;
          const y = a.north + (b.north - a.north) * t;
          const up = a.up + (b.up - a.up) * t;
          if (mapData.heightmap) {
            checkedHeightmapSamples += 1;
            if (heightmapHeightAt(x, y) == null) unknownHeightmapSamples += 1;
          }
          const required = obstacleHeightAt(x, y) + clearance;
          if (up < required) {
            errors.push(`segment ${i}->${i + 1} clearance fail near E ${x.toFixed(1)}, N ${y.toFixed(1)}`);
            break;
          }
        }
      }
      if (unknownHeightmapSamples) {
        warnings.push(`heightmap has no data for ${unknownHeightmapSamples}/${checkedHeightmapSamples} checked samples`);
      }
      const length = points.slice(1).reduce((sum, p, i) => {
        const a = points[i];
        return sum + Math.hypot(p.east - a.east, p.north - a.north, p.up - a.up);
      }, 0);
      return {errors, warnings, count: points.length, length};
    }

    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.lineCap = 'round';

      const imageOpacity = Number(el('imageOpacity').value || 0);
      const heightOpacity = Number(el('heightOpacity').value || 0);

      if (backgroundImage && backgroundImage.complete && backgroundImage.naturalWidth) {
        const b = mapData.backgroundImage.bounds || (mapData.heightmap ? mapData.heightmap.bounds : mapData.bounds);
        const [x1, y1] = metersToScreenDirect(b.west, b.north);
        const [x2, y2] = metersToScreenDirect(b.east, b.south);
        ctx.imageSmoothingEnabled = false;
        ctx.globalAlpha = imageOpacity;
        const left = Math.min(x1, x2);
        const top = Math.min(y1, y2);
        const width = Math.abs(x2 - x1);
        const height = Math.abs(y2 - y1);
        ctx.save();
        ctx.translate(mapData.display?.flip_x ? left + width : left, mapData.display?.flip_y ? top + height : top);
        ctx.scale(mapData.display?.flip_x ? -1 : 1, mapData.display?.flip_y ? -1 : 1);
        ctx.drawImage(backgroundImage, 0, 0, width, height);
        ctx.restore();
        ctx.globalAlpha = 1.0;
        ctx.imageSmoothingEnabled = true;
      }

      for (const ob of state.obstacles) {
        ctx.fillStyle = 'rgba(194, 65, 58, 0.22)';
        ctx.strokeStyle = 'rgba(194, 65, 58, 0.82)';
        ctx.lineWidth = 1.5;
        if (ob.type === 'rect') {
          const [x1, y1] = metersToScreen(ob.x, ob.y);
          const [x2, y2] = metersToScreen(ob.x + ob.w, ob.y + ob.h);
          ctx.fillRect(x1, y2, x2 - x1, y1 - y2);
          ctx.strokeRect(x1, y2, x2 - x1, y1 - y2);
        } else if (ob.type === 'circle') {
          const [cx, cy] = metersToScreen(ob.x, ob.y);
          const [rx] = metersToScreen(ob.x + ob.r, ob.y);
          ctx.beginPath();
          ctx.arc(cx, cy, Math.abs(rx - cx), 0, Math.PI * 2);
          ctx.fill();
          ctx.stroke();
        }
      }

      if (mapData.heightmap && heightOpacity > 0 && el('showScanCells').checked) {
        const hm = mapData.heightmap;
        const step = 1;
        const maxHeight = hm.values.reduce((m, v) => Number.isFinite(v) ? Math.max(m, v) : m, 0);
        const minVisibleHeight = Number(el('heightMin').value || 0);
        const labels = hm.labels || [];
        for (let row = 0; row < hm.height; row += step) {
          for (let col = 0; col < hm.width; col += step) {
            const value = hm.values[row * hm.width + col];
            if (!Number.isFinite(value)) continue;
            if (value < minVisibleHeight) continue;
            const x = hm.bounds.west + (col + 0.5) * hm.resolution_m;
            const y = hm.bounds.north - (row + 0.5) * hm.resolution_m;
            const flipHeightmapX = state.heightmapFlipX || state.heightmapRotate180;
            const flipHeightmapY = state.heightmapFlipY || state.heightmapRotate180;
            const drawX = flipHeightmapX ? hm.bounds.west + hm.bounds.east - x : x;
            const drawY = flipHeightmapY ? hm.bounds.south + hm.bounds.north - y : y;
            const half = hm.resolution_m * step * 0.5;
            const corners = [
              heightmapMapToScreen(drawX - half, drawY - half),
              heightmapMapToScreen(drawX + half, drawY - half),
              heightmapMapToScreen(drawX + half, drawY + half),
              heightmapMapToScreen(drawX - half, drawY + half),
            ];
            if (corners.some(p => !p)) continue;
            const alpha = Math.min(0.82, 0.28 + (value / Math.max(1, maxHeight)) * 0.54) * heightOpacity;
            const label = labels[row * hm.width + col] || '';
            if (label === 'Vegetation' || label === 'Terrain') {
              ctx.fillStyle = `rgba(35, 126, 83, ${alpha})`;
            } else if (label === 'Buildings' || label === 'Walls' || label === 'Fences') {
              ctx.fillStyle = `rgba(219, 87, 61, ${alpha})`;
            } else if (label === 'Poles' || label === 'TrafficSigns' || label === 'TrafficLight') {
              ctx.fillStyle = `rgba(126, 87, 194, ${alpha})`;
            } else if (label === 'Car' || label === 'Truck' || label === 'Bus' || label === 'Motorcycle' || label === 'Bicycle') {
              ctx.fillStyle = `rgba(25, 118, 210, ${alpha})`;
            } else {
              ctx.fillStyle = `rgba(230, 148, 43, ${alpha})`;
            }
            ctx.beginPath();
            ctx.moveTo(corners[0][0], corners[0][1]);
            for (const corner of corners.slice(1)) ctx.lineTo(corner[0], corner[1]);
            ctx.closePath();
            ctx.fill();
          }
        }
      }

      const sampled = resample(state.waypoints, Number(el('spacing').value || 0));
      if (sampled.length > 1) {
        ctx.beginPath();
        sampled.forEach((p, i) => {
          const [x, y] = metersToScreen(p.east, p.north);
          if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.strokeStyle = '#1b7f5c';
        ctx.lineWidth = 2.5;
        ctx.stroke();
      }

      state.waypoints.forEach((p, i) => {
        const [x, y] = metersToScreen(p.east, p.north);
        ctx.fillStyle = '#ffffff';
        ctx.strokeStyle = i === state.selectedWaypoint ? '#c2413a' : '#1b7f5c';
        ctx.lineWidth = i === state.selectedWaypoint ? 3 : 2;
        ctx.beginPath();
        ctx.arc(x, y, 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = '#202124';
        ctx.font = '12px sans-serif';
        ctx.fillText(String(i + 1), x + 8, y - 8);
        ctx.fillStyle = '#1b7f5c';
        ctx.fillText(`${formatMeters(p.up)}m`, x + 8, y + 8);
      });

      state.controlPoints.forEach((point, i) => {
        const [x, y] = metersToScreen(point.map[0], point.map[1]);
        ctx.strokeStyle = '#0b65c2';
        ctx.fillStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = '#0b65c2';
        ctx.font = '12px sans-serif';
        ctx.fillText(`C${i + 1}`, x + 7, y + 14);
      });
    }

    function setClickMode(mode) {
      state.clickMode = mode;
      ['calibPixel', 'calibMap'].forEach(id => el(id).classList.remove('active'));
      if (mode === 'calib_pixel') el('calibPixel').classList.add('active');
      if (mode === 'calib_map') el('calibMap').classList.add('active');
      refresh();
    }

    function calibrationPayload() {
      return {
        schema: 'aerion_topview_control_points_v1',
        image: mapData.backgroundImage?.name || '',
        heightmap_display_transform: {
          flip_x: state.heightmapFlipX,
          flip_y: state.heightmapFlipY,
          rotate_180: state.heightmapRotate180,
        },
        control_points: state.controlPoints,
      };
    }

    function updateCalibrationStatus() {
      const pending = state.pendingPixel
        ? `pending image pixel: ${state.pendingPixel[0].toFixed(1)}, ${state.pendingPixel[1].toFixed(1)}\n`
        : '';
      el('calibrationStatus').textContent =
        `mode: ${state.clickMode}\n${pending}pairs: ${state.controlPoints.length}\n` +
        state.controlPoints.slice(-6).map((p, i) => {
          const idx = state.controlPoints.length - Math.min(6, state.controlPoints.length) + i + 1;
          return `C${idx} px(${p.pixel[0].toFixed(1)},${p.pixel[1].toFixed(1)}) -> map(${p.map[0].toFixed(1)},${p.map[1].toFixed(1)})`;
        }).join('\n');
    }

    function refresh() {
      const sampled = resample(state.waypoints, Number(el('spacing').value || 0));
      const result = validate(sampled);
      const screenTopAxis = mapData.display?.map_rotate_180
        ? (mapData.display?.flip_y ? '+North' : '-North')
        : (mapData.display?.flip_y ? '-North' : '+North');
      const screenRightAxis = mapData.display?.map_rotate_180 ? '-East' : '+East';
      el('status').textContent =
        `map: ${mapData.map}\nalignment: ${mapData.alignment ? `${mapData.alignment.method}, rms ${Number(mapData.alignment.rms_error_m || 0).toFixed(2)}m` : 'not loaded'}\ndisplay axes: screen up ${screenTopAxis}, screen right ${screenRightAxis}\nexport axes: ${axisTransformName()}\ngps origin: ${gpsOrigin().latitude.toFixed(7)}, ${gpsOrigin().longitude.toFixed(7)}, alt ${gpsOrigin().altitude.toFixed(1)}m\nheightmap: ${mapData.heightmap ? `${mapData.heightmap.width}x${mapData.heightmap.height}, coverage ${(mapData.heightmap.coverage.ratio * 100).toFixed(1)}%` : 'not loaded'}\nhome map xy: ${origin().east.toFixed(1)}, ${origin().north.toFixed(1)}\nraw waypoints: ${state.waypoints.length}\nmission waypoints: ${result.count}\nlength: ${result.length.toFixed(1)}m\n` +
        `heightmap transform: flipX=${state.heightmapFlipX}, flipY=${state.heightmapFlipY}, rotate180=${state.heightmapRotate180}\n` +
        (result.warnings.length ? `warnings:\n- ${result.warnings.join('\n- ')}\n` : '') +
        (result.errors.length ? `errors:\n- ${result.errors.join('\n- ')}` : 'validation: passed');
      el('waypoints').innerHTML = state.waypoints.map((p, i) =>
        `<tr data-index="${i}" class="${i === state.selectedWaypoint ? 'selected' : ''}"><td>${i + 1}</td><td>${toLocal(p).east.toFixed(1)}</td><td>${toLocal(p).north.toFixed(1)}</td><td><input data-index="${i}" value="${formatMeters(p.up)}" /></td></tr>`
      ).join('');
      el('waypoints').querySelectorAll('tr').forEach(row => {
        row.addEventListener('click', () => {
          state.selectedWaypoint = Number(row.dataset.index);
          el('currentAlt').value = state.waypoints[state.selectedWaypoint].up.toFixed(1);
          refresh();
        });
      });
      el('waypoints').querySelectorAll('input').forEach(input => {
        input.addEventListener('change', () => {
          const index = Number(input.dataset.index);
          state.waypoints[index].up = Number(input.value);
          state.selectedWaypoint = index;
          el('currentAlt').value = state.waypoints[index].up.toFixed(1);
          refresh();
        });
      });
      updateCalibrationStatus();
      draw();
    }

    function download(name, mime, text) {
      const a = document.createElement('a');
      a.href = URL.createObjectURL(new Blob([text], {type: mime}));
      a.download = name;
      a.click();
      URL.revokeObjectURL(a.href);
    }

    function missionPayload() {
      const mapPoints = resample(state.waypoints, Number(el('spacing').value || 0));
      const points = mapPoints.map(localToGps);
      const result = validate(mapPoints);
      return {
        schema: 'aerion_mission_v1',
        name: el('name').value || 'clicked_mission',
        source: 'mission_editor',
        coordinate_frame: 'gps',
        waypoint_count: points.length,
        waypoints: points,
        metadata: {
          map: mapData.map,
          gps_origin: gpsOrigin(),
          altitude_mode: 'relative',
          validation: result,
        },
      };
    }

    function resize() {
      const rect = canvas.getBoundingClientRect();
      canvas.width = Math.max(1, Math.floor(rect.width * devicePixelRatio));
      canvas.height = Math.max(1, Math.floor(rect.height * devicePixelRatio));
      ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
      canvas.width = rect.width;
      canvas.height = rect.height;
      refresh();
    }

    canvas.addEventListener('mousedown', event => {
      state.dragging = false;
      state.dragStart = [event.offsetX, event.offsetY];
      state.panStart = [state.panX, state.panY];
    });
    canvas.addEventListener('mousemove', event => {
      state.lastMouse = [event.offsetX, event.offsetY];
      if (!state.dragStart) return;
      const dx = event.offsetX - state.dragStart[0];
      const dy = event.offsetY - state.dragStart[1];
      if (Math.hypot(dx, dy) > 3) {
        state.dragging = true;
        state.panX = state.panStart[0] + dx;
        state.panY = state.panStart[1] + dy;
        draw();
      }
    });
    canvas.addEventListener('mouseup', event => {
      const wasDragging = state.dragging;
      state.dragStart = null;
      if (wasDragging) return;
      const [east, north] = screenToMeters(event.offsetX, event.offsetY);
      if (state.clickMode === 'calib_pixel') {
        const pixel = screenToImagePixel(event.offsetX, event.offsetY);
        if (!pixel) {
          el('calibrationStatus').textContent = 'No background image is loaded.';
          return;
        }
        state.pendingPixel = pixel;
        setClickMode('calib_map');
        return;
      }
      if (state.clickMode === 'calib_map') {
        if (!state.pendingPixel) {
          el('calibrationStatus').textContent = 'Click Image Point first.';
          setClickMode('calib_pixel');
          return;
        }
        const index = state.controlPoints.length + 1;
        state.controlPoints.push({
          name: `cp_${String(index).padStart(2, '0')}`,
          pixel: [state.pendingPixel[0], state.pendingPixel[1]],
          map: [east, north],
        });
        state.pendingPixel = null;
        setClickMode('calib_pixel');
        return;
      }
      state.waypoints.push({east, north, up: nextWaypointAltitude(east, north)});
      state.selectedWaypoint = state.waypoints.length - 1;
      el('currentAlt').value = state.waypoints[state.selectedWaypoint].up.toFixed(1);
      refresh();
    });
    canvas.addEventListener('wheel', event => {
      event.preventDefault();
      const factor = event.deltaY < 0 ? 1.12 : 0.89;
      state.zoom = Math.min(16, Math.max(0.4, state.zoom * factor));
      draw();
    }, {passive: false});
    window.addEventListener('keydown', event => {
      if (event.key === 'Backspace') {
        removeLastWaypoint();
      }
    });
    window.addEventListener('resize', resize);

    el('applyObstacles').addEventListener('click', () => {
      try {
        state.obstacles = JSON.parse(el('obstacles').value);
        refresh();
      } catch (error) {
        el('status').textContent = `Obstacle JSON error: ${error.message}`;
      }
    });
    el('undo').addEventListener('click', removeLastWaypoint);
    el('clear').addEventListener('click', () => { state.waypoints = []; state.selectedWaypoint = null; refresh(); });
    el('validate').addEventListener('click', refresh);
    ['imageOpacity', 'heightOpacity', 'heightMin', 'showScanCells'].forEach(id => {
      el(id).addEventListener('input', draw);
      el(id).addEventListener('change', draw);
    });
    function syncHeightmapControls() {
      el('heightFlipX').checked = state.heightmapFlipX;
      el('heightFlipY').checked = state.heightmapFlipY;
    }
    el('heightFlipX').addEventListener('change', () => {
      state.heightmapFlipX = el('heightFlipX').checked;
      refresh();
    });
    el('heightFlipY').addEventListener('change', () => {
      state.heightmapFlipY = el('heightFlipY').checked;
      refresh();
    });
    el('heightRotate180').addEventListener('click', () => {
      state.heightmapRotate180 = !state.heightmapRotate180;
      refresh();
    });
    el('heightReset').addEventListener('click', () => {
      state.heightmapFlipX = false;
      state.heightmapFlipY = false;
      state.heightmapRotate180 = false;
      syncHeightmapControls();
      refresh();
    });
    el('axisTransform').addEventListener('input', refresh);
    el('axisTransform').addEventListener('change', refresh);
    el('altDown').addEventListener('click', () => {
      adjustNextAltitude(-Math.abs(numberValue('altStep', 1)));
      applyAltitudeToSelected();
    });
    el('altUp').addEventListener('click', () => {
      adjustNextAltitude(Math.abs(numberValue('altStep', 1)));
      applyAltitudeToSelected();
    });
    el('altSafe').addEventListener('click', () => {
      setSafeNextAltitude();
      applyAltitudeToSelected();
    });
    el('presetPhoto').addEventListener('click', () => {
      el('imageOpacity').value = 1;
      el('heightOpacity').value = 0;
      el('heightMin').value = 2;
      el('showScanCells').checked = true;
      draw();
    });
    el('presetSafety').addEventListener('click', () => {
      el('imageOpacity').value = 0.82;
      el('heightOpacity').value = 0.85;
      el('heightMin').value = 2;
      el('showScanCells').checked = true;
      draw();
    });
    el('calibPixel').addEventListener('click', () => setClickMode('calib_pixel'));
    el('calibMap').addEventListener('click', () => setClickMode('calib_map'));
    el('calibUndo').addEventListener('click', () => {
      if (state.pendingPixel) state.pendingPixel = null;
      else state.controlPoints.pop();
      refresh();
    });
    el('calibDownload').addEventListener('click', () => {
      download('topview_control_points.json', 'application/json', JSON.stringify(calibrationPayload(), null, 2));
    });
    el('originFirst').addEventListener('click', () => {
      if (!state.waypoints.length) return;
      el('originEast').value = state.waypoints[0].east.toFixed(3);
      el('originNorth').value = state.waypoints[0].north.toFixed(3);
      refresh();
    });
    el('downloadJson').addEventListener('click', () => {
      const payload = missionPayload();
      download(`${payload.name}.json`, 'application/json', JSON.stringify(payload, null, 2));
    });
    el('downloadCsv').addEventListener('click', () => {
      const lines = ['east,north,up', ...state.waypoints.map(p => {
        const local = toLocal(p);
        return `${local.east.toFixed(3)},${local.north.toFixed(3)},${local.up.toFixed(3)}`;
      })];
      download(`${el('name').value || 'clicked_mission'}.csv`, 'text/csv', lines.join('\n') + '\n');
    });
    ['clearance', 'defaultAlt', 'currentAlt', 'altStep', 'spacing', 'maxSegment', 'name', 'originEast', 'originNorth', 'autoClearance'].forEach(id => el(id).addEventListener('change', refresh));

    state.obstacles = JSON.parse(el('obstacles').value);
    syncHeightmapControls();
    resize();
  </script>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xodr", type=Path, default=DEFAULT_XODR, help="Input OpenDRIVE .xodr map")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Standalone HTML output")
    parser.add_argument("--heightmap", type=Path, default=None, help="Optional aerion_heightmap_v1 JSON")
    parser.add_argument("--heightmap-alignment", type=Path, default=None, help="Optional aerion_heightmap_alignment_v1 JSON")
    parser.add_argument("--alignment", type=Path, default=None, help="Optional aerion_topview_alignment_v1 JSON")
    parser.add_argument("--background-image", type=Path, default=None, help="Optional top-down PNG/JPG background")
    parser.add_argument(
        "--background-bounds",
        nargs=4,
        type=float,
        metavar=("WEST", "EAST", "SOUTH", "NORTH"),
        help="Map bounds covered by --background-image",
    )
    parser.add_argument("--home-map-x", type=float, default=0.0, help="Default FCU home OpenDRIVE/map X")
    parser.add_argument("--home-map-y", type=float, default=0.0, help="Default FCU home OpenDRIVE/map Y")
    parser.add_argument("--gps-origin-lat", type=float, default=37.5665, help="OriginGeopoint latitude for GPS export")
    parser.add_argument("--gps-origin-lon", type=float, default=126.9780, help="OriginGeopoint longitude for GPS export")
    parser.add_argument("--gps-origin-alt", type=float, default=38.0, help="OriginGeopoint altitude for GPS export metadata")
    parser.add_argument("--display-flip-x", action="store_true", help="Flip the whole editor display horizontally")
    parser.add_argument("--display-flip-y", action="store_true", help="Flip the whole editor display vertically")
    parser.add_argument("--map-layer-flip-x", action="store_true", help="Flip only map-derived layers horizontally")
    parser.add_argument("--heightmap-flip-x", action="store_true", help="Flip only the heightmap layer horizontally")
    parser.add_argument("--heightmap-flip-y", action="store_true", help="Flip only the heightmap layer vertically")
    parser.add_argument("--heightmap-rotate-180", action="store_true", help="Rotate only the heightmap layer 180 degrees")
    parser.add_argument("--map-rotate-180", action="store_true", help="Rotate only map-derived layers by 180 degrees")
    parser.add_argument("--step-m", type=float, default=2.0, help="Sampling distance for road rendering")
    args = parser.parse_args()

    xodr_path = args.xodr.expanduser().resolve()
    if not xodr_path.exists():
        raise FileNotFoundError(f"OpenDRIVE map not found: {xodr_path}")

    map_data = read_xodr(xodr_path, args.step_m)
    if args.heightmap is not None:
        map_data["heightmap"] = read_heightmap(args.heightmap.expanduser().resolve())
    if args.heightmap_alignment is not None:
        map_data["heightmapAlignment"] = read_heightmap_alignment(args.heightmap_alignment.expanduser().resolve())
    if args.alignment is not None:
        map_data["alignment"] = read_alignment(args.alignment.expanduser().resolve())
    if args.background_image is not None:
        bg_bounds = None
        if args.background_bounds:
            west, east, south, north = args.background_bounds
            bg_bounds = {"west": west, "east": east, "south": south, "north": north}
        map_data["backgroundImage"] = read_background_image(args.background_image.expanduser().resolve(), bg_bounds)
    map_data["gpsOrigin"] = {
        "latitude": args.gps_origin_lat,
        "longitude": args.gps_origin_lon,
        "altitude": args.gps_origin_alt,
    }
    map_data["display"] = {
        "flip_x": bool(args.display_flip_x),
        "flip_y": bool(args.display_flip_y),
        "map_layer_flip_x": bool(args.map_layer_flip_x),
        "heightmap_flip_x": bool(args.heightmap_flip_x),
        "heightmap_flip_y": bool(args.heightmap_flip_y),
        "heightmap_rotate_180": bool(args.heightmap_rotate_180),
        "map_rotate_180": bool(args.map_rotate_180),
    }
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    encoded = html.escape(json.dumps(map_data, separators=(",", ":")), quote=False)
    html_text = (
        HTML_TEMPLATE.replace("__MAP_DATA__", encoded)
        .replace("__HOME_MAP_X__", f"{args.home_map_x:.3f}")
        .replace("__HOME_MAP_Y__", f"{args.home_map_y:.3f}")
    )
    output.write_text(html_text, encoding="utf-8")
    print(f"Wrote {output} ({len(map_data['roads'])} roads)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
