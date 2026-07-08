#!/usr/bin/env python3
"""Town10HD_Opt.xodr(OpenDRIVE) 도로 중심선 → drone NED 상대 웨이포인트(JSON).

'도로 모양 상대비행': CARLA↔드론 절대 정렬 불가(georef 없음)하므로, 도로의 *모양*만
추출해 드론 시작점 기준 상대 NED로 변환. 드론이 실제 도로 궤적(직선/커브)을 추종.

CARLA/OpenDRIVE 좌표(x=East, y=North, m) → AirSim NED(x=North, y=East): nx=cy, ny=cx.
첫 점을 원점으로 빼서 '상대 오프셋' 리스트 출력 → 비행 스크립트가 드론 현재 NED에 더함.

사용: python3 scripts/extract_road_waypoints.py [--road-id N] [--max-len 200] [--step 8] [--scale 1.0] [--alt 30] [--out wp.json]
"""
import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_XODR = Path.home() / "workspace/engines/CarlaUE5/Unreal/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town10HD_Opt.xodr"


def sample_geometry(g, step):
    x = float(g.get("x", 0)); y = float(g.get("y", 0))
    hdg = float(g.get("hdg", 0)); length = float(g.get("length", 0))
    arc = g.find("arc")
    n = max(2, int(math.ceil(length / step)) + 1)
    pts = []
    k = float(arc.get("curvature")) if (arc is not None) else 0.0
    for i in range(n):
        s = length * i / (n - 1)
        if abs(k) > 1e-9:
            pts.append((x + (math.sin(hdg + k * s) - math.sin(hdg)) / k,
                        y - (math.cos(hdg + k * s) - math.cos(hdg)) / k))
        else:
            pts.append((x + math.cos(hdg) * s, y + math.sin(hdg) * s))
    return pts


def road_points(road, step):
    pv = road.find("planView")
    if pv is None:
        return []
    pts = []
    for g in pv.findall("geometry"):
        seg = sample_geometry(g, step)
        if pts and seg and math.dist(pts[-1], seg[0]) < 1e-6:
            seg = seg[1:]
        pts.extend(seg)
    return pts


def path_length(pts):
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xodr", default=str(DEFAULT_XODR))
    ap.add_argument("--road-id", type=int, default=None, help="특정 road id (미지정시 가장 긴 도로)")
    ap.add_argument("--step", type=float, default=8.0, help="웨이포인트 간격(m)")
    ap.add_argument("--max-len", type=float, default=220.0, help="경로 최대 길이(m, 초과분 절단)")
    ap.add_argument("--scale", type=float, default=1.0, help="크기 배율(1=실측)")
    ap.add_argument("--alt", type=float, default=30.0, help="비행 고도(m)")
    ap.add_argument("--out", default="recordings/road_waypoints.json")
    args = ap.parse_args()

    root = ET.parse(args.xodr).getroot()
    roads = [(r.get("id"), road_points(r, args.step)) for r in root.findall("road")]
    roads = [(rid, p) for rid, p in roads if len(p) >= 2]

    if args.road_id is not None:
        sel = [(rid, p) for rid, p in roads if rid == str(args.road_id)]
        if not sel:
            print(f"road id {args.road_id} 없음. 가용 일부: {[r[0] for r in roads[:15]]}"); return 2
        rid, pts = sel[0]
    else:
        rid, pts = max(roads, key=lambda rp: path_length(rp[1]))

    # 길이 절단
    trimmed = [pts[0]]
    acc = 0.0
    for i in range(1, len(pts)):
        acc += math.dist(pts[i - 1], pts[i])
        trimmed.append(pts[i])
        if acc >= args.max_len:
            break
    pts = trimmed

    # CARLA(x=E,y=N) → NED(x=N,y=E), 첫 점 원점화 + scale
    cx0, cy0 = pts[0]
    wps = [[(cy - cy0) * args.scale, (cx - cx0) * args.scale, -args.alt] for (cx, cy) in pts]

    out = Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    meta = {"map": Path(args.xodr).stem, "road_id": rid, "alt": args.alt, "scale": args.scale,
            "count": len(wps), "length_m": round(path_length(pts) * args.scale, 1),
            "waypoints_ned_rel": wps}
    out.write_text(json.dumps(meta, indent=1))
    ns = [p[0] for p in wps]; es = [p[1] for p in wps]
    print(f"road id={rid}, {len(wps)}점, 길이 {meta['length_m']}m (scale {args.scale})")
    print(f"  NED 상대범위: N {min(ns):.0f}~{max(ns):.0f}  E {min(es):.0f}~{max(es):.0f}  alt {args.alt}m")
    print(f"  → {out}")


if __name__ == "__main__":
    raise SystemExit(main())
