#!/usr/bin/env python3
"""Build reusable AERION local-coordinate mission JSON files."""

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path


def line_waypoints(count: int, spacing: float, altitude: float, heading_deg: float):
    heading = math.radians(heading_deg)
    east_step = math.sin(heading) * spacing
    north_step = math.cos(heading) * spacing
    return [
        {
            'east': east_step * index,
            'north': north_step * index,
            'up': altitude,
        }
        for index in range(count)
    ]


def box_waypoints(north_size: float, east_size: float, altitude: float, close: bool):
    waypoints = [
        {'east': 0.0, 'north': 0.0, 'up': altitude},
        {'east': 0.0, 'north': north_size, 'up': altitude},
        {'east': east_size, 'north': north_size, 'up': altitude},
        {'east': east_size, 'north': 0.0, 'up': altitude},
    ]
    if close:
        waypoints.append({'east': 0.0, 'north': 0.0, 'up': altitude})
    return waypoints


def orbit_waypoints(radius: float, count: int, altitude: float, clockwise: bool):
    direction = -1.0 if clockwise else 1.0
    waypoints = []
    for index in range(count):
        angle = direction * 2.0 * math.pi * index / count
        waypoints.append(
            {
                'east': radius * math.sin(angle),
                'north': radius * math.cos(angle),
                'up': altitude,
            }
        )
    waypoints.append(waypoints[0].copy())
    return waypoints


def road_waypoints(path: Path):
    data = json.loads(path.read_text(encoding='utf-8'))
    waypoints = []
    for north, east, down in data.get('waypoints_ned_rel', []):
        waypoints.append({'east': east, 'north': north, 'up': -down})
    if not waypoints:
        raise ValueError(f'no waypoints_ned_rel found in {path}')
    return waypoints, data


def _float_from(row: dict, names: tuple[str, ...], default: float | None = None) -> float:
    for name in names:
        if name in row and row[name] not in ('', None):
            return float(row[name])
    if default is not None:
        return default
    raise KeyError(names[0])


def _waypoint_from_mapping(row: dict, default_altitude: float):
    frame = str(row.get('frame', row.get('coordinate_frame', 'local_enu'))).lower()
    if frame in ('local_enu', 'enu'):
        return {
            'east': _float_from(row, ('east', 'e', 'x'), 0.0),
            'north': _float_from(row, ('north', 'n', 'y'), 0.0),
            'up': _float_from(row, ('up', 'u', 'z', 'altitude', 'alt'), default_altitude),
        }
    if frame in ('local_ned', 'airsim_ned', 'ned'):
        return {
            'east': _float_from(row, ('east', 'e', 'y'), 0.0),
            'north': _float_from(row, ('north', 'n', 'x'), 0.0),
            'up': -_float_from(row, ('down', 'd', 'z'), -default_altitude),
        }
    if frame in ('carla', 'opendrive', 'carla_opendrive'):
        return {
            'east': _float_from(row, ('x', 'east', 'e'), 0.0),
            'north': _float_from(row, ('y', 'north', 'n'), 0.0),
            'up': _float_from(row, ('z', 'up', 'altitude', 'alt'), default_altitude),
        }
    raise ValueError(f'unsupported import frame: {frame}')


def import_csv_waypoints(path: Path, default_altitude: float):
    with path.open(newline='', encoding='utf-8') as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError(f'CSV has no header: {path}')
        return [_waypoint_from_mapping(row, default_altitude) for row in reader]


def import_json_waypoints(path: Path, default_altitude: float):
    data = json.loads(path.read_text(encoding='utf-8'))
    if 'waypoints_ned_rel' in data:
        return road_waypoints(path)[0], {'source_format': 'waypoints_ned_rel', 'metadata': data}

    if isinstance(data, list):
        return [_waypoint_from_mapping(item, default_altitude) for item in data], {'source_format': 'json_list'}

    frame = str(data.get('frame', data.get('coordinate_frame', 'local_enu'))).lower()
    metadata = data.get('metadata') or {}
    local_source = metadata.get('local_source') if isinstance(metadata, dict) else None
    if frame in ('global', 'gps', 'wgs84') and isinstance(local_source, dict):
        local_frame = str(local_source.get('coordinate_frame', 'local_enu')).lower()
        local_waypoints = local_source.get('waypoints') or []
        if local_waypoints:
            waypoints = [
                _waypoint_from_mapping(
                    {'coordinate_frame': item.get('coordinate_frame', item.get('frame', local_frame)), **item}
                    if isinstance(item, dict)
                    else {'coordinate_frame': local_frame, 'east': item[0], 'north': item[1], 'up': item[2]},
                    default_altitude,
                )
                for item in local_waypoints
            ]
            return waypoints, {'source_format': 'json_gps_local_source', 'input_metadata': {k: v for k, v in data.items() if k != 'waypoints'}}

    waypoints = []
    for item in data.get('waypoints', []):
        if isinstance(item, dict):
            item = {'coordinate_frame': item.get('coordinate_frame', item.get('frame', frame)), **item}
            waypoints.append(_waypoint_from_mapping(item, default_altitude))
        elif isinstance(item, (list, tuple)):
            if frame in ('local_enu', 'enu'):
                east, north = float(item[0]), float(item[1])
                up = float(item[2]) if len(item) > 2 else default_altitude
            elif frame in ('local_ned', 'airsim_ned', 'ned'):
                north, east = float(item[0]), float(item[1])
                up = -float(item[2]) if len(item) > 2 else default_altitude
            elif frame in ('carla', 'opendrive', 'carla_opendrive'):
                east, north = float(item[0]), float(item[1])
                up = float(item[2]) if len(item) > 2 else default_altitude
            else:
                raise ValueError(f'unsupported import frame: {frame}')
            waypoints.append({'east': east, 'north': north, 'up': up})

    return waypoints, {'source_format': 'json', 'input_metadata': {k: v for k, v in data.items() if k != 'waypoints'}}


def import_waypoints(path: Path, default_altitude: float):
    suffix = path.suffix.lower()
    if suffix == '.csv':
        return import_csv_waypoints(path, default_altitude), {'source_format': 'csv'}
    if suffix == '.json':
        return import_json_waypoints(path, default_altitude)
    raise ValueError(f'unsupported input file type: {path}')


def segment_length(a: dict, b: dict) -> float:
    return math.sqrt(
        (b['east'] - a['east']) ** 2
        + (b['north'] - a['north']) ** 2
        + (b['up'] - a['up']) ** 2
    )


def path_length(waypoints: list[dict]) -> float:
    return sum(segment_length(waypoints[i - 1], waypoints[i]) for i in range(1, len(waypoints)))


def resample_waypoints(waypoints: list[dict], spacing: float):
    if spacing <= 0.0 or len(waypoints) < 2:
        return waypoints

    output = [waypoints[0].copy()]
    carry = 0.0
    previous = waypoints[0].copy()

    for target in waypoints[1:]:
        start = previous
        seg_len = segment_length(start, target)
        while carry + seg_len >= spacing and seg_len > 1e-9:
            remain = spacing - carry
            ratio = remain / seg_len
            point = {
                'east': start['east'] + (target['east'] - start['east']) * ratio,
                'north': start['north'] + (target['north'] - start['north']) * ratio,
                'up': start['up'] + (target['up'] - start['up']) * ratio,
            }
            output.append(point)
            start = point
            seg_len = segment_length(start, target)
            carry = 0.0
        carry += seg_len
        previous = target

    if segment_length(output[-1], waypoints[-1]) > 1e-6:
        output.append(waypoints[-1].copy())
    return output


def smooth_waypoints(waypoints: list[dict], iterations: int):
    smoothed = [wp.copy() for wp in waypoints]
    for _ in range(max(0, iterations)):
        if len(smoothed) < 3:
            return smoothed
        output = [smoothed[0]]
        for a, b in zip(smoothed[:-1], smoothed[1:]):
            output.append(
                {
                    'east': 0.75 * a['east'] + 0.25 * b['east'],
                    'north': 0.75 * a['north'] + 0.25 * b['north'],
                    'up': 0.75 * a['up'] + 0.25 * b['up'],
                }
            )
            output.append(
                {
                    'east': 0.25 * a['east'] + 0.75 * b['east'],
                    'north': 0.25 * a['north'] + 0.75 * b['north'],
                    'up': 0.25 * a['up'] + 0.75 * b['up'],
                }
            )
        output.append(smoothed[-1])
        smoothed = output
    return smoothed


def validate_waypoints(
    waypoints: list[dict],
    min_altitude: float,
    max_altitude: float,
    max_segment: float,
    max_waypoints: int,
):
    errors = []
    warnings = []
    if len(waypoints) < 2:
        errors.append('mission needs at least 2 waypoints')
    if len(waypoints) > max_waypoints:
        errors.append(f'waypoint count {len(waypoints)} exceeds max {max_waypoints}')

    for index, wp in enumerate(waypoints, start=1):
        for axis in ('east', 'north', 'up'):
            value = wp.get(axis)
            if value is None or not math.isfinite(float(value)):
                errors.append(f'waypoint {index} has invalid {axis}: {value}')
        up = float(wp.get('up', 0.0))
        if up < min_altitude:
            errors.append(f'waypoint {index} altitude {up:.2f}m is below minimum {min_altitude:.2f}m')
        if up > max_altitude:
            errors.append(f'waypoint {index} altitude {up:.2f}m exceeds maximum {max_altitude:.2f}m')

    max_seen_segment = 0.0
    for index in range(1, len(waypoints)):
        length = segment_length(waypoints[index - 1], waypoints[index])
        max_seen_segment = max(max_seen_segment, length)
        if length > max_segment:
            warnings.append(
                f'segment {index}->{index + 1} is {length:.1f}m; consider --resample-spacing'
            )

    return {
        'errors': errors,
        'warnings': warnings,
        'count': len(waypoints),
        'length_m': path_length(waypoints),
        'max_segment_m': max_seen_segment,
        'min_altitude_m': min((wp['up'] for wp in waypoints), default=0.0),
        'max_altitude_m': max((wp['up'] for wp in waypoints), default=0.0),
    }


def load_heightmap(path: Path) -> dict:
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('schema') != 'aerion_heightmap_v1':
        raise ValueError(f"unsupported heightmap schema: {data.get('schema')}")
    for key in ('bounds', 'resolution_m', 'width', 'height', 'values'):
        if key not in data:
            raise ValueError(f'heightmap missing {key}: {path}')
    return data


def heightmap_height_at(heightmap: dict, map_x: float, map_y: float) -> float | None:
    bounds = heightmap['bounds']
    resolution = float(heightmap['resolution_m'])
    col = int(math.floor((map_x - bounds['west']) / resolution))
    row = int(math.floor((bounds['north'] - map_y) / resolution))
    width = int(heightmap['width'])
    height = int(heightmap['height'])
    if col < 0 or row < 0 or col >= width or row >= height:
        return None
    value = heightmap['values'][row * width + col]
    return None if value is None else float(value)


def _home_origin_from_metadata(extra: dict) -> tuple[float, float] | None:
    metadata = extra.get('input_metadata', {}).get('metadata', {})
    local_source = metadata.get('local_source') if isinstance(metadata, dict) else None
    if isinstance(local_source, dict):
        origin = local_source.get('home_origin_map_xy')
        if isinstance(origin, dict) and 'east' in origin and 'north' in origin:
            return float(origin['east']), float(origin['north'])
    origin = metadata.get('home_origin_map_xy')
    if isinstance(origin, dict) and 'east' in origin and 'north' in origin:
        return float(origin['east']), float(origin['north'])
    return None


def validate_heightmap_clearance(
    waypoints: list[dict],
    heightmap: dict,
    clearance: float,
    home_map_x: float,
    home_map_y: float,
    sample_spacing: float,
    unknown_policy: str,
):
    errors = []
    warnings = []
    checked_samples = 0
    unknown_samples = 0
    min_clearance = None

    def check_sample(label: str, local_east: float, local_north: float, up: float):
        nonlocal checked_samples, unknown_samples, min_clearance
        map_x = local_east + home_map_x
        map_y = local_north + home_map_y
        obstacle_up = heightmap_height_at(heightmap, map_x, map_y)
        checked_samples += 1
        if obstacle_up is None:
            unknown_samples += 1
            return
        actual_clearance = up - obstacle_up
        min_clearance = actual_clearance if min_clearance is None else min(min_clearance, actual_clearance)
        if actual_clearance < clearance:
            errors.append(
                f'{label} clearance {actual_clearance:.1f}m below required {clearance:.1f}m '
                f'at map({map_x:.1f},{map_y:.1f})'
            )

    for index, wp in enumerate(waypoints, start=1):
        check_sample(f'waypoint {index}', wp['east'], wp['north'], wp['up'])

    for index in range(1, len(waypoints)):
        a = waypoints[index - 1]
        b = waypoints[index]
        length = segment_length(a, b)
        samples = max(2, int(math.ceil(length / max(0.5, sample_spacing))))
        for sample in range(1, samples):
            ratio = sample / samples
            check_sample(
                f'segment {index}->{index + 1}',
                a['east'] + (b['east'] - a['east']) * ratio,
                a['north'] + (b['north'] - a['north']) * ratio,
                a['up'] + (b['up'] - a['up']) * ratio,
            )

    if unknown_samples:
        message = (
            f'heightmap has no data for {unknown_samples}/{checked_samples} checked sample(s); '
            'scan a larger area or use a conservative altitude'
        )
        if unknown_policy == 'error':
            errors.append(message)
        elif unknown_policy == 'warn':
            warnings.append(message)

    return {
        'errors': errors,
        'warnings': warnings,
        'checked_samples': checked_samples,
        'unknown_samples': unknown_samples,
        'required_clearance_m': clearance,
        'min_clearance_m': min_clearance,
        'home_map_x': home_map_x,
        'home_map_y': home_map_y,
        'heightmap_coverage': heightmap.get('coverage', {}),
    }


def write_mission(path: Path, name: str, source: str, waypoints: list[dict], extra: dict | None = None):
    payload = {
        'schema': 'aerion_mission_v1',
        'name': name,
        'created_at': datetime.now().isoformat(timespec='seconds'),
        'source': source,
        'coordinate_frame': 'local_enu',
        'waypoint_count': len(waypoints),
        'waypoints': waypoints,
    }
    if extra:
        payload['metadata'] = extra
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    return path


def parse_args():
    parser = argparse.ArgumentParser(description='Build local ENU mission JSON for aerion_gps_route_mission')
    parser.add_argument('--pattern', choices=('line', 'box', 'orbit', 'road', 'import'), required=True)
    parser.add_argument('--name', default='', help='mission name stored in the JSON')
    parser.add_argument('--out', type=Path, default=Path('recordings/missions/mission.json'))
    parser.add_argument('--input', type=Path, default=None, help='CSV/JSON path for --pattern import')
    parser.add_argument('--altitude', type=float, default=5.0, help='mission altitude above current home, meters')
    parser.add_argument('--count', type=int, default=5, help='line/orbit waypoint count')
    parser.add_argument('--spacing', type=float, default=10.0, help='line waypoint spacing, meters')
    parser.add_argument('--heading-deg', type=float, default=0.0, help='line heading: 0 north, 90 east')
    parser.add_argument('--north-size', type=float, default=30.0, help='box north side length, meters')
    parser.add_argument('--east-size', type=float, default=30.0, help='box east side length, meters')
    parser.add_argument('--radius', type=float, default=20.0, help='orbit radius, meters')
    parser.add_argument('--clockwise', action='store_true', help='orbit direction')
    parser.add_argument('--open-box', action='store_true', help='do not return to the first box point')
    parser.add_argument('--road-file', type=Path, default=Path('recordings/road_waypoints.json'))
    parser.add_argument('--resample-spacing', type=float, default=0.0, help='resample path at this spacing in meters')
    parser.add_argument('--smooth-iterations', type=int, default=0, help='Chaikin smoothing iterations')
    parser.add_argument('--min-altitude', type=float, default=0.5)
    parser.add_argument('--max-altitude', type=float, default=120.0)
    parser.add_argument('--max-segment', type=float, default=50.0)
    parser.add_argument('--max-waypoints', type=int, default=200)
    parser.add_argument('--heightmap', type=Path, default=None, help='aerion_heightmap_v1 JSON for clearance validation')
    parser.add_argument('--clearance', type=float, default=8.0, help='required clearance above heightmap obstacles')
    parser.add_argument('--home-map-x', type=float, default=None, help='OpenDRIVE/map X coordinate of FCU home')
    parser.add_argument('--home-map-y', type=float, default=None, help='OpenDRIVE/map Y coordinate of FCU home')
    parser.add_argument('--clearance-sample-spacing', type=float, default=2.0)
    parser.add_argument('--heightmap-unknown-policy', choices=('warn', 'error', 'ignore'), default='warn')
    parser.add_argument('--validate-only', action='store_true', help='validate input/generated path without writing')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.pattern == 'line':
        waypoints = line_waypoints(args.count, args.spacing, args.altitude, args.heading_deg)
        extra = {'count': args.count, 'spacing': args.spacing, 'heading_deg': args.heading_deg}
    elif args.pattern == 'box':
        waypoints = box_waypoints(args.north_size, args.east_size, args.altitude, not args.open_box)
        extra = {'north_size': args.north_size, 'east_size': args.east_size, 'closed': not args.open_box}
    elif args.pattern == 'orbit':
        waypoints = orbit_waypoints(args.radius, args.count, args.altitude, args.clockwise)
        extra = {'radius': args.radius, 'count': args.count, 'clockwise': args.clockwise}
    elif args.pattern == 'road':
        waypoints, road_meta = road_waypoints(args.road_file.expanduser())
        extra = {'road_file': str(args.road_file), 'road_metadata': road_meta}
    else:
        if args.input is None:
            raise ValueError('--input is required for --pattern import')
        waypoints, import_meta = import_waypoints(args.input.expanduser(), args.altitude)
        extra = {'input': str(args.input), **import_meta}

    if args.smooth_iterations:
        waypoints = smooth_waypoints(waypoints, args.smooth_iterations)
    if args.resample_spacing > 0.0:
        waypoints = resample_waypoints(waypoints, args.resample_spacing)

    validation = validate_waypoints(
        waypoints=waypoints,
        min_altitude=args.min_altitude,
        max_altitude=args.max_altitude,
        max_segment=args.max_segment,
        max_waypoints=args.max_waypoints,
    )

    if args.heightmap is not None:
        heightmap = load_heightmap(args.heightmap.expanduser())
        inferred_origin = _home_origin_from_metadata(extra)
        home_map_x = args.home_map_x
        home_map_y = args.home_map_y
        if home_map_x is None or home_map_y is None:
            if inferred_origin is None:
                raise ValueError(
                    '--home-map-x/--home-map-y are required with --heightmap unless the input mission '
                    'contains metadata.local_source.home_origin_map_xy or metadata.home_origin_map_xy'
                )
            home_map_x, home_map_y = inferred_origin
        clearance_validation = validate_heightmap_clearance(
            waypoints=waypoints,
            heightmap=heightmap,
            clearance=args.clearance,
            home_map_x=float(home_map_x),
            home_map_y=float(home_map_y),
            sample_spacing=args.clearance_sample_spacing,
            unknown_policy=args.heightmap_unknown_policy,
        )
        validation['heightmap_clearance'] = clearance_validation
        validation['errors'].extend(clearance_validation['errors'])
        validation['warnings'].extend(clearance_validation['warnings'])

    extra = {**extra, 'validation': validation}

    print(
        f"mission stats: count={validation['count']} length={validation['length_m']:.1f}m "
        f"max_segment={validation['max_segment_m']:.1f}m "
        f"alt={validation['min_altitude_m']:.1f}..{validation['max_altitude_m']:.1f}m"
    )
    for warning in validation['warnings']:
        print(f'WARN: {warning}')
    if validation['errors']:
        for error in validation['errors']:
            print(f'ERROR: {error}')
        return 2
    if args.validate_only:
        print('validation passed')
        return 0

    name = args.name or args.pattern
    output = write_mission(args.out.expanduser(), name, args.pattern, waypoints, extra)
    print(f'wrote {output} ({len(waypoints)} local_enu waypoint(s))')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
