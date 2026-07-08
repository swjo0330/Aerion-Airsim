#!/usr/bin/env python3
"""Apply deterministic orientation transforms to a top-down map image."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image

FLIP_LEFT_RIGHT = getattr(getattr(Image, "Transpose", Image), "FLIP_LEFT_RIGHT")
FLIP_TOP_BOTTOM = getattr(getattr(Image, "Transpose", Image), "FLIP_TOP_BOTTOM")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata-input", type=Path, default=None)
    parser.add_argument("--metadata-output", type=Path, default=None)
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=0)
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--flip-y", action="store_true")
    parser.add_argument("--note", default="")
    args = parser.parse_args()

    image = Image.open(args.input).convert("RGB")
    if args.rotate:
        image = image.rotate(args.rotate, expand=True)
    if args.flip_x:
        image = image.transpose(FLIP_LEFT_RIGHT)
    if args.flip_y:
        image = image.transpose(FLIP_TOP_BOTTOM)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)

    if args.metadata_input and args.metadata_output:
        data = json.loads(args.metadata_input.read_text(encoding="utf-8"))
        data["image"] = str(args.output.resolve())
        data["width"] = image.width
        data["height"] = image.height
        data.setdefault("orientation_transform", {})
        data["orientation_transform"].update(
            {
                "rotate_deg_ccw": args.rotate,
                "flip_x_after_rotate": args.flip_x,
                "flip_y_after_rotate": args.flip_y,
                "note": args.note,
            }
        )
        args.metadata_output.write_text(json.dumps(data, indent=2), encoding="utf-8")

    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
