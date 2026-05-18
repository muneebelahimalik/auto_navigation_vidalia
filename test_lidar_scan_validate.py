#!/usr/bin/env python3

import argparse
import asyncio
import csv
import math
from pathlib import Path

from lidar.lidar_driver import LidarDriver


def save_csv(scan, out_csv):
    with open(out_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x_m", "y_m", "z_m", "range_m", "intensity", "ring", "azimuth_deg"])
        for p in scan:
            r = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
            writer.writerow([p.x, p.y, p.z, r, p.intensity, p.ring, p.azimuth])


def save_topdown_svg(scan, out_svg, max_range=6.0):
    width = 1200
    height = 1200
    scale = width / (2 * max_range)
    cx = width / 2
    cy = height / 2

    dots = []
    for p in scan:
        if abs(p.x) <= max_range and abs(p.y) <= max_range and -1.5 <= p.z <= 1.5:
            px = cx + p.x * scale
            py = cy - p.y * scale
            if 0 <= px <= width and 0 <= py <= height:
                dots.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="1" />')

    # Forward wedge, left/right labels, and sensor origin
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/>

<!-- grid -->
<line x1="{cx}" y1="0" x2="{cx}" y2="{height}" stroke="lightgray" stroke-width="1"/>
<line x1="0" y1="{cy}" x2="{width}" y2="{cy}" stroke="lightgray" stroke-width="1"/>

<!-- range rings -->
<circle cx="{cx}" cy="{cy}" r="{1*scale}" fill="none" stroke="#dddddd" stroke-width="1"/>
<circle cx="{cx}" cy="{cy}" r="{2*scale}" fill="none" stroke="#dddddd" stroke-width="1"/>
<circle cx="{cx}" cy="{cy}" r="{3*scale}" fill="none" stroke="#dddddd" stroke-width="1"/>
<circle cx="{cx}" cy="{cy}" r="{4*scale}" fill="none" stroke="#dddddd" stroke-width="1"/>
<circle cx="{cx}" cy="{cy}" r="{5*scale}" fill="none" stroke="#dddddd" stroke-width="1"/>

<!-- LiDAR origin and axes -->
<circle cx="{cx}" cy="{cy}" r="6" fill="red"/>
<line x1="{cx}" y1="{cy}" x2="{cx}" y2="{cy - 180}" stroke="red" stroke-width="3" marker-end="url(#arrow)"/>
<text x="{cx + 12}" y="{cy - 190}" font-size="22" fill="red">FORWARD +Y</text>
<text x="{width - 180}" y="{cy - 12}" font-size="22">RIGHT +X</text>
<text x="30" y="{cy - 12}" font-size="22">LEFT -X</text>

<defs>
<marker id="arrow" markerWidth="10" markerHeight="10" refX="5" refY="3" orient="auto" markerUnits="strokeWidth">
<path d="M0,0 L0,6 L6,3 z" fill="red" />
</marker>
</defs>

<g fill="black" opacity="0.70">
{chr(10).join(dots)}
</g>
</svg>
'''

    with open(out_svg, "w") as f:
        f.write(svg)


def print_summary(scan):
    regions = {
        "front_center": [],
        "front_wide": [],
        "left": [],
        "right": [],
        "rear": [],
        "ground_like": [],
        "above_lidar": [],
    }

    for p in scan:
        planar = math.hypot(p.x, p.y)

        if 0.0 < p.y <= 5.0 and abs(p.x) <= 0.5:
            regions["front_center"].append(planar)
        if 0.0 < p.y <= 5.0 and abs(p.x) <= 1.5:
            regions["front_wide"].append(planar)
        if -5.0 <= p.x < -0.5 and abs(p.y) <= 2.0:
            regions["left"].append(abs(p.x))
        if 0.5 < p.x <= 5.0 and abs(p.y) <= 2.0:
            regions["right"].append(abs(p.x))
        if -5.0 <= p.y < 0.0 and abs(p.x) <= 1.5:
            regions["rear"].append(abs(p.y))
        if -0.85 <= p.z <= -0.45 and 0.0 < p.y <= 5.0:
            regions["ground_like"].append(planar)
        if p.z > 0.10 and 0.0 < p.y <= 5.0:
            regions["above_lidar"].append(planar)

    print(f"Total points: {len(scan)}")
    for name, values in regions.items():
        if values:
            print(f"{name:14s}: count={len(values):5d}, nearest={min(values):.2f} m")
        else:
            print(f"{name:14s}: count=    0, nearest=inf")


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-prefix", default="lidar_validation")
    parser.add_argument("--max-range", type=float, default=6.0)
    args = parser.parse_args()

    print("LiDAR scan validation. This does NOT command the robot.")
    print("Waiting for one complete scan...")

    async with LidarDriver() as lidar:
        async for scan in lidar.scan_stream():
            out_csv = Path(f"{args.out_prefix}.csv")
            out_svg = Path(f"{args.out_prefix}_topdown.svg")

            print_summary(scan)
            save_csv(scan, out_csv)
            save_topdown_svg(scan, out_svg, args.max_range)

            print(f"Saved CSV: {out_csv}")
            print(f"Saved top-down SVG: {out_svg}")
            break


if __name__ == "__main__":
    asyncio.run(main())
