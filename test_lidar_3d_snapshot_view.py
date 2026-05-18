#!/usr/bin/env python3

import asyncio
import json
import math
import time
from pathlib import Path

from lidar.lidar_driver import LidarDriver

OUT_HTML = Path("lidar_3d_live_view.html")


def make_html(points, timestamp):
    points_json = json.dumps(points)

    return f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta http-equiv="refresh" content="1">
<title>LiDAR 3D Point Cloud View</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
<style>
body {{
    font-family: Arial, sans-serif;
    margin: 20px;
}}
#plot {{
    width: 100%;
    height: 850px;
}}
</style>
</head>
<body>
<h2>Live LiDAR 3D Point Cloud Snapshot</h2>
<p>
Updated: {timestamp}<br>
Coordinate frame: X = right, Y = forward, Z = up relative to LiDAR.
</p>
<div id="plot"></div>

<script>
const points = {points_json};

const x = points.map(p => p[0]);
const y = points.map(p => p[1]);
const z = points.map(p => p[2]);
const c = points.map(p => p[2]);

const trace = {{
    x: x,
    y: y,
    z: z,
    mode: "markers",
    type: "scatter3d",
    marker: {{
        size: 2,
        color: c,
        colorscale: "Viridis",
        opacity: 0.85
    }},
    name: "VLP-16 points"
}};

const lidarOrigin = {{
    x: [0],
    y: [0],
    z: [0],
    mode: "markers+text",
    type: "scatter3d",
    marker: {{
        size: 8,
        color: "red"
    }},
    text: ["LiDAR"],
    textposition: "top center",
    name: "LiDAR origin"
}};

const forwardLine = {{
    x: [0, 0],
    y: [0, 3],
    z: [0, 0],
    mode: "lines+text",
    type: "scatter3d",
    line: {{
        color: "red",
        width: 8
    }},
    text: ["", "Forward +Y"],
    textposition: "top center",
    name: "Forward +Y"
}};

const rightLine = {{
    x: [0, 2],
    y: [0, 0],
    z: [0, 0],
    mode: "lines+text",
    type: "scatter3d",
    line: {{
        color: "blue",
        width: 6
    }},
    text: ["", "Right +X"],
    textposition: "top center",
    name: "Right +X"
}};

const layout = {{
    scene: {{
        xaxis: {{
            title: "X right of LiDAR (m)",
            range: [-5, 5]
        }},
        yaxis: {{
            title: "Y forward of LiDAR (m)",
            range: [-2, 8]
        }},
        zaxis: {{
            title: "Z up from LiDAR (m)",
            range: [-1.5, 1.5]
        }},
        aspectmode: "manual",
        aspectratio: {{
            x: 1,
            y: 1.2,
            z: 0.5
        }},
        camera: {{
            eye: {{
                x: 1.5,
                y: -2.5,
                z: 1.2
            }}
        }}
    }},
    margin: {{
        l: 0,
        r: 0,
        b: 0,
        t: 20
    }}
}};

Plotly.newPlot("plot", [trace, lidarOrigin, forwardLine, rightLine], layout);
</script>
</body>
</html>
"""


async def main():
    print("3D LiDAR snapshot viewer. This does NOT command the robot.")
    print(f"Open this file in VS Code: {OUT_HTML.resolve()}")
    print("The HTML file refreshes every second.")

    async with LidarDriver() as lidar:
        scan_id = 0

        async for scan in lidar.scan_stream():
            scan_id += 1

            points = []
            for p in scan:
                # Keep useful field of view near the robot.
                if -5.0 <= p.x <= 5.0 and -2.0 <= p.y <= 8.0 and -1.5 <= p.z <= 1.5:
                    points.append([round(p.x, 3), round(p.y, 3), round(p.z, 3)])

            # Downsample for browser performance.
            if len(points) > 12000:
                step = max(1, len(points) // 12000)
                points = points[::step]

            html = make_html(points, time.strftime("%H:%M:%S"))
            OUT_HTML.write_text(html)

            print(f"scan={scan_id:04d} | points={len(points)} | wrote {OUT_HTML}", flush=True)

            await asyncio.sleep(0.2)


if __name__ == "__main__":
    asyncio.run(main())
