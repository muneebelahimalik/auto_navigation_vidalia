#!/usr/bin/env python3

import asyncio
import json
import math
import time
from pathlib import Path

from lidar.lidar_driver import LidarDriver

OUT_HTML = Path("lidar_live_view.html")
OUT_JSON = Path("lidar_live_points.json")


HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>LiDAR Live View</title>
<style>
body { font-family: Arial, sans-serif; margin: 20px; }
canvas { border: 1px solid #aaa; background: white; }
</style>
</head>
<body>
<h2>Live LiDAR Top-Down View</h2>
<p>Red dot = LiDAR. Up = forward +Y. Right = +X. Refreshes from local JSON file.</p>
<canvas id="canvas" width="1000" height="1000"></canvas>

<script>
const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");
const W = canvas.width;
const H = canvas.height;
const cx = W / 2;
const cy = H / 2;
const maxRange = 6.0;
const scale = W / (2 * maxRange);

function drawGrid() {
    ctx.clearRect(0, 0, W, H);

    ctx.strokeStyle = "#ddd";
    ctx.lineWidth = 1;

    ctx.beginPath();
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, H);
    ctx.moveTo(0, cy);
    ctx.lineTo(W, cy);
    ctx.stroke();

    for (let r = 1; r <= 6; r++) {
        ctx.beginPath();
        ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI);
        ctx.stroke();
    }

    ctx.fillStyle = "red";
    ctx.beginPath();
    ctx.arc(cx, cy, 6, 0, 2 * Math.PI);
    ctx.fill();

    ctx.strokeStyle = "red";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx, cy - 140);
    ctx.stroke();

    ctx.fillStyle = "red";
    ctx.font = "18px Arial";
    ctx.fillText("FORWARD +Y", cx + 10, cy - 150);

    ctx.fillStyle = "black";
    ctx.fillText("LEFT -X", 25, cy - 10);
    ctx.fillText("RIGHT +X", W - 130, cy - 10);
}

async function update() {
    try {
        const response = await fetch("lidar_live_points.json?nocache=" + Date.now());
        const data = await response.json();

        drawGrid();

        for (const p of data.points) {
            const x = p[0];
            const y = p[1];
            const z = p[2];

            if (Math.abs(x) > maxRange || Math.abs(y) > maxRange) continue;

            const px = cx + x * scale;
            const py = cy - y * scale;

            if (z < -0.75) {
                ctx.fillStyle = "gray";
            } else if (z < 0.0) {
                ctx.fillStyle = "blue";
            } else {
                ctx.fillStyle = "black";
            }

            ctx.fillRect(px, py, 2, 2);
        }

        ctx.fillStyle = "green";
        ctx.font = "16px Arial";
        ctx.fillText("points: " + data.points.length, 20, 25);
        ctx.fillText("updated: " + data.time, 20, 45);
    } catch (e) {
        drawGrid();
        ctx.fillStyle = "red";
        ctx.font = "18px Arial";
        ctx.fillText("Waiting for lidar_live_points.json...", 20, 30);
    }
}

drawGrid();
setInterval(update, 500);
</script>
</body>
</html>
"""


async def main():
    OUT_HTML.write_text(HTML)
    print("Writing live LiDAR files. This does NOT command the robot.")
    print(f"Open this file in VS Code: {OUT_HTML.resolve()}")
    print("Keep this script running.")

    async with LidarDriver() as lidar:
        async for scan in lidar.scan_stream():
            points = []

            for p in scan:
                if abs(p.x) <= 8.0 and abs(p.y) <= 8.0 and -1.5 <= p.z <= 1.5:
                    points.append([round(p.x, 3), round(p.y, 3), round(p.z, 3)])

            if len(points) > 10000:
                step = max(1, len(points) // 10000)
                points = points[::step]

            OUT_JSON.write_text(json.dumps({
                "time": time.strftime("%H:%M:%S"),
                "points": points,
            }))

            print(f"updated {len(points)} points at {time.strftime('%H:%M:%S')}", flush=True)


if __name__ == "__main__":
    asyncio.run(main())

