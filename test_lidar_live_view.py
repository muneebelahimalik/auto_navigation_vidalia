#!/usr/bin/env python3

import asyncio
import http.server
import json
import math
import socketserver
import threading
import time
from pathlib import Path

from lidar.lidar_driver import LidarDriver

socketserver.TCPServer.allow_reuse_address = True

OUT_JSON = Path("live_lidar_points.json")
OUT_HTML = Path("live_lidar_view.html")


HTML = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Live VLP-16 LiDAR View</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; }
    canvas { border: 1px solid #ccc; background: white; }
    .info { margin-bottom: 10px; }
  </style>
</head>
<body>
  <h2>Live VLP-16 LiDAR Top-Down View</h2>
  <div class="info">
    Red dot = LiDAR. Up = forward +Y. Right = +X.
  </div>
  <canvas id="canvas" width="900" height="900"></canvas>

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
  ctx.strokeStyle = "#e0e0e0";
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
  ctx.arc(cx, cy, 5, 0, 2 * Math.PI);
  ctx.fill();

  ctx.strokeStyle = "red";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx, cy - 120);
  ctx.stroke();

  ctx.fillStyle = "red";
  ctx.font = "18px Arial";
  ctx.fillText("FORWARD +Y", cx + 10, cy - 130);

  ctx.fillStyle = "black";
  ctx.fillText("LEFT -X", 30, cy - 10);
  ctx.fillText("RIGHT +X", W - 130, cy - 10);
}

async function update() {
  try {
    const res = await fetch("live_lidar_points.json?ts=" + Date.now());
    const data = await res.json();

    drawGrid();

    ctx.fillStyle = "black";
    for (const p of data.points) {
      const x = p[0];
      const y = p[1];
      const z = p[2];

      if (Math.abs(x) > maxRange || Math.abs(y) > maxRange) continue;
      if (z < -1.2 || z > 1.5) continue;

      const px = cx + x * scale;
      const py = cy - y * scale;

      ctx.fillRect(px, py, 2, 2);
    }

    ctx.fillStyle = "blue";
    ctx.font = "16px Arial";
    ctx.fillText("points: " + data.points.length, 20, 25);
    ctx.fillText("updated: " + data.time, 20, 45);
  } catch (err) {
    drawGrid();
    ctx.fillStyle = "red";
    ctx.fillText("Waiting for LiDAR data...", 20, 25);
  }
}

drawGrid();
setInterval(update, 300);
</script>
</body>
</html>
"""


def start_server(port):
    handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("", port), handler) as httpd:
        print(f"Open this in your browser through SSH tunnel: http://localhost:{port}/live_lidar_view.html")
        httpd.serve_forever()


async def lidar_writer(max_points):
    print("Starting live LiDAR writer. This does NOT command the robot.")
    async with LidarDriver() as lidar:
        async for scan in lidar.scan_stream():
            pts = []

            for p in scan:
                if abs(p.x) <= 8.0 and abs(p.y) <= 8.0 and -1.5 <= p.z <= 1.5:
                    pts.append([round(p.x, 3), round(p.y, 3), round(p.z, 3)])

            if len(pts) > max_points:
                step = max(1, len(pts) // max_points)
                pts = pts[::step]

            OUT_JSON.write_text(json.dumps({
                "time": time.strftime("%H:%M:%S"),
                "points": pts,
            }))


async def main():
    OUT_HTML.write_text(HTML)

    port = 8020
    thread = threading.Thread(target=start_server, args=(port,), daemon=True)
    thread.start()

    await lidar_writer(max_points=8000)


if __name__ == "__main__":
    asyncio.run(main())
