"""
port_check.py — who else is bound to the VLP-16's UDP port?

The driver binds UDP 2368 with SO_REUSEPORT, so a second LiDAR program can
bind it too — but the kernel steers one sender's whole flow to ONE socket,
normally the older one.  A previous row_follow that is still alive (stopped
with Ctrl+Z, or hung while saving) therefore silently takes every packet and a
new run waits forever (field 2026-10-29).  ``udp_port_holders`` finds those
processes from /proc (no root needed for the user's own processes) so the
entry points can refuse to start and say which PID to stop.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import List, Tuple

LIDAR_UDP_PORT = 2368


def _socket_inodes(port: int, proc: Path) -> set:
    inodes = set()
    for table in ("net/udp", "net/udp6"):
        try:
            lines = (proc / table).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            f = line.split()
            if len(f) < 10:
                continue
            try:
                if int(f[1].rsplit(":", 1)[1], 16) == port:
                    inodes.add(f[9])
            except (IndexError, ValueError):
                continue
    return inodes


def udp_port_holders(port: int = LIDAR_UDP_PORT, proc: str = "/proc",
                     exclude_pid: int = None) -> List[Tuple[int, str]]:
    """(pid, command line) of every visible process with a socket bound to
    ``port``, excluding ``exclude_pid`` (default: this process)."""
    root = Path(proc)
    me = os.getpid() if exclude_pid is None else exclude_pid
    inodes = _socket_inodes(port, root)
    if not inodes:
        return []
    targets = {f"socket:[{i}]" for i in inodes}
    found = []
    for d in root.iterdir():
        if not d.name.isdigit() or int(d.name) == me:
            continue
        try:
            fds = list((d / "fd").iterdir())
        except OSError:                       # another user's process / gone
            continue
        for fd in fds:
            try:
                if os.readlink(fd) in targets:
                    cmd = (d / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                        errors="replace").strip()
                    found.append((int(d.name), cmd or "?"))
                    break
            except OSError:
                continue
    return found


def report_port_holders(port: int = LIDAR_UDP_PORT) -> bool:
    """Print any other process holding the LiDAR port; True if one was found."""
    holders = udp_port_holders(port)
    if not holders:
        return False
    print(f"\n[lidar] UDP {port} is already in use — this run would receive NO LiDAR data:")
    for pid, cmd in holders:
        print(f"    PID {pid}: {cmd[:120]}")
    pids = " ".join(str(p) for p, _ in holders)
    print("  Stop the old run first (Ctrl+C in its terminal; Ctrl+Z only PAUSES it), or:")
    print(f"    kill -INT {pids}; sleep 3; kill -9 {pids} 2>/dev/null")
    return True
