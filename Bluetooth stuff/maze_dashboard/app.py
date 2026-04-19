"""
Maze robot dashboard: FastAPI + WebSocket + wireless UART to the robot.

Transports (pick one):
  • Bluetooth Classic SPP (ZS-040 / HC-05 on macOS): set MAZE_BT_SERIAL to a callout device, e.g.
      export MAZE_BT_SERIAL=/dev/cu.HC-05-DevB
      export MAZE_BT_BAUD=9600
    Pair the module in System Settings → Bluetooth first. Use `ls /dev/cu.*` to find the port name.
    After a successful session, `MAZE_BT_SERIAL=last` reuses `bt_serial_last.json` beside this file.
  • BLE (HM-10 / Nordic UART): leave MAZE_BT_SERIAL unset; uses Bleak as before.

Run from this directory:
  python3 -m venv .venv && source .venv/bin/activate  # or Windows: .venv\\Scripts\\activate
  pip install -r requirements.txt
  uvicorn app:app --host 127.0.0.1 --port 8765

Env (BLE only, when MAZE_BT_SERIAL is not set):
  MAZE_BLE_NAME   substring(s) to match scan name, comma-separated; default matches HMSoft and BT05
  MAZE_BLE_ADDR   optional AA:BB:CC:DD:EE:FF to skip scan entirely (fastest; set once from System Report or ble_discover.py)
  MAZE_BLE_SCAN_TIMEOUT   max seconds to wait for an advertisement (default: 12); scan stops as soon as the robot is seen
  MAZE_BLE_QUICK_ADDR_TIMEOUT  seconds to try ble_last_device.json address before name scan (default: 2.5)
  MAZE_BLE_USE_LAST_ADDR  0 to ignore ble_last_device.json quick path
  MAZE_BLE_UART_UUID   one characteristic for both directions (HM-10 FFE1 style)
  MAZE_BLE_WRITE_UUID / MAZE_BLE_NOTIFY_UUID  explicit pair (Nordic UART style)

  First-time BLE setup:  python ble_discover.py --save   (writes ble_uart_cache.json beside app.py)
  After a successful connect, ble_last_device.json is written so the next run can reconnect faster.
"""
from __future__ import annotations

import asyncio
import json
import os
import re
import uuid
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from fastapi import Body, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from macos_ble import resolve_ble_device_for_address

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakCharacteristicNotFoundError
except ImportError:
    BleakClient = None  # type: ignore
    BleakScanner = None  # type: ignore

    class BleakCharacteristicNotFoundError(Exception):  # type: ignore[misc, no-redef]
        pass


# Written by `python ble_discover.py --save` (optional first-run helper).
BLE_UART_CACHE_PATH = Path(__file__).resolve().parent / "ble_uart_cache.json"
# Written after a successful BLE connection; used to try find_device_by_address before a full name scan.
BLE_LAST_ADDR_PATH = Path(__file__).resolve().parent / "ble_last_device.json"
# Written after a successful Bluetooth Classic serial session (MAZE_BT_SERIAL=last).
BT_SERIAL_LAST_PATH = Path(__file__).resolve().parent / "bt_serial_last.json"
RUN_ARCHIVE_PATH = Path(__file__).resolve().parent / "maze_run_archive.json"
RUN_ARCHIVE_MAX = 100
_archive_lock = asyncio.Lock()

# Fallback pairs tried after cache/env; HM-10 FFE1 before Nordic (Nordic UUIDs can appear stale).
UART_PROFILE_CANDIDATES: list[tuple[str, str]] = [
    ("0000ffe1-0000-1000-8000-00805f9b34fb", "0000ffe1-0000-1000-8000-00805f9b34fb"),
    ("6e400002-b5a3-f393-e0a9-e50e24dcca9e", "6e400003-b5a3-f393-e0a9-e50e24dcca9e"),
]

# Substrings matched when MAZE_BLE_NAME is unset (HMSoft vs BT05/CC41-style clones).
DEFAULT_NAME_SUBSTRINGS: tuple[str, ...] = ("HMSOFT", "BT05")


def _adv_substrings() -> list[str]:
    raw = os.environ.get("MAZE_BLE_NAME", "").strip()
    if raw:
        return [s.strip().upper() for s in raw.split(",") if s.strip()]
    return list(DEFAULT_NAME_SUBSTRINGS)


def _env_float(key: str, default: float) -> float:
    raw = os.environ.get(key, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


# Max time for a scan pass; discovery stops as soon as the device matches (BleakScanner.find_device_by_*).
SCAN_TIMEOUT = _env_float("MAZE_BLE_SCAN_TIMEOUT", 12.0)
QUICK_ADDR_TIMEOUT = _env_float("MAZE_BLE_QUICK_ADDR_TIMEOUT", 2.5)
RECONNECT_DELAY = 2.0


def _load_last_serial_port() -> str | None:
    if not BT_SERIAL_LAST_PATH.is_file():
        return None
    try:
        data = json.loads(BT_SERIAL_LAST_PATH.read_text(encoding="utf-8"))
        p = str(data.get("port", "")).strip()
        if p.startswith("/dev/"):
            return p
    except (OSError, json.JSONDecodeError, TypeError, ValueError):
        pass
    return None


def _save_last_serial_port(port: str) -> None:
    p = port.strip()
    if not p.startswith("/dev/"):
        return
    try:
        BT_SERIAL_LAST_PATH.write_text(
            json.dumps({"port": p}, indent=2) + "\n",
            encoding="utf-8",
        )
    except OSError:
        pass


def _resolve_serial_port() -> str | None:
    raw = (os.environ.get("MAZE_BT_SERIAL") or os.environ.get("MAZE_SERIAL_PORT") or "").strip()
    if not raw:
        return None
    if raw.lower() == "last":
        return _load_last_serial_port()
    return raw


def _wants_serial_transport() -> bool:
    return bool((os.environ.get("MAZE_BT_SERIAL") or os.environ.get("MAZE_SERIAL_PORT") or "").strip())


def _print_dev_server_hints() -> None:
    """Console hints when running uvicorn (dev or production)."""
    print("\n[maze_dashboard] Robot link (see README for full steps):")
    if _wants_serial_transport():
        resolved = _resolve_serial_port()
        port_disp = resolved if resolved else "(resolve MAZE_BT_SERIAL or use last after first good connect)"
        print(f"  Transport: Bluetooth Classic SPP (ZS-040 / HC-05) → {port_disp}")
        print("  macOS: pair the module first; open /dev/cu.* (callout), not /dev/tty.*")
        print("  Env: MAZE_BT_SERIAL, MAZE_BT_BAUD (default 9600); MAZE_BT_SERIAL=last uses bt_serial_last.json")
    else:
        print("  Transport: BLE via Bleak (HM-10 / Nordic UART). Leave MAZE_BT_SERIAL unset.")
        print("  First-time UUIDs: python ble_discover.py --save")
    print("  Tip: same command/telemetry protocol as USB — only the radio path changes.\n")


def _serial_help_hint() -> str:
    try:
        dev = Path("/dev")
        if dev.is_dir():
            cus = sorted(dev.glob("cu.*"))
            if cus:
                tail = ", ".join(str(p) for p in cus[-5:])
                return f"; paired SPP ports often look like {tail}"
    except OSError:
        pass
    return ""


def _char_can_write(char: Any) -> bool:
    props = set(char.properties)
    return "write" in props or "write-without-response" in props


def _char_can_notify(char: Any) -> bool:
    props = set(char.properties)
    return "notify" in props or "indicate" in props


def _cache_uart_pairs(services: Any) -> list[tuple[str, str]]:
    if not BLE_UART_CACHE_PATH.is_file():
        return []
    try:
        data = json.loads(BLE_UART_CACHE_PATH.read_text(encoding="utf-8"))
        w = str(data.get("write_uuid", "")).strip()
        n = str(data.get("notify_uuid", "")).strip()
        if not w or not n:
            return []
        cw = services.get_characteristic(w)
        cn = services.get_characteristic(n) if n.lower() != w.lower() else cw
        if cw and cn and _char_can_write(cw) and _char_can_notify(cn):
            return [(cw.uuid, cn.uuid)]
    except (OSError, json.JSONDecodeError, TypeError, ValueError):
        pass
    return []


def build_uart_candidate_pairs(services: Any) -> list[tuple[str, str]]:
    """
    Ordered (write_uuid, notify_uuid) candidates. Actual subscription is probed with start_notify
    because some stacks list Nordic TX in GATT but subscriptions still fail (HM-10 / clones).
    """
    out: list[tuple[str, str]] = []
    seen: set[tuple[str, str]] = set()

    def push(w_raw: str, n_raw: str) -> None:
        cw = services.get_characteristic(w_raw)
        cn = services.get_characteristic(n_raw) if n_raw.lower() != w_raw.lower() else cw
        if not cw or not cn:
            return
        if not _char_can_write(cw) or not _char_can_notify(cn):
            return
        key = (cw.uuid.lower(), cn.uuid.lower())
        if key in seen:
            return
        seen.add(key)
        out.append((cw.uuid, cn.uuid))

    for w, n in _cache_uart_pairs(services):
        push(w, n)

    single = os.environ.get("MAZE_BLE_UART_UUID", "").strip()
    w_env = os.environ.get("MAZE_BLE_WRITE_UUID", "").strip()
    n_env = os.environ.get("MAZE_BLE_NOTIFY_UUID", "").strip()
    if single:
        w_env = n_env = single
    if w_env and n_env:
        push(w_env, n_env)

    singles: list[Any] = [
        c for c in services.characteristics.values() if _char_can_write(c) and _char_can_notify(c)
    ]
    singles.sort(key=lambda c: (0 if "ffe1" in c.uuid.lower() else 1, c.uuid.lower()))
    for c in singles:
        push(c.uuid, c.uuid)

    for w_u, n_u in UART_PROFILE_CANDIDATES:
        push(w_u, n_u)

    return out


async def select_uart_by_notify_probe(client: Any, on_notify: Any) -> tuple[str, str]:
    pairs = build_uart_candidate_pairs(client.services)
    if not pairs:
        raise RuntimeError(
            "No UART-like characteristics (write + notify). "
            "Run: python ble_discover.py   then   python ble_discover.py --save"
        )
    last: Exception | None = None
    for write_uuid, notify_uuid in pairs:
        try:
            await client.start_notify(notify_uuid, on_notify)
            return (write_uuid, notify_uuid)
        except BleakCharacteristicNotFoundError as e:
            last = e
            continue
    raise RuntimeError(
        f"Could not subscribe on any notify characteristic ({len(pairs)} tried). Last: {last!r}. "
        "Run: python ble_discover.py --save"
    ) from last


def path_to_polyline(path: str) -> list[tuple[float, float]]:
    """Turn L/S/R/B string into grid polyline (turtle, 90° turns). Screen: +x right, +y down; start facing up (-y)."""
    x, y = 0.0, 0.0
    dx, dy = 0.0, -1.0
    pts: list[tuple[float, float]] = [(x, y)]
    for t in path.upper():
        if t not in "LSRB":
            continue
        x += dx
        y += dy
        pts.append((x, y))
        if t == "L":
            dx, dy = dy, -dx
        elif t == "R":
            dx, dy = -dy, dx
        elif t == "B":
            dx, dy = -dx, -dy
    return pts


def path_to_visual_polyline(path: str) -> list[tuple[float, float]]:
    """Map-preview polyline for turn logs: start straight, then keep going after the final logged turn."""
    x, y = 0.0, 0.0
    dx, dy = 0.0, -1.0
    pts: list[tuple[float, float]] = [(x, y)]
    moved = False
    for t in path.upper():
        if t not in "LSRB":
            continue
        x += dx
        y += dy
        pts.append((x, y))
        moved = True
        if t == "L":
            dx, dy = dy, -dx
        elif t == "R":
            dx, dy = -dy, dx
        elif t == "B":
            dx, dy = -dx, -dy
    if moved:
        x += dx
        y += dy
        pts.append((x, y))
    return pts


def shortest_path_polyline(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """
    Collapse an explored walk into the simple start-to-finish route through its visited grid graph.

    This matches the dashboard's physical-optimal preview for the course maze, which is expected to
    behave like a tree/perfect maze once training has explored the dead-end branches.
    """
    if len(pts) < 2:
        return list(pts)

    graph: dict[tuple[float, float], set[tuple[float, float]]] = {}
    for a, b in zip(pts, pts[1:]):
        if a == b:
            continue
        graph.setdefault(a, set()).add(b)
        graph.setdefault(b, set()).add(a)

    start = pts[0]
    finish = pts[-1]
    q: deque[tuple[float, float]] = deque([start])
    prev: dict[tuple[float, float], tuple[float, float] | None] = {start: None}

    while q:
        cur = q.popleft()
        if cur == finish:
            break
        for nxt in sorted(graph.get(cur, ())):
            if nxt in prev:
                continue
            prev[nxt] = cur
            q.append(nxt)

    if finish not in prev:
        return list(pts)

    out: list[tuple[float, float]] = []
    cur: tuple[float, float] | None = finish
    while cur is not None:
        out.append(cur)
        cur = prev[cur]
    out.reverse()
    return out


# working_code.ino prints STATE: <name>; legacy firmware used STATUS:PHASE:detail.
_ROBOT_STATE_TO_PHASE: dict[str, str] = {
    "IDLE_WAIT_CMD": "IDLE",
    "WAIT_COMMAND": "IDLE",
    "WAIT_TRAIN_START": "ARM_TRAIN",
    "ARM_TRAINING": "ARM_TRAIN",
    "TRAINING": "TRAINING",
    "TRAINING_READY": "TRAINING_READY",
    "WAIT_SOLVE_START": "ARM_SOLVE",
    "READY_TO_SOLVE": "TRAINING_READY",
    "ARM_SOLVING": "ARM_SOLVE",
    "SOLVING": "SOLVING",
    "SOLVED": "SOLVED",
    "ERROR_STATE": "ERROR",
    "PAUSED_ARM_TRAINING": "ARM_TRAIN",
    "PAUSED_TRAINING": "TRAINING",
    "PAUSED_ARM_SOLVING": "ARM_SOLVE",
    "PAUSED_SOLVING": "SOLVING",
}


def _value_after_prefix(line: str, *prefixes: str) -> str | None:
    for prefix in prefixes:
        for sep in (":", "="):
            token = f"{prefix}{sep}"
            if line.startswith(token):
                return line[len(token) :].strip()
    return None


def _phase_from_stage(stage: str) -> str | None:
    key = stage.strip().upper()
    if key == "IDLE":
        return "IDLE"
    if key == "EEPROM_PATH_READY":
        return "TRAINING_READY"
    if key == "TRAINING_CHECKPOINT_READY":
        return "TRAINING_CHECKPOINT"
    if key.startswith("PAUSED_"):
        if key == "PAUSED_ARM_TRAINING":
            return "ARM_TRAIN"
        if key == "PAUSED_TRAINING":
            return "TRAINING"
        if key == "PAUSED_ARM_SOLVING":
            return "ARM_SOLVE"
        if key == "PAUSED_SOLVING":
            return "SOLVING"
    if key.startswith("WAITING_FOR_TRAIN_START_BOX"):
        return "ARM_TRAIN"
    if key.startswith("TRAINING_"):
        return "TRAINING" if key != "TRAINING_COMPLETE" else "TRAINING_READY"
    if key == "PATH_LOADED_FROM_EEPROM":
        return "TRAINING_READY"
    if key.startswith("WAITING_FOR_SOLVE_START_BOX"):
        return "ARM_SOLVE"
    if key.startswith("SOLVING_"):
        return "SOLVING" if key != "SOLVING_COMPLETE" else "SOLVED"
    return None


def bounds_of_polylines(*polylines: list[tuple[float, float]]) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for pl in polylines:
        for px, py in pl:
            xs.append(px)
            ys.append(py)
    if not xs:
        return -1.0, -1.0, 1.0, 1.0
    return min(xs), min(ys), max(xs), max(ys)


@dataclass
class DashboardState:
    logs: list[str] = field(default_factory=list)
    max_logs: int = 800
    ble_connected: bool = False
    ble_label: str = "disconnected"
    # "ble" | "serial" — which radio path the backend is using (UI label).
    link_transport: str = "ble"
    path_raw: str = ""
    path_opt: str = ""
    training_turns: list[str] = field(default_factory=list)
    # Chronological L/S/R/B from every junction (intersection + forced); matches physical path.
    training_geo_path: str = ""
    # Same length as training_geo_path: "forced" vs "intersection" for map corner markers.
    training_geo_roles: list[str] = field(default_factory=list)
    solve_geo_path: str = ""
    train_count_intersection: int = 0
    train_count_forced: int = 0
    solve_count_intersection: int = 0
    solve_count_forced: int = 0
    last_node_index: int = -1
    last_solve_step: int = 0
    phase: str = "UNKNOWN"
    last_status_detail: str = ""
    robot_paused: bool = False

    def append_log(self, line: str) -> None:
        line = line.rstrip("\r\n")
        self.logs.append(line)
        if len(self.logs) > self.max_logs:
            self.logs = self.logs[-self.max_logs :]

    def reset_training_progress(self) -> None:
        self.training_turns.clear()
        self.training_geo_path = ""
        self.training_geo_roles.clear()
        self.train_count_intersection = 0
        self.train_count_forced = 0
        self.last_node_index = -1

    def reset_solve_geo(self) -> None:
        self.solve_geo_path = ""
        self.solve_count_intersection = 0
        self.solve_count_forced = 0

    def append_training_geo(self, turn: str, *, role: str) -> None:
        t = turn.upper()
        if len(t) == 1 and t in "LSRB":
            self.training_geo_path += t
            self.training_geo_roles.append(role)

    def append_solve_geo(self, turn: str) -> None:
        t = turn.upper()
        if len(t) == 1 and t in "LSRB":
            self.solve_geo_path += t

    def record_training_turn(self, node_index: int, turn: str) -> None:
        if node_index < 0:
            return
        is_new_node = node_index >= len(self.training_turns)
        if node_index < len(self.training_turns):
            self.training_turns[node_index] = turn
        else:
            self.training_turns.append(turn)
        self.last_node_index = node_index
        if is_new_node:
            self.append_training_geo(turn, role="intersection")
            self.train_count_intersection += 1

    def ingest_line(self, line: str) -> None:
        self.append_log(line)
        u = line.strip()
        raw_path = _value_after_prefix(u, "PATH_RAW", "RAW_PATH")
        opt_path = _value_after_prefix(u, "PATH_OPT", "OPT_PATH", "OPTIMAL_PATH", "FINAL_OPTIMAL_PATH")
        state_name = _value_after_prefix(u, "STATE")
        stage_name = _value_after_prefix(u, "STAGE")
        command_name = _value_after_prefix(u, "CMD")

        if raw_path is not None:
            self.path_raw = raw_path
            self.last_node_index = -1
        elif opt_path is not None:
            self.path_opt = opt_path
        elif u.startswith("RESULT:"):
            self.path_opt = u.split(":", 1)[1].strip()
        elif u.startswith("STATUS:"):
            rest = u[7:]
            parts = rest.split(":", 1)
            self.phase = parts[0].strip()
            self.last_status_detail = parts[1].strip() if len(parts) > 1 else ""
            if self.phase == "ARMED" and self.last_status_detail == "TRAIN":
                self.reset_training_progress()
            if self.phase == "ARMED" and self.last_status_detail == "SOLVE":
                self.last_solve_step = 0
            if self.phase == "SOLVING_NODE":
                m_step = re.match(r"STEP_(\d+)_([LSRB])", self.last_status_detail.upper())
                if m_step:
                    self.last_solve_step = int(m_step.group(1))
        elif state_name is not None:
            key = state_name.upper()
            mapped = _ROBOT_STATE_TO_PHASE.get(key)
            self.phase = mapped if mapped is not None else key
            self.last_status_detail = ""
            self.robot_paused = key.startswith("PAUSED_")
            if self.phase == "ARM_SOLVE" and not self.robot_paused:
                self.last_solve_step = 0
        elif stage_name is not None:
            phase = _phase_from_stage(stage_name)
            if phase is not None:
                self.phase = phase
            self.last_status_detail = stage_name
            self.robot_paused = stage_name.strip().upper().startswith("PAUSED_")
        elif command_name == "TRAIN":
            self.reset_training_progress()
            self.path_raw = ""
            self.path_opt = ""
            self.last_solve_step = 0
            self.reset_solve_geo()
        elif command_name == "RESUME_TRAIN":
            self.last_solve_step = 0
        elif command_name in ("START", "SOLVE"):
            self.last_solve_step = 0
            self.reset_solve_geo()
        elif re.match(r"^(TRAIN|SOLVE):\s*", u, re.IGNORECASE):
            _prefix, detail = u.split(":", 1)
            self.last_status_detail = detail.strip()
        else:
            m_node_legacy = re.match(r"^NODE:(\d+):([LSRB])$", u.upper())
            m_node_firmware = re.match(r"^NODE\s+(\d+)\b.*\bTURN=([LSRB])\b", u.upper())
            m_forced = re.match(r"^FORCED_TURN=([LSRB])", u.upper())
            m_solve_forced = re.match(r"^SOLVE_FORCED_TURN=([LSRB])", u.upper())
            m_solve_node = re.match(
                r"^SOLVE_NODE\s+STEP=(\d+)/(\d+)\b.*\bTURN=([LSRB])\b",
                u.upper(),
            )
            m_solve_step = re.match(r"^SOLVE_NODE\s+STEP=(\d+)/(\d+)\b", u.upper())

            if m_node_legacy:
                self.record_training_turn(int(m_node_legacy.group(1)), m_node_legacy.group(2))
            elif m_node_firmware:
                self.record_training_turn(int(m_node_firmware.group(1)), m_node_firmware.group(2))
            elif m_forced:
                self.append_training_geo(m_forced.group(1), role="forced")
                self.train_count_forced += 1
            elif m_solve_node:
                self.last_solve_step = int(m_solve_node.group(1))
                self.append_solve_geo(m_solve_node.group(3))
                self.solve_count_intersection += 1
            elif m_solve_forced:
                self.append_solve_geo(m_solve_forced.group(1))
                self.solve_count_forced += 1
            elif m_solve_step:
                self.last_solve_step = int(m_solve_step.group(1))

    def vehicle_xy(self) -> tuple[float, float] | None:
        if self.phase in ("TRAINING", "ARM_TRAIN"):
            seq = self.training_geo_path or "".join(self.training_turns)
            if seq:
                pl = path_to_polyline(seq)
                if len(pl) >= 2:
                    return pl[-1]
        if self.phase in ("SOLVING", "ARM_SOLVE", "SOLVING_NODE"):
            if self.solve_geo_path:
                pl = path_to_polyline(self.solve_geo_path)
                if len(pl) >= 2:
                    return pl[-1]
            if not self.path_opt:
                return None
            pl = path_to_polyline(self.path_opt)
            if not pl:
                return None
            i = max(0, min(self.last_solve_step, len(pl) - 1))
            return pl[i]
        return None

    def to_view(self) -> dict[str, Any]:
        # The firmware's RAW_PATH only contains intersection turns; the full physical
        # geometry (forced + intersection) lives in training_geo_path, so prefer it
        # for the visualization when available. After training completes we fall back
        # to the simplified path_raw so old/archived sessions still render something.
        geo_training_seq = self.training_geo_path or "".join(self.training_turns)
        train_vertex_roles: list[str] = []
        if self.training_geo_path:
            train_vertex_roles = list(self.training_geo_roles)
        elif geo_training_seq:
            train_vertex_roles = ["intersection"] * len(geo_training_seq)
        if train_vertex_roles and len(train_vertex_roles) != len(geo_training_seq):
            train_vertex_roles = []
        if geo_training_seq:
            raw_pl = path_to_visual_polyline(geo_training_seq)
            partial_pl: list[tuple[float, float]] = [] if self.path_raw else raw_pl
        elif self.path_raw:
            raw_pl = path_to_visual_polyline(self.path_raw)
            partial_pl = []
        else:
            raw_pl = []
            partial_pl = []
        opt_pl = []
        if self.path_opt:
            if len(raw_pl) >= 2:
                opt_pl = shortest_path_polyline(raw_pl)
            else:
                opt_pl = path_to_visual_polyline(self.path_opt)
        solve_geo_pl = path_to_visual_polyline(self.solve_geo_path) if self.solve_geo_path else []
        if solve_geo_pl:
            green_pl = solve_geo_pl
        elif self.phase in ("TRAINING", "ARM_TRAIN"):
            green_pl = raw_pl or partial_pl or opt_pl
        else:
            green_pl = opt_pl or raw_pl or partial_pl
        mn_x, mn_y, mx_x, mx_y = bounds_of_polylines(raw_pl, opt_pl, partial_pl, solve_geo_pl)
        pad = 2.0
        min_x = mn_x - pad
        min_y = mn_y - pad
        max_x = mx_x + pad
        max_y = mx_y + pad
        w = max(max_x - min_x, 2.0)
        h = max(max_y - min_y, 2.0)
        veh = self.vehicle_xy()
        return {
            "ble_connected": self.ble_connected,
            "ble_label": self.ble_label,
            "link_transport": self.link_transport,
            "logs": self.logs[-400:],
            "phase": self.phase,
            "last_status_detail": self.last_status_detail,
            "robot_paused": self.robot_paused,
            "path_raw": self.path_raw,
            "path_opt": self.path_opt,
            "polyline_raw": raw_pl,
            "polyline_opt": opt_pl,
            "polyline_green": green_pl,
            "polyline_partial_train": partial_pl,
            "polyline_solve_geo": solve_geo_pl,
            "train_vertex_roles": train_vertex_roles,
            "bounds": {"min_x": min_x, "min_y": min_y, "w": w, "h": h},
            "vehicle": {"x": veh[0], "y": veh[1]} if veh else None,
            "last_node_index": self.last_node_index,
            "last_solve_step": self.last_solve_step,
            "turn_stats": {
                "train_intersection": self.train_count_intersection,
                "train_forced": self.train_count_forced,
                "solve_intersection": self.solve_count_intersection,
                "solve_forced": self.solve_count_forced,
            },
        }

    def to_archive_view(self) -> dict[str, Any]:
        """Full dashboard snapshot for persistent run archive (includes all buffered log lines)."""
        v = self.to_view()
        v["logs"] = list(self.logs)
        return v

    def clear_dashboard_session(self) -> None:
        """Reset local log + map state so the UI matches an empty session."""
        self.logs.clear()
        self.path_raw = ""
        self.path_opt = ""
        self.reset_training_progress()
        self.reset_solve_geo()
        self.last_solve_step = 0
        self.phase = "UNKNOWN"
        self.last_status_detail = ""
        self.robot_paused = False


state = DashboardState()
if _wants_serial_transport():
    state.link_transport = "serial"
clients: set[WebSocket] = set()
ble_task: asyncio.Task | None = None
out_queue: asyncio.Queue[str] = asyncio.Queue()
write_queue: asyncio.Queue[bytes] = asyncio.Queue()


def _archive_default() -> dict[str, Any]:
    return {"version": 1, "runs": []}


def _read_archive_file() -> dict[str, Any]:
    if not RUN_ARCHIVE_PATH.is_file():
        return _archive_default()
    try:
        raw = json.loads(RUN_ARCHIVE_PATH.read_text(encoding="utf-8"))
        if not isinstance(raw, dict) or not isinstance(raw.get("runs"), list):
            return _archive_default()
        return raw
    except (OSError, json.JSONDecodeError, TypeError):
        return _archive_default()


def _write_archive_file(data: dict[str, Any]) -> None:
    RUN_ARCHIVE_PATH.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _run_meta(entry: dict[str, Any]) -> dict[str, Any]:
    view = entry.get("view") or {}
    logs = view.get("logs") if isinstance(view.get("logs"), list) else []
    pr = view.get("path_raw") or ""
    po = view.get("path_opt") or ""
    return {
        "id": entry.get("id", ""),
        "created_at": entry.get("created_at", ""),
        "label": entry.get("label") or "",
        "line_count": len(logs),
        "path_raw_len": len(pr),
        "path_opt_len": len(po),
    }


async def broadcast(payload: dict[str, Any]) -> None:
    dead: list[WebSocket] = []
    for ws in clients:
        try:
            await ws.send_json(payload)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)


async def broadcast_state() -> None:
    await broadcast({"type": "state", "data": state.to_view()})


def _load_last_ble_address() -> str | None:
    if not BLE_LAST_ADDR_PATH.is_file():
        return None
    try:
        data = json.loads(BLE_LAST_ADDR_PATH.read_text(encoding="utf-8"))
        a = str(data.get("address", "")).strip()
        if len(a.replace(":", "").replace("-", "")) >= 8:
            compact = a.replace(":", "").replace("-", "")
            return a.replace("-", ":") if len(compact) == 12 else a
    except (OSError, json.JSONDecodeError, TypeError, ValueError):
        pass
    return None


def _save_last_ble_address(address: str) -> None:
    addr = address.strip()
    if len(addr.replace(":", "")) < 8:
        return
    compact = addr.replace(":", "").replace("-", "")
    if len(compact) == 12:
        addr = addr.replace("-", ":").upper()
    try:
        BLE_LAST_ADDR_PATH.write_text(
            json.dumps({"address": addr}, indent=2) + "\n",
            encoding="utf-8",
        )
    except OSError:
        pass


def _adv_matches_robot_name(d: Any, ad: Any) -> bool:
    """HM-10 may expose the name on BLEDevice.name and/or AdvertisementData.local_name (scan response)."""
    subs = _adv_substrings()
    for n in (getattr(ad, "local_name", None), getattr(d, "name", None)):
        if not n:
            continue
        u = str(n).upper()
        if any(s in u for s in subs):
            return True
    return False


async def find_robot_ble_device() -> tuple[Any | None, str, str | None]:
    """
    Prefer last successful address (short timeout), then scan until name matches (stops on first match).
    """
    if BleakScanner is None:
        return None, "", None

    use_last = os.environ.get("MAZE_BLE_USE_LAST_ADDR", "1").strip().lower() not in (
        "0",
        "false",
        "no",
    )
    if use_last:
        last = _load_last_ble_address()
        if last:
            direct = await resolve_ble_device_for_address(last)
            if direct:
                return direct, (direct.name or ""), last
            found = await BleakScanner.find_device_by_address(last, timeout=QUICK_ADDR_TIMEOUT)
            if found:
                return found, (found.name or ""), last

    found = await BleakScanner.find_device_by_filter(_adv_matches_robot_name, timeout=SCAN_TIMEOUT)
    if found:
        return found, (found.name or ""), found.address
    return None, "", None


async def serial_connection_loop() -> None:
    """Bluetooth Classic (HC-05 / ZS-040) via pyserial — pair in macOS Bluetooth settings, then open /dev/cu.*."""
    try:
        import serial as pyserial  # pyserial
    except ImportError:
        state.link_transport = "serial"
        state.ble_label = "pyserial not installed (pip install pyserial)"
        state.ble_connected = False
        await broadcast_state()
        return

    try:
        baud = int(os.environ.get("MAZE_BT_BAUD", "9600") or 9600)
    except ValueError:
        baud = 9600

    while True:
        state.link_transport = "serial"
        state.ble_connected = False
        port = _resolve_serial_port()
        if not port:
            state.ble_label = "set MAZE_BT_SERIAL=/dev/cu.… or MAZE_BT_SERIAL=last"
            await broadcast_state()
            await asyncio.sleep(RECONNECT_DELAY)
            continue

        state.ble_label = f"opening {port}…"
        await broadcast_state()

        if not Path(port).exists():
            state.ble_label = f"missing {port}{_serial_help_hint()}"
            await broadcast_state()
            await asyncio.sleep(RECONNECT_DELAY)
            continue

        try:
            ser = pyserial.Serial(port, baud, timeout=0.3)
        except Exception as e:
            state.append_log(f"[dashboard] serial open error: {e!r}")
            state.ble_label = f"error: {e}"
            await broadcast_state()
            await asyncio.sleep(RECONNECT_DELAY)
            continue

        state.append_log(f"[dashboard] Bluetooth SPP serial {port} @ {baud} baud")
        state.ble_connected = True
        state.ble_label = f"connected {port}"
        _save_last_serial_port(port)
        await broadcast_state()

        async def pump_writes() -> None:
            while ser.is_open:
                payload = await write_queue.get()
                try:
                    await asyncio.to_thread(ser.write, payload)
                    await asyncio.to_thread(ser.flush)
                except Exception:
                    break

        writer = asyncio.create_task(pump_writes())
        try:
            while ser.is_open:
                raw = await asyncio.to_thread(ser.readline)
                if not raw:
                    await asyncio.sleep(0.02)
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                state.ingest_line(line)
                try:
                    out_queue.put_nowait(line)
                except Exception:
                    pass
                await broadcast_state()
        except Exception as e:
            state.append_log(f"[dashboard] serial error: {e!r}")
            state.ble_label = f"error: {e}"
        finally:
            writer.cancel()
            try:
                await writer
            except asyncio.CancelledError:
                pass
            try:
                ser.close()
            except Exception:
                pass

        state.ble_connected = False
        state.ble_label = "disconnected"
        await broadcast_state()
        await asyncio.sleep(RECONNECT_DELAY)


async def link_connection_loop() -> None:
    if _wants_serial_transport():
        await serial_connection_loop()
    else:
        await ble_connection_loop()


async def ble_connection_loop() -> None:
    state.link_transport = "ble"
    if BleakClient is None or BleakScanner is None:
        state.ble_label = "bleak not installed"
        state.ble_connected = False
        await broadcast_state()
        return

    addr_filter = os.environ.get("MAZE_BLE_ADDR", "").strip().upper()

    while True:
        state.link_transport = "ble"
        state.ble_connected = False
        state.ble_label = "scanning…"
        await broadcast_state()

        address: str | None = None
        name_log = ""
        connect_target: Any | None = None
        saved_identifier: str | None = None

        try:
            if addr_filter and len(addr_filter) >= 12:
                address = addr_filter.replace("-", ":")
                connect_target = await resolve_ble_device_for_address(address)
                saved_identifier = address
                if connect_target is not None:
                    name_log = connect_target.name or ""
                state.ble_label = f"connecting {address}"
            else:
                dev, name_log, saved_identifier = await find_robot_ble_device()
                if not dev:
                    state.ble_label = f"no device (match {','.join(_adv_substrings())})"
                    await broadcast_state()
                    await asyncio.sleep(RECONNECT_DELAY)
                    continue
                connect_target = dev
                address = dev.address

            state.ble_label = f"connecting {name_log or address}"
            await broadcast_state()

            buf = bytearray()

            def on_notify(_h: int, data: bytearray) -> None:
                buf.extend(data)
                while b"\n" in buf:
                    i = buf.index(b"\n")
                    line = bytes(buf[:i]).decode("utf-8", errors="replace")
                    del buf[: i + 1]
                    state.ingest_line(line)
                    try:
                        out_queue.put_nowait(line)
                    except Exception:
                        pass

            async with BleakClient(connect_target or address) as client:
                write_uuid, notify_uuid = await select_uart_by_notify_probe(client, on_notify)
                state.append_log(f"[dashboard] BLE UART write={write_uuid} notify={notify_uuid}")

                state.ble_connected = True
                state.ble_label = f"connected {name_log or address}"
                _save_last_ble_address(saved_identifier or address or "")
                await broadcast_state()

                async def pump_writes() -> None:
                    while True:
                        payload = await write_queue.get()
                        try:
                            await client.write_gatt_char(write_uuid, payload, response=False)
                        except Exception:
                            break

                writer = asyncio.create_task(pump_writes())
                try:
                    while client.is_connected:
                        try:
                            await asyncio.wait_for(out_queue.get(), timeout=0.25)
                            await broadcast_state()
                        except asyncio.TimeoutError:
                            await broadcast_state()
                finally:
                    writer.cancel()
                    try:
                        await writer
                    except asyncio.CancelledError:
                        pass

        except Exception as e:
            state.append_log(f"[dashboard] BLE error: {e!r}")
            state.ble_label = f"error: {e}"
        finally:
            state.ble_connected = False
            await broadcast_state()

        await asyncio.sleep(RECONNECT_DELAY)


async def out_queue_drain_to_broadcast() -> None:
    """Drain log lines queued from BLE thread path (if any)."""
    while True:
        line = await out_queue.get()
        await broadcast({"type": "log_line", "line": line})
        await broadcast_state()


INDEX_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Maze robot dashboard</title>
  <link rel="preconnect" href="https://fonts.googleapis.com"/>
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin/>
  <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=Sora:wght@500;600;700&display=swap" rel="stylesheet"/>
  <style>
    :root {
      --bg-deep: #070a0f;
      --bg-panel: rgba(18, 24, 36, 0.72);
      --border: rgba(100, 140, 200, 0.14);
      --border-strong: rgba(120, 170, 255, 0.22);
      --fg: #e8eef7;
      --muted: #8a9bb8;
      --accent: #38bdf8;
      --accent-dim: rgba(56, 189, 248, 0.15);
      --opt: #34d399;
      --opt-glow: rgba(52, 211, 153, 0.35);
      --raw: #94a3b8;
      --veh: #fb923c;
      --err: #f87171;
      --warn: #fbbf24;
      --radius: 14px;
      --shadow: 0 18px 50px rgba(0, 0, 0, 0.45);
    }
    * { box-sizing: border-box; }
    html { height: 100%; }
    body {
      margin: 0;
      min-height: 100%;
      font-family: "Sora", ui-sans-serif, system-ui, sans-serif;
      color: var(--fg);
      background:
        radial-gradient(ellipse 120% 80% at 50% -20%, rgba(56, 189, 248, 0.12), transparent 50%),
        radial-gradient(ellipse 70% 50% at 100% 0%, rgba(52, 211, 153, 0.06), transparent 45%),
        var(--bg-deep);
      background-attachment: fixed;
      padding: clamp(16px, 3vw, 28px);
    }
    .shell {
      max-width: 1180px;
      margin: 0 auto;
    }
    header.masthead {
      display: flex;
      flex-wrap: wrap;
      align-items: flex-end;
      justify-content: space-between;
      gap: 16px 24px;
      margin-bottom: 20px;
    }
    .masthead-right {
      display: flex;
      flex-direction: column;
      align-items: flex-end;
      gap: 10px;
    }
    .masthead-right .status-row { justify-content: flex-end; }
    .brand {
      display: flex;
      flex-direction: column;
      gap: 4px;
    }
    .brand-kicker {
      font-size: 0.7rem;
      font-weight: 600;
      letter-spacing: 0.22em;
      text-transform: uppercase;
      color: var(--muted);
    }
    h1 {
      font-size: clamp(1.35rem, 2.5vw, 1.65rem);
      font-weight: 700;
      margin: 0;
      letter-spacing: -0.02em;
      line-height: 1.2;
    }
    .status-row {
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 10px;
    }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 8px 14px;
      border-radius: 999px;
      font-size: 0.78rem;
      font-weight: 600;
      background: var(--bg-panel);
      border: 1px solid var(--border);
      backdrop-filter: blur(10px);
      max-width: 100%;
    }
    .pill-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      flex-shrink: 0;
      background: var(--muted);
      box-shadow: 0 0 0 2px rgba(255, 255, 255, 0.06);
    }
    .pill-dot.ok { background: var(--opt); box-shadow: 0 0 12px var(--opt-glow); }
    .pill-dot.bad { background: var(--err); box-shadow: 0 0 10px rgba(248, 113, 113, 0.45); }
    .pill-dot.scan { background: var(--warn); animation: pulse 1.4s ease-in-out infinite; }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.45; }
    }
    .pill-label { color: var(--muted); font-weight: 500; }
    .pill-value { color: var(--fg); word-break: break-word; }
    .pill.phase-error { border-color: rgba(248, 113, 113, 0.35); background: rgba(248, 113, 113, 0.08); }
    .pill.phase-error .pill-value { color: #fecaca; }
    .pill.phase-warn { border-color: rgba(251, 191, 36, 0.35); }
    .pill.phase-ok .pill-dot { background: var(--accent); box-shadow: 0 0 12px rgba(56, 189, 248, 0.4); }

    .grid-top {
      display: grid;
      grid-template-columns: minmax(280px, 340px) 1fr;
      gap: 18px;
      align-items: stretch;
    }
    @media (max-width: 860px) {
      .grid-top { grid-template-columns: 1fr; }
    }
    .panel {
      background: var(--bg-panel);
      border-radius: var(--radius);
      border: 1px solid var(--border);
      box-shadow: var(--shadow);
      backdrop-filter: blur(12px);
      padding: clamp(14px, 2.2vw, 20px);
    }
    .panel h2 {
      font-size: 0.65rem;
      font-weight: 600;
      letter-spacing: 0.18em;
      text-transform: uppercase;
      color: var(--muted);
      margin: 0 0 12px;
    }
    .btn-groups { display: flex; flex-direction: column; gap: 14px; }
    .btn-group-label {
      font-size: 0.65rem;
      font-weight: 600;
      letter-spacing: 0.14em;
      text-transform: uppercase;
      color: var(--muted);
      margin-bottom: 8px;
    }
    .btn-row { display: flex; flex-wrap: wrap; gap: 8px; }
    button.cmd {
      font-family: inherit;
      font-size: 0.8rem;
      font-weight: 600;
      letter-spacing: 0.04em;
      padding: 10px 16px;
      border-radius: 10px;
      border: 1px solid transparent;
      cursor: pointer;
      transition: transform 0.12s ease, box-shadow 0.12s ease, filter 0.12s ease, opacity 0.12s;
    }
    button.cmd:disabled {
      opacity: 0.38;
      cursor: not-allowed;
      transform: none;
    }
    button.cmd:not(:disabled):active { transform: scale(0.97); }
    button.cmd.primary {
      color: #041018;
      background: linear-gradient(165deg, #7dd3fc 0%, var(--accent) 45%, #0ea5e9 100%);
      box-shadow: 0 4px 20px rgba(56, 189, 248, 0.35);
    }
    button.cmd.primary:not(:disabled):hover {
      filter: brightness(1.06);
      box-shadow: 0 6px 28px rgba(56, 189, 248, 0.45);
    }
    button.cmd.secondary {
      color: var(--fg);
      background: rgba(30, 41, 59, 0.85);
      border-color: var(--border-strong);
    }
    button.cmd.secondary:not(:disabled):hover {
      background: rgba(51, 65, 85, 0.9);
      border-color: rgba(148, 163, 184, 0.35);
    }

    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 14px 18px;
      margin-top: 16px;
      padding-top: 14px;
      border-top: 1px solid var(--border);
      font-size: 0.72rem;
      color: var(--muted);
    }
    .legend-item { display: inline-flex; align-items: center; gap: 8px; }
    .swatch {
      width: 12px;
      height: 12px;
      border-radius: 3px;
      flex-shrink: 0;
    }
    .swatch.raw { background: var(--raw); }
    .swatch.opt { background: var(--opt); box-shadow: 0 0 10px var(--opt-glow); }
    .swatch.veh { border-radius: 50%; background: var(--veh); box-shadow: 0 0 8px rgba(251, 146, 60, 0.5); }
    .swatch.start { border-radius: 50%; background: #22c55e; box-shadow: 0 0 8px rgba(34, 197, 94, 0.45); }
    .swatch.finish { border-radius: 50%; background: #f43f5e; box-shadow: 0 0 8px rgba(244, 63, 94, 0.4); }

    #map-wrap {
      display: flex;
      flex-direction: column;
      min-height: 0;
    }
    #map-wrap h2 { margin-bottom: 6px; }
    .map-stats {
      font-size: 0.8125rem;
      color: #94a3b8;
      margin: 0 0 12px 0;
      min-height: 1.25em;
    }
    .map-canvas {
      flex: 1;
      display: flex;
      align-items: center;
      justify-content: center;
      min-height: min(52vh, 440px);
      padding: 12px;
      background: rgba(5, 8, 14, 0.65);
      border-radius: 12px;
      border: 1px solid var(--border);
    }
    svg#map {
      display: block;
      width: 100%;
      max-width: 720px;
      height: auto;
      aspect-ratio: 560 / 400;
      border-radius: 10px;
      border: 1px solid rgba(56, 189, 248, 0.12);
      box-shadow: inset 0 0 0 1px rgba(255, 255, 255, 0.03), 0 12px 40px rgba(0, 0, 0, 0.35);
    }

    .log-panel { margin-top: 18px; }
    .log-panel header {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 10px;
    }
    .log-panel h2 { margin: 0; }
    .log-panel-actions { display: flex; flex-wrap: wrap; align-items: center; gap: 10px; flex-shrink: 0; }
    .log-meta { font-size: 0.7rem; color: var(--muted); font-weight: 500; }
    button.log-copy { padding: 6px 12px; font-size: 0.72rem; letter-spacing: 0.06em; }
    #log {
      font-family: "IBM Plex Mono", ui-monospace, monospace;
      font-size: 14px;
      font-weight: 400;
      line-height: 1.6;
      height: min(360px, 42vh);
      min-height: 200px;
      overflow-y: auto;
      padding: 14px 16px;
      border-radius: 12px;
      background: rgba(4, 7, 12, 0.92);
      border: 1px solid var(--border);
      scrollbar-color: rgba(148, 163, 184, 0.35) transparent;
      overscroll-behavior: contain;
      -webkit-overflow-scrolling: touch;
    }
    #log .log-line { display: block; padding: 3px 0; border-bottom: 1px solid rgba(255, 255, 255, 0.04); }
    #log .log-line:last-child { border-bottom: none; }
    #log .log-line.tag-turn {
      margin-top: 12px;
      margin-bottom: 4px;
      padding: 10px 14px 10px 16px;
      border-bottom: none;
      border-left: 4px solid #34d399;
      border-radius: 0 8px 8px 0;
      background: rgba(52, 211, 153, 0.1);
      color: #ecfdf5;
      font-weight: 600;
      letter-spacing: 0.02em;
    }
    #log .log-line.tag-turn:first-child { margin-top: 0; }
    #log .log-line.tag-turn + .log-line.tag-turn { margin-top: 8px; }
    #log .log-line.tag-turn::before {
      content: "Turn · ";
      font-weight: 500;
      color: #6ee7b7;
      opacity: 0.95;
    }
    #log .tag-debug {
      color: #64748b;
      font-size: 0.88em;
      line-height: 1.5;
    }
    #log .tag-warning { color: #fb923c; font-weight: 600; }
    #log .tag-status { color: #93c5fd; }
    #log .tag-path { color: #6ee7b7; }
    #log .tag-node { color: #c4b5fd; }
    #log .tag-result { color: #fcd34d; }
    #log .tag-error { color: #fca5a5; }
    #log .tag-default { color: #cbd5e1; }
    .archive-log .log-line { display: block; padding: 2px 0; }
    .archive-log .log-line.tag-turn {
      margin-top: 10px;
      margin-bottom: 3px;
      padding: 8px 12px 8px 14px;
      border-left: 4px solid #34d399;
      border-radius: 0 8px 8px 0;
      background: rgba(52, 211, 153, 0.1);
      color: #ecfdf5;
      font-weight: 600;
    }
    .archive-log .log-line.tag-turn:first-child { margin-top: 0; }
    .archive-log .log-line.tag-turn + .log-line.tag-turn { margin-top: 6px; }
    .archive-log .log-line.tag-turn::before {
      content: "Turn · ";
      font-weight: 500;
      color: #6ee7b7;
      opacity: 0.95;
    }
    .archive-log .tag-debug { color: #64748b; font-size: 0.88em; }
    .archive-log .tag-warning { color: #fb923c; font-weight: 600; }
    .archive-log .tag-status { color: #93c5fd; }
    .archive-log .tag-path { color: #6ee7b7; }
    .archive-log .tag-node { color: #c4b5fd; }
    .archive-log .tag-result { color: #fcd34d; }
    .archive-log .tag-error { color: #fca5a5; }
    .archive-log .tag-default { color: #cbd5e1; }
    .log-tail-label {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      font-size: 0.72rem;
      font-weight: 600;
      color: var(--muted);
      cursor: pointer;
      user-select: none;
    }
    .log-tail-label input { accent-color: var(--accent); cursor: pointer; }

    .drawer-backdrop {
      position: fixed;
      inset: 0;
      background: rgba(0, 0, 0, 0.5);
      z-index: 80;
      opacity: 0;
      pointer-events: none;
      transition: opacity 0.2s ease;
    }
    .drawer-backdrop.open {
      opacity: 1;
      pointer-events: auto;
    }
    .drawer-panel {
      position: fixed;
      top: 0;
      right: 0;
      height: 100%;
      width: min(440px, 100vw);
      max-width: 100%;
      background: rgba(11, 15, 24, 0.97);
      border-left: 1px solid var(--border-strong);
      box-shadow: -12px 0 48px rgba(0, 0, 0, 0.55);
      z-index: 90;
      transform: translateX(100%);
      transition: transform 0.28s cubic-bezier(0.22, 1, 0.36, 1);
      display: flex;
      flex-direction: column;
      padding: 18px 16px 24px;
      overflow: hidden;
    }
    .drawer-panel.open { transform: translateX(0); }
    .drawer-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 14px;
      flex-shrink: 0;
    }
    .drawer-header h2 {
      margin: 0;
      font-size: 0.75rem;
      font-weight: 600;
      letter-spacing: 0.16em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .drawer-save-row {
      display: flex;
      flex-direction: column;
      gap: 8px;
      margin-bottom: 10px;
      flex-shrink: 0;
    }
    .drawer-save-row input[type="text"] {
      width: 100%;
      padding: 10px 12px;
      border-radius: 10px;
      border: 1px solid var(--border);
      background: rgba(4, 7, 12, 0.92);
      color: var(--fg);
      font-family: inherit;
      font-size: 0.85rem;
    }
    .drawer-hint {
      font-size: 0.68rem;
      color: var(--muted);
      margin: 0 0 12px;
      line-height: 1.45;
      flex-shrink: 0;
    }
    .archive-list {
      list-style: none;
      margin: 0;
      padding: 0;
      overflow-y: auto;
      flex: 1;
      min-height: 0;
      border-top: 1px solid var(--border);
      padding-top: 10px;
    }
    .archive-list li {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      padding: 10px 8px;
      border-radius: 10px;
      cursor: pointer;
      border: 1px solid transparent;
    }
    .archive-list li:hover { background: rgba(56, 189, 248, 0.06); }
    .archive-list li.selected {
      background: rgba(56, 189, 248, 0.1);
      border-color: rgba(56, 189, 248, 0.2);
    }
    .archive-list .meta { font-size: 0.72rem; color: var(--muted); }
    .archive-list .title { font-size: 0.82rem; font-weight: 600; color: var(--fg); }
    .archive-detail {
      flex-shrink: 0;
      max-height: 42vh;
      min-height: 120px;
      overflow-y: auto;
      margin-top: 12px;
      padding-top: 12px;
      border-top: 1px solid var(--border);
    }
    .archive-detail.is-hidden { display: none; }
    .archive-detail .map-canvas {
      min-height: 0;
      padding: 8px;
      margin-bottom: 10px;
    }
    svg#archive-map {
      display: block;
      width: 100%;
      max-width: 100%;
      height: auto;
      aspect-ratio: 560 / 400;
      border-radius: 10px;
      border: 1px solid rgba(56, 189, 248, 0.12);
    }
    .archive-log {
      font-family: "IBM Plex Mono", ui-monospace, monospace;
      font-size: 13px;
      line-height: 1.55;
      max-height: 240px;
      overflow-y: auto;
      padding: 12px 14px;
      border-radius: 10px;
      background: rgba(4, 7, 12, 0.92);
      border: 1px solid var(--border);
      overscroll-behavior: contain;
    }
    .archive-detail-actions { margin-top: 10px; display: flex; gap: 8px; flex-wrap: wrap; }
    .archive-empty { font-size: 0.78rem; color: var(--muted); padding: 12px 4px; }
  </style>
</head>
<body>
  <div class="shell">
    <header class="masthead">
      <div class="brand">
        <span class="brand-kicker">Live link</span>
        <h1>Maze robot · dashboard</h1>
      </div>
      <div class="masthead-right">
        <button type="button" class="cmd secondary" id="archive-toggle-btn">Saved runs</button>
        <div class="status-row" id="status-row" aria-live="polite"></div>
      </div>
    </header>

    <div class="grid-top">
      <div class="panel">
        <h2>Controls</h2>
        <div class="btn-groups">
          <div>
            <div class="btn-group-label">Run</div>
            <div class="btn-row">
              <button type="button" class="cmd primary" data-cmd="TRAIN">Train</button>
              <button type="button" class="cmd secondary" data-cmd="RESUME_TRAIN" id="resume-train-btn"
                title="Continue training after a power loss (checkpoint in EEPROM). Place on start box, then tap.">Resume train</button>
              <button type="button" class="cmd primary" data-cmd="SOLVE">Solve</button>
              <button type="button" class="cmd primary" data-cmd="START">Start</button>
              <button type="button" class="cmd secondary" id="pause-btn" disabled
                title="While training or solving: pause motors and hold state. Click again to resume (PAUSE / RESUME).">Pause</button>
            </div>
          </div>
          <div>
            <div class="btn-group-label">Tools</div>
            <div class="btn-row">
              <button type="button" class="cmd secondary" data-cmd="GETPATH">Get path</button>
              <button type="button" class="cmd secondary" data-cmd="STATUS">Status</button>
              <button type="button" class="cmd secondary" data-cmd="CLEAR"
                title="Clears telemetry log and map on this screen, and sends CLEAR to the robot (clears RAM paths and saved maze data in EEPROM).">Clear all</button>
            </div>
          </div>
        </div>
        <div class="legend" role="list">
          <span class="legend-item" role="listitem"><span class="swatch raw" aria-hidden="true"></span> Raw / explored</span>
          <span class="legend-item" role="listitem"><span class="swatch opt" aria-hidden="true"></span> Optimal</span>
          <span class="legend-item" role="listitem"><span class="swatch start" aria-hidden="true"></span> Start</span>
          <span class="legend-item" role="listitem"><span class="swatch finish" aria-hidden="true"></span> Finish</span>
          <span class="legend-item" role="listitem"><span class="swatch veh" aria-hidden="true"></span> Vehicle</span>
        </div>
      </div>
      <div class="panel" id="map-wrap">
        <h2>Path preview</h2>
        <p class="map-stats" id="map-stats" aria-live="polite"></p>
        <div class="map-canvas">
          <svg id="map" width="560" height="400" viewBox="0 0 560 400" role="img" aria-label="Maze path visualization"></svg>
        </div>
      </div>
    </div>

    <div class="panel log-panel">
      <header>
        <h2>Telemetry</h2>
        <div class="log-panel-actions">
          <label class="log-tail-label" title="When off, new telemetry will not jump the console to the bottom — scroll freely while the robot runs">
            <input type="checkbox" id="log-tail" checked /> Live tail
          </label>
          <span class="log-meta" id="log-meta"></span>
          <button type="button" class="cmd secondary log-copy" id="log-copy-btn"
            title="Copy all log lines to the clipboard">Copy</button>
        </div>
      </header>
      <div id="log"></div>
    </div>
  </div>

  <div id="drawer-backdrop" class="drawer-backdrop" aria-hidden="true"></div>
  <aside id="drawer" class="drawer-panel" aria-hidden="true" aria-label="Saved runs archive">
    <div class="drawer-header">
      <h2>Saved runs</h2>
      <button type="button" class="cmd secondary" id="drawer-close">Close</button>
    </div>
    <div class="drawer-save-row">
      <input type="text" id="save-run-label" placeholder="Label (optional)" maxlength="200"/>
      <button type="button" class="cmd primary" id="save-run-btn">Save current session</button>
    </div>
    <p class="drawer-hint">Snapshots are stored in <code>maze_run_archive.json</code> next to the server. They persist across page reloads and backend restarts (up to """ + str(RUN_ARCHIVE_MAX) + r""" entries).</p>
    <ul id="archive-list" class="archive-list" role="list"></ul>
    <div id="archive-detail" class="archive-detail is-hidden">
      <div class="map-canvas">
        <svg id="archive-map" width="560" height="400" viewBox="0 0 560 400" role="img" aria-label="Saved run path"></svg>
      </div>
      <div id="archive-log" class="archive-log" aria-label="Saved run log"></div>
      <div class="archive-detail-actions">
        <button type="button" class="cmd secondary" id="archive-copy-log">Copy log</button>
        <button type="button" class="cmd secondary" id="archive-delete-btn">Delete this run</button>
      </div>
    </div>
  </aside>
<script>
(function () {
  const logEl = document.getElementById("log");
  const statusRow = document.getElementById("status-row");
  const logMeta = document.getElementById("log-meta");
  const logCopyBtn = document.getElementById("log-copy-btn");
  const logTailEl = document.getElementById("log-tail");
  const drawer = document.getElementById("drawer");
  const drawerBackdrop = document.getElementById("drawer-backdrop");
  const archiveToggleBtn = document.getElementById("archive-toggle-btn");
  const drawerClose = document.getElementById("drawer-close");
  const archiveListEl = document.getElementById("archive-list");
  const archiveDetail = document.getElementById("archive-detail");
  const archiveMapSvg = document.getElementById("archive-map");
  const archiveLogEl = document.getElementById("archive-log");
  const saveRunBtn = document.getElementById("save-run-btn");
  const saveRunLabel = document.getElementById("save-run-label");
  const archiveDeleteBtn = document.getElementById("archive-delete-btn");
  const archiveCopyLogBtn = document.getElementById("archive-copy-log");
  const pauseBtn = document.getElementById("pause-btn");
  let lastDashboardState = { ble_connected: false, robot_paused: false, phase: "", link_transport: "ble" };
  let lastLogLines = [];
  let selectedArchiveId = null;
  let archiveDetailLines = [];
  const svg = document.getElementById("map");
  const mapStatsEl = document.getElementById("map-stats");
  const W = 560, H = 400;
  let wsRef = null;
  let wsHadOpen = false;

  function escapeText(s) {
    return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
  }

  function setLogCopyEnabled(on) {
    logCopyBtn.disabled = !on;
  }

  async function copyLogToClipboard() {
    const text = lastLogLines.join("\n");
    if (!text) return;
    try {
      await navigator.clipboard.writeText(text);
    } catch (err) {
      const ta = document.createElement("textarea");
      ta.value = text;
      ta.setAttribute("readonly", "");
      ta.style.position = "fixed";
      ta.style.left = "-9999px";
      document.body.appendChild(ta);
      ta.select();
      document.execCommand("copy");
      document.body.removeChild(ta);
    }
    const prev = logCopyBtn.textContent;
    logCopyBtn.textContent = "Copied";
    setTimeout(function () { logCopyBtn.textContent = prev; }, 1200);
  }

  function logLineClass(line) {
    const t = line.trim();
    const u = t.toUpperCase();
    if (/^NODE_DEBUG\b/i.test(t)) return "tag-debug";
    if (/^WARNING_/i.test(t)) return "tag-warning";
    if (/^FORCED_TURN=|^SOLVE_FORCED_TURN=|^SOLVE_NODE\s+STEP=/i.test(t)) return "tag-turn";
    if (u.startsWith("STATUS:") || u.startsWith("STATE:") ||
        u.startsWith("STATE=") || u.startsWith("STAGE=") || u.startsWith("CMD=")) {
      if (u.indexOf("ERROR") !== -1) return "tag-error";
      return "tag-status";
    }
    if (u.startsWith("PATH_RAW:") || u.startsWith("PATH_OPT:") ||
        u.startsWith("RAW_PATH:") || u.startsWith("OPT_PATH:") ||
        u.startsWith("RAW_PATH=") || u.startsWith("OPTIMAL_PATH=") ||
        u.startsWith("FINAL_OPTIMAL_PATH=")) return "tag-path";
    if (/^NODE:\d+:[LSRB]$/i.test(t) ||
        /^NODE\s+\d+\b.*\bTURN=[LSRB]\b/i.test(t)) return "tag-node";
    if (u.startsWith("RESULT:")) return "tag-result";
    return "tag-default";
  }

  function logLineHtml(line) {
    const cls = logLineClass(line);
    return "<span class=\"log-line " + cls + "\">" + escapeText(line) + "</span>";
  }

  function renderStatusPills(data, wsOpen) {
    const ble = data.ble_connected;
    const label = (data.ble_label || "").replace(/</g, "");
    const linkKind = (data.link_transport || "ble").toLowerCase();
    const linkPill = linkKind === "serial" ? "Serial" : "BLE";

    let dotClass = "bad";
    if (ble) dotClass = "ok";
    else if (/scanning|connecting|opening/i.test(label)) dotClass = "scan";

    const linkTitle = wsOpen
      ? (linkKind === "serial" ? "Bluetooth classic (SPP serial)" : "Bluetooth Low Energy")
      : "WebSocket disconnected";
    const rows = [
      "<div class=\"pill\" title=\"" + escapeText(linkTitle) + "\">",
      "<span class=\"pill-dot " + dotClass + "\"></span>",
      "<span class=\"pill-label\">" + escapeText(linkPill) + "</span>",
      "<span class=\"pill-value\">" + escapeText(label) + "</span>",
      "</div>"
    ];
    if (wsOpen === false && wsHadOpen) {
      rows.push("<div class=\"pill phase-warn\"><span class=\"pill-label\">Link</span><span class=\"pill-value\">Reconnecting…</span></div>");
    }
    statusRow.innerHTML = rows.join("");
  }

  function connectWs() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(proto + "://" + location.host + "/ws");
    wsRef = ws;
    const buttons = document.querySelectorAll("button.cmd[data-cmd]");

    function setButtonsEnabled(on) {
      buttons.forEach((btn) => { btn.disabled = !on; });
      if (pauseBtn) pauseBtn.disabled = !on;
    }

    ws.onopen = () => {
      wsHadOpen = true;
      setButtonsEnabled(false);
      renderStatusPills({ ble_connected: false, ble_label: "waiting…", link_transport: lastDashboardState.link_transport || "ble" }, true);
    };
    ws.onclose = () => {
      setButtonsEnabled(false);
      renderStatusPills({ ble_connected: false, ble_label: "WS closed" }, false);
      setTimeout(connectWs, 1500);
    };
    ws.onmessage = (ev) => {
      const msg = JSON.parse(ev.data);
      if (msg.type === "state") {
      if (msg.data && msg.data.link_transport) {
        lastDashboardState.link_transport = msg.data.link_transport;
      }
      render(msg.data);
    }
    };
    document.querySelectorAll("button.cmd[data-cmd]").forEach((btn) => {
      btn.onclick = () => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({ type: "cmd", "cmd": btn.getAttribute("data-cmd") }));
        }
      };
    });
    if (pauseBtn) {
      pauseBtn.onclick = () => {
        if (ws.readyState !== WebSocket.OPEN) return;
        const cmd = lastDashboardState.robot_paused ? "RESUME" : "PAUSE";
        ws.send(JSON.stringify({ type: "cmd", "cmd": cmd }));
      };
    }
    setButtonsEnabled(false);
  }

  function phaseAllowsPause(phase) {
    const p = (phase || "").toUpperCase();
    return p === "TRAINING" || p === "ARM_TRAIN" || p === "SOLVING" || p === "ARM_SOLVE";
  }

  function updateCommandButtons(data) {
    const wsOpen = wsRef && wsRef.readyState === WebSocket.OPEN;
    const bleOk = wsOpen && data.ble_connected;
    const phase = (data.phase || "").toUpperCase();
    document.querySelectorAll("button.cmd[data-cmd]").forEach((btn) => {
      const cmd = btn.getAttribute("data-cmd");
      if (cmd === "RESUME_TRAIN") {
        btn.disabled = !bleOk || phase !== "TRAINING_CHECKPOINT";
      } else {
        btn.disabled = !bleOk;
      }
    });
    if (pauseBtn) {
      const canPauseOrResume = data.robot_paused || phaseAllowsPause(data.phase);
      pauseBtn.disabled = !bleOk || !canPauseOrResume;
      pauseBtn.textContent = data.robot_paused ? "Resume" : "Pause";
    }
  }

  function toSvg(b, x, y) {
    const px = (x - b.min_x) / b.w * (W - 24) + 12;
    const py = (y - b.min_y) / b.h * (H - 24) + 12;
    return [px, py];
  }

  function polyD(b, pts) {
    if (!pts.length) return "";
    let d = "";
    for (let i = 0; i < pts.length; i++) {
      const [px, py] = toSvg(b, pts[i][0], pts[i][1]);
      d += (i === 0 ? "M" : "L") + px.toFixed(2) + " " + py.toFixed(2) + " ";
    }
    return d.trim();
  }

  function samePts(a, b) {
    if (!Array.isArray(a) || !Array.isArray(b) || a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) {
      if (!Array.isArray(a[i]) || !Array.isArray(b[i])) return false;
      if (a[i][0] !== b[i][0] || a[i][1] !== b[i][1]) return false;
    }
    return true;
  }

  function addEndpointBubblePx(svgEl, px, py, kind) {
    const g = document.createElementNS("http://www.w3.org/2000/svg", "g");
    g.setAttribute("class", "map-endpoint");
    const tip = document.createElementNS("http://www.w3.org/2000/svg", "title");
    tip.textContent = kind === "start" ? "Start" : "Finish";
    g.appendChild(tip);
    const halo = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    halo.setAttribute("cx", String(px));
    halo.setAttribute("cy", String(py));
    halo.setAttribute("r", "14");
    halo.setAttribute("fill", kind === "start" ? "rgba(34, 197, 94, 0.28)" : "rgba(244, 63, 94, 0.25)");
    halo.setAttribute("pointer-events", "none");
    g.appendChild(halo);
    const c = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    c.setAttribute("cx", String(px));
    c.setAttribute("cy", String(py));
    c.setAttribute("r", "11");
    c.setAttribute("fill", kind === "start" ? "#22c55e" : "#f43f5e");
    c.setAttribute("stroke", "#f8fafc");
    c.setAttribute("stroke-width", "2.5");
    g.appendChild(c);
    const txt = document.createElementNS("http://www.w3.org/2000/svg", "text");
    txt.setAttribute("x", String(px));
    txt.setAttribute("y", String(py));
    txt.setAttribute("text-anchor", "middle");
    txt.setAttribute("dominant-baseline", "central");
    txt.setAttribute("fill", "#f8fafc");
    txt.setAttribute("font-size", "11");
    txt.setAttribute("font-weight", "700");
    txt.setAttribute("font-family", "ui-sans-serif, system-ui, sans-serif");
    txt.setAttribute("pointer-events", "none");
    txt.textContent = kind === "start" ? "S" : "F";
    g.appendChild(txt);
    svgEl.appendChild(g);
  }

  function addTurnKindMarkers(svgEl, b, rawPts, roles) {
    if (!roles || !roles.length || !rawPts || rawPts.length < 2) return;
    for (let i = 0; i < roles.length; i++) {
      const pi = i + 1;
      if (pi >= rawPts.length) break;
      const role = roles[i];
      const forced = role === "forced";
      const [px, py] = toSvg(b, rawPts[pi][0], rawPts[pi][1]);
      const g = document.createElementNS("http://www.w3.org/2000/svg", "g");
      g.setAttribute("class", "map-turn-kind");
      const tip = document.createElementNS("http://www.w3.org/2000/svg", "title");
      tip.textContent = forced ? "Hard turn (forced segment)" : "Intersection decision";
      g.appendChild(tip);
      const c = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      c.setAttribute("cx", String(px));
      c.setAttribute("cy", String(py));
      c.setAttribute("r", forced ? "5" : "4");
      c.setAttribute("fill", forced ? "#f59e0b" : "#38bdf8");
      c.setAttribute("stroke", "#0f172a");
      c.setAttribute("stroke-width", "1.5");
      g.appendChild(c);
      svgEl.appendChild(g);
    }
  }

  function addStartFinishMarkers(svgEl, b, primaryPts) {
    if (!primaryPts || primaryPts.length < 2) return;
    const startPt = primaryPts[0];
    const endPt = primaryPts[primaryPts.length - 1];
    const sp = toSvg(b, startPt[0], startPt[1]);
    const ep = toSvg(b, endPt[0], endPt[1]);
    let sx = sp[0];
    let sy = sp[1];
    let ex = ep[0];
    let ey = ep[1];
    if (Math.hypot(sx - ex, sy - ey) < 20) {
      sx -= 12;
      ex += 12;
    }
    addEndpointBubblePx(svgEl, sx, sy, "start");
    addEndpointBubblePx(svgEl, ex, ey, "finish");
  }

  function addMapGrid(svgEl, idSuffix) {
    const gid = "grid-" + idSuffix;
    const vid = "vgrad-" + idSuffix;
    const defs = document.createElementNS("http://www.w3.org/2000/svg", "defs");
    const pattern = document.createElementNS("http://www.w3.org/2000/svg", "pattern");
    pattern.setAttribute("id", gid);
    pattern.setAttribute("width", "28");
    pattern.setAttribute("height", "28");
    pattern.setAttribute("patternUnits", "userSpaceOnUse");
    const ppath = document.createElementNS("http://www.w3.org/2000/svg", "path");
    ppath.setAttribute("d", "M 28 0 L 0 0 0 28");
    ppath.setAttribute("fill", "none");
    ppath.setAttribute("stroke", "rgba(148, 163, 184, 0.12)");
    ppath.setAttribute("stroke-width", "1");
    pattern.appendChild(ppath);
    defs.appendChild(pattern);
    const vgrad = document.createElementNS("http://www.w3.org/2000/svg", "linearGradient");
    vgrad.setAttribute("id", vid);
    vgrad.setAttribute("x1", "0");
    vgrad.setAttribute("y1", "0");
    vgrad.setAttribute("x2", "0");
    vgrad.setAttribute("y2", "1");
    const s1 = document.createElementNS("http://www.w3.org/2000/svg", "stop");
    s1.setAttribute("offset", "0%");
    s1.setAttribute("stop-color", "#0a0f18");
    s1.setAttribute("stop-opacity", "0.25");
    const s2 = document.createElementNS("http://www.w3.org/2000/svg", "stop");
    s2.setAttribute("offset", "100%");
    s2.setAttribute("stop-color", "#030508");
    s2.setAttribute("stop-opacity", "0.85");
    vgrad.appendChild(s1);
    vgrad.appendChild(s2);
    defs.appendChild(vgrad);
    svgEl.appendChild(defs);
    const bg = document.createElementNS("http://www.w3.org/2000/svg", "rect");
    bg.setAttribute("x", "0");
    bg.setAttribute("y", "0");
    bg.setAttribute("width", String(W));
    bg.setAttribute("height", String(H));
    bg.setAttribute("fill", "url(#" + gid + ")");
    svgEl.appendChild(bg);
    const vignette = document.createElementNS("http://www.w3.org/2000/svg", "rect");
    vignette.setAttribute("x", "0");
    vignette.setAttribute("y", "0");
    vignette.setAttribute("width", String(W));
    vignette.setAttribute("height", String(H));
    vignette.setAttribute("fill", "url(#" + vid + ")");
    vignette.setAttribute("pointer-events", "none");
    svgEl.appendChild(vignette);
  }

  function appendGreenPath(svgEl, b, pts) {
    const p = document.createElementNS("http://www.w3.org/2000/svg", "path");
    p.setAttribute("d", polyD(b, pts));
    p.setAttribute("fill", "none");
    p.setAttribute("stroke", "#059669");
    p.setAttribute("stroke-width", "10");
    p.setAttribute("stroke-linecap", "round");
    p.setAttribute("stroke-linejoin", "round");
    p.setAttribute("opacity", "0.45");
    svgEl.appendChild(p);
    const p1 = document.createElementNS("http://www.w3.org/2000/svg", "path");
    p1.setAttribute("d", polyD(b, pts));
    p1.setAttribute("fill", "none");
    p1.setAttribute("stroke", "#34d399");
    p1.setAttribute("stroke-width", "4");
    p1.setAttribute("stroke-linecap", "round");
    p1.setAttribute("stroke-linejoin", "round");
    svgEl.appendChild(p1);
    const p2 = document.createElementNS("http://www.w3.org/2000/svg", "path");
    p2.setAttribute("d", polyD(b, pts));
    p2.setAttribute("fill", "none");
    p2.setAttribute("stroke", "#6ee7b7");
    p2.setAttribute("stroke-width", "2");
    p2.setAttribute("stroke-linecap", "round");
    svgEl.appendChild(p2);
  }

  function renderMapSvg(svgEl, data, idSuffix) {
    const b = data.bounds;
    if (!b) return;
    while (svgEl.firstChild) svgEl.removeChild(svgEl.firstChild);
    addMapGrid(svgEl, idSuffix);
    const rawPts = data.polyline_raw || [];
    const optPts = data.polyline_opt || [];
    const greenPts = data.polyline_green || [];
    const partPts = data.polyline_partial_train || [];
    const solveGeoPts = data.polyline_solve_geo || [];
    const phaseU = (data.phase || "").toUpperCase();
    const solvingPhase = phaseU === "SOLVING" || phaseU === "SOLVING_NODE" || phaseU === "ARM_SOLVE";
    const showSolveGeo =
      solveGeoPts.length >= 2 &&
      (solvingPhase || phaseU === "SOLVED" || idSuffix === "arch");

    if (rawPts.length >= 2) {
      const p = document.createElementNS("http://www.w3.org/2000/svg", "path");
      p.setAttribute("d", polyD(b, rawPts));
      p.setAttribute("fill", "none");
      p.setAttribute("stroke", "#64748b");
      p.setAttribute("stroke-width", "3");
      p.setAttribute("stroke-linecap", "round");
      p.setAttribute("stroke-linejoin", "round");
      const tip = document.createElementNS("http://www.w3.org/2000/svg", "title");
      tip.textContent = "Physical route (all segments including hard turns)";
      p.appendChild(tip);
      svgEl.appendChild(p);
    }
    if (partPts.length >= 2 && (!rawPts.length || data.phase === "TRAINING" || data.phase === "ARM_TRAIN")) {
      const p = document.createElementNS("http://www.w3.org/2000/svg", "path");
      p.setAttribute("d", polyD(b, partPts));
      p.setAttribute("fill", "none");
      p.setAttribute("stroke", "#94a3b8");
      p.setAttribute("stroke-width", "2");
      p.setAttribute("stroke-dasharray", "6 4");
      svgEl.appendChild(p);
    }
    const showOptRef = optPts.length >= 2 && !samePts(optPts, greenPts) && (showSolveGeo || rawPts.length >= 2);
    if (showOptRef) {
      const ref = document.createElementNS("http://www.w3.org/2000/svg", "path");
      ref.setAttribute("d", polyD(b, optPts));
      ref.setAttribute("fill", "none");
      ref.setAttribute("stroke", "#475569");
      ref.setAttribute("stroke-width", "2");
      ref.setAttribute("stroke-dasharray", "5 5");
      ref.setAttribute("stroke-linecap", "round");
      ref.setAttribute("stroke-linejoin", "round");
      ref.setAttribute("opacity", "0.55");
      const tip = document.createElementNS("http://www.w3.org/2000/svg", "title");
      tip.textContent = "Optimal route preview";
      ref.appendChild(tip);
      svgEl.appendChild(ref);
    }
    if (greenPts.length >= 2) {
      appendGreenPath(svgEl, b, greenPts);
    }
    if (!showSolveGeo && Array.isArray(data.train_vertex_roles) && data.train_vertex_roles.length) {
      addTurnKindMarkers(svgEl, b, rawPts, data.train_vertex_roles);
    }
    let markerPts = greenPts.length >= 2 ? greenPts : null;
    if (!markerPts && rawPts.length >= 2) markerPts = rawPts;
    if (!markerPts && optPts.length >= 2) markerPts = optPts;
    if (!markerPts && partPts.length >= 2) markerPts = partPts;
    addStartFinishMarkers(svgEl, b, markerPts);
    if (data.vehicle) {
      const [cx, cy] = toSvg(b, data.vehicle.x, data.vehicle.y);
      const glow = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      glow.setAttribute("cx", cx);
      glow.setAttribute("cy", cy);
      glow.setAttribute("r", "16");
      glow.setAttribute("fill", "rgba(251, 146, 60, 0.28)");
      svgEl.appendChild(glow);
      const c = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      c.setAttribute("cx", cx);
      c.setAttribute("cy", cy);
      c.setAttribute("r", "8");
      c.setAttribute("fill", "#fb923c");
      c.setAttribute("stroke", "#fff7ed");
      c.setAttribute("stroke-width", "2");
      svgEl.appendChild(c);
    }
  }

  function renderTelemetryLog(lines) {
    const tailOn = logTailEl && logTailEl.checked;
    const threshold = 80;
    const fromBottom = logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight;
    const nearBottom = fromBottom < threshold;
    lastLogLines = lines.slice();
    logEl.innerHTML = lines.map(logLineHtml).join("");
    if (tailOn && nearBottom) {
      logEl.scrollTop = logEl.scrollHeight;
    } else {
      logEl.scrollTop = Math.max(0, logEl.scrollHeight - logEl.clientHeight - fromBottom);
    }
    logMeta.textContent = lines.length ? lines.length + " lines" : "Empty";
    setLogCopyEnabled(lines.length > 0);
  }

  function formatTurnStats(data) {
    const ts = data.turn_stats || {};
    const phaseU = (data.phase || "").toUpperCase();
    const ti = ts.train_intersection ?? 0;
    const tf = ts.train_forced ?? 0;
    const si = ts.solve_intersection ?? 0;
    const sf = ts.solve_forced ?? 0;
    if (phaseU === "TRAINING" || phaseU === "ARM_TRAIN") {
      if (ti + tf === 0) return "";
      return "Intersections " + ti + " · Hard turns " + tf;
    }
    if (phaseU === "SOLVING" || phaseU === "SOLVING_NODE" || phaseU === "ARM_SOLVE" || phaseU === "SOLVED") {
      if (si + sf === 0) return "";
      return "Intersections " + si + " · Hard turns " + sf;
    }
    return "";
  }

  function render(data) {
    lastDashboardState = data;
    const wsOpen = wsRef && wsRef.readyState === WebSocket.OPEN;
    renderStatusPills(data, wsOpen);
    updateCommandButtons(data);
    if (Array.isArray(data.logs)) {
      renderTelemetryLog(data.logs);
    }
    if (mapStatsEl) mapStatsEl.textContent = formatTurnStats(data);
    renderMapSvg(svg, data, "live");
  }

  function setDrawerOpen(on) {
    drawer.classList.toggle("open", on);
    drawerBackdrop.classList.toggle("open", on);
    drawer.setAttribute("aria-hidden", on ? "false" : "true");
    drawerBackdrop.setAttribute("aria-hidden", on ? "false" : "true");
    if (on) refreshArchiveList();
  }

  async function refreshArchiveList() {
    archiveDetail.classList.add("is-hidden");
    selectedArchiveId = null;
    try {
      const r = await fetch("/api/runs");
      const rows = await r.json();
      archiveListEl.innerHTML = "";
      if (!rows.length) {
        const li = document.createElement("li");
        li.className = "archive-empty";
        li.setAttribute("role", "listitem");
        li.textContent = "No saved runs yet. Save the current session above.";
        archiveListEl.appendChild(li);
        return;
      }
      rows.forEach(function (row) {
        const li = document.createElement("li");
        li.dataset.id = row.id;
        li.setAttribute("role", "listitem");
        const dt = row.created_at ? new Date(row.created_at).toLocaleString() : "";
        const lab = row.label || "Untitled";
        li.innerHTML = "<div><div class=\"title\">" + escapeText(lab) + "</div><div class=\"meta\">" + escapeText(dt) + " · " + row.line_count + " lines</div></div>";
        archiveListEl.appendChild(li);
      });
    } catch (e) {
      archiveListEl.innerHTML = "";
      const li = document.createElement("li");
      li.className = "archive-empty";
      li.setAttribute("role", "listitem");
      li.textContent = "Could not load archive.";
      archiveListEl.appendChild(li);
    }
  }

  archiveListEl.addEventListener("click", function (ev) {
    const li = ev.target.closest("li");
    if (!li || !li.dataset.id) return;
    selectedArchiveId = li.dataset.id;
    archiveListEl.querySelectorAll("li[data-id]").forEach(function (el) {
      el.classList.toggle("selected", el.dataset.id === selectedArchiveId);
    });
    loadArchiveDetail(selectedArchiveId);
  });

  async function loadArchiveDetail(id) {
    try {
      const r = await fetch("/api/runs/" + encodeURIComponent(id));
      if (!r.ok) throw new Error("missing");
      const j = await r.json();
      const view = j.entry.view;
      archiveDetailLines = Array.isArray(view.logs) ? view.logs.slice() : [];
      archiveDetail.classList.remove("is-hidden");
      renderMapSvg(archiveMapSvg, view, "arch");
      archiveLogEl.innerHTML = archiveDetailLines.map(logLineHtml).join("");
      archiveLogEl.scrollTop = 0;
    } catch (e) {
      archiveDetail.classList.add("is-hidden");
    }
  }

  archiveToggleBtn.onclick = function () { setDrawerOpen(!drawer.classList.contains("open")); };
  drawerClose.onclick = function () { setDrawerOpen(false); };
  drawerBackdrop.onclick = function () { setDrawerOpen(false); };

  saveRunBtn.onclick = async function () {
    const label = saveRunLabel.value || "";
    try {
      await fetch("/api/runs", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ label: label })
      });
      saveRunLabel.value = "";
      await refreshArchiveList();
    } catch (e) {}
  };

  archiveDeleteBtn.onclick = async function () {
    if (!selectedArchiveId) return;
    if (!confirm("Delete this saved run?")) return;
    try {
      await fetch("/api/runs/" + encodeURIComponent(selectedArchiveId), { method: "DELETE" });
      selectedArchiveId = null;
      archiveDetail.classList.add("is-hidden");
      await refreshArchiveList();
    } catch (e) {}
  };

  archiveCopyLogBtn.onclick = async function () {
    const text = archiveDetailLines.join("\n");
    if (!text) return;
    try {
      await navigator.clipboard.writeText(text);
    } catch (err) {
      const ta = document.createElement("textarea");
      ta.value = text;
      ta.setAttribute("readonly", "");
      ta.style.position = "fixed";
      ta.style.left = "-9999px";
      document.body.appendChild(ta);
      ta.select();
      document.execCommand("copy");
      document.body.removeChild(ta);
    }
  };

  logCopyBtn.onclick = function () { copyLogToClipboard(); };
  setLogCopyEnabled(false);

  connectWs();
})();
</script>
</body>
</html>
"""

app = FastAPI()


@app.get("/api/runs")
async def api_list_runs() -> list[dict[str, Any]]:
    async with _archive_lock:
        data = await asyncio.to_thread(_read_archive_file)
    runs = data.get("runs", [])
    return [_run_meta(r) for r in runs if isinstance(r, dict)]


@app.post("/api/runs")
async def api_save_run(body: dict[str, Any] = Body(default_factory=dict)) -> dict[str, Any]:
    label = str(body.get("label", "")).strip()[:200]
    entry: dict[str, Any] = {
        "id": uuid.uuid4().hex,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "label": label,
        "view": state.to_archive_view(),
    }
    async with _archive_lock:
        data = await asyncio.to_thread(_read_archive_file)
        runs = data.setdefault("runs", [])
        if not isinstance(runs, list):
            runs = []
            data["runs"] = runs
        runs.insert(0, entry)
        data["runs"] = data["runs"][:RUN_ARCHIVE_MAX]
        await asyncio.to_thread(_write_archive_file, data)
    return {"ok": True, "entry": _run_meta(entry)}


@app.get("/api/runs/{run_id}")
async def api_get_run(run_id: str) -> dict[str, Any]:
    async with _archive_lock:
        data = await asyncio.to_thread(_read_archive_file)
    for r in data.get("runs", []):
        if isinstance(r, dict) and r.get("id") == run_id:
            return {"entry": r}
    raise HTTPException(status_code=404, detail="Run not found")


@app.delete("/api/runs/{run_id}")
async def api_delete_run(run_id: str) -> dict[str, Any]:
    async with _archive_lock:
        data = await asyncio.to_thread(_read_archive_file)
        kept = [r for r in data.get("runs", []) if not (isinstance(r, dict) and r.get("id") == run_id)]
        data["runs"] = kept
        await asyncio.to_thread(_write_archive_file, data)
    return {"ok": True}


@app.on_event("startup")
async def startup() -> None:
    _print_dev_server_hints()
    global ble_task
    ble_task = asyncio.create_task(link_connection_loop())


@app.on_event("shutdown")
async def shutdown() -> None:
    if ble_task:
        ble_task.cancel()
        try:
            await ble_task
        except asyncio.CancelledError:
            pass


@app.get("/")
async def index() -> HTMLResponse:
    return HTMLResponse(INDEX_HTML)


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    clients.add(ws)
    try:
        await ws.send_json({"type": "state", "data": state.to_view()})
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("type") == "cmd" and isinstance(msg.get("cmd"), str):
                cmd = msg["cmd"].strip().upper()
                allowed = {"TRAIN", "RESUME_TRAIN", "SOLVE", "START", "GETPATH", "STATUS", "CLEAR", "PAUSE", "RESUME"}
                if cmd in allowed:
                    if cmd == "CLEAR":
                        state.clear_dashboard_session()
                        await broadcast_state()
                    uart_cmd = "PATH" if cmd == "GETPATH" else cmd
                    # Leading newline: HM-10 often injects OK+LOST/OK+CONN without \n; without this,
                    # bytes can merge with the next command (e.g. OK+CONN + TRAIN -> UNKNOWN COMMAND).
                    payload = b"\n" + (uart_cmd + "\n").encode("ascii", errors="ignore")
                    await write_queue.put(payload)
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)


