# ECNG 4504 — Autonomous maze-learning & maze-solving robot

**American University in Cairo · School of Sciences and Engineering**  
**ECNG 4504 — Embedded Systems for Wireless Communications · Spring 2026**

Course project: an autonomous robot that learns an unknown maze, then computes and executes a path. A PC application talks to the vehicle over **BLE**; **Python** (FastAPI) and a small **web** UI support commands, logging, and visualization.

## What is in this repository

| Deliverable | Location |
|-------------|----------|
| **MCU firmware** (final) | `working_code.ino` |
| **Backend** (FastAPI, BLE) | `Bluetooth stuff/maze_dashboard/` |
| **Web UI** (log viewer) | `Bluetooth stuff/log-viewer/` |
| **BLE helper scripts** | `Bluetooth stuff/ble_log_receiver.py`, `Bluetooth stuff/ble_scan.py` — root `Bluetooth stuff/requirements.txt` for lightweight deps |
| **Bill of materials** | `component_list(1).xlsx` |
| **PCB schematic** (exported image) | `PCB Schematic.png` |
| **Team details** | `team.txt` |
| **3D / mechanical design** | `3d design/` (Fusion 360 `.f3z`) |

### KiCAD source files

The **schematic screenshot** above is included for submission. There are **no** KiCAD project files (`.kicad_pro`, `.kicad_sch`, `.kicad_pcb`) in this tree yet. When you add them, place them under a clear folder (for example `hardware/kicad/`) and commit so reviewers can open the project in KiCAD.

## Run the PC applications locally

**Maze dashboard (Python)**

```bash
cd "Bluetooth stuff/maze_dashboard"
python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt
uvicorn app:app --reload --host 127.0.0.1 --port 8000
```

**Log viewer (see `Bluetooth stuff/log-viewer/package.json` for scripts)**

```bash
cd "Bluetooth stuff/log-viewer"
bun install
bun run serve
```

---

*Repository: **ecng4504-maze-solver** — maze learning and solving per the course charter.*
