# ECNG 4504 — Autonomous maze-learning & maze-solving robot

**American University in Cairo · School of Sciences and Engineering**  
**ECNG 4504 — Embedded Systems for Wireless Communications · Spring 2026**

---

## Abstract

This project is an **autonomous line-following robot** that **learns** an unknown maze in a training pass and later **executes a shortened turn sequence** to reach the goal efficiently. A **PC host** connects over **Bluetooth** (BLE with HM-10, or **Bluetooth Classic SPP** with a ZS-040 / HC-05) to issue commands, stream status, and visualize behavior. On-robot software is a single Arduino sketch (`working_code.ino`) built around **discrete line following** (no PID), **left-hand-rule** decisions at intersections, **in-place path simplification**, and **EEPROM** persistence of the optimized path.

---

## How we approached the project

We treated the work as a loop of **measure → interpret → act → log**, repeated until the vehicle behaved reliably on real tape and real junction geometry.

1. **Hardware and sensing**  
   We brought up **five reflective sensors** (left-to-right array), **differential drive** via motor drivers, and **wireless UART on `Serial1`** (HM-10 BLE or ZS-040 / HC-05 classic) so the laptop could see the same telemetry as USB debug.

2. **Baseline motion**  
   Before worrying about mapping, we stabilized **line following** using **fixed PWM steps** and simple rules from the sensor pattern (centered forward, soft correct near center, stronger correct when the line moves under an outer sensor). This deliberately avoids PID tuning loops in favor of predictable, repeatable motion on our course.

3. **Seeing intersections**  
   Real junctions rarely match ideal drawings: wide tape, gaps, and noise meant “all black” or “wide center” could mean different things. We added **timing and lockouts** (`NODE_LOCKOUT_MS`) so the robot did not re-trigger the same junction, and **short probe moves** with **burst sampling** across the array to classify **left / straight / right** exits when possible.

4. **Training as a path string**  
   During **training**, at each real decision node the robot applies the **left-hand rule**: prefer **L**, then **S**, then **R**. The sequence of taken turns is appended to a **raw path** (`L`, `S`, `R`, or `B` for dead-end / U-turn). **Dead ends** are detected when the line is lost for long enough and recovered with a **U-turn** and a recorded **`B`**.

5. **When geometry is ambiguous**  
   For **multi-exit** junctions, recording is **deferred** briefly: after the turn, the firmware **accumulates “witness”** from the outer sensors and **scores candidate exit masks** so the stored junction record matches what the robot actually saw after committing to a turn.

6. **Optimizing the remembered path**  
   After training, the firmware **folds** the raw path: any **middle `B`** (dead-end) with neighbors is combined using **turn angles** (90° increments) so backtracking detours collapse into a single equivalent turn. That yields the **optimal path** string used in solve mode.

7. **Solve mode**  
   In **solving**, the robot walks the **optimized** sequence: at each true decision node it executes the next character of `optimalPath`. If sensor classification and the stored path disagree, it logs a **mismatch** and falls back to the left-hand choice so the run degrades safely.

8. **Host software**  
   We built a **FastAPI** dashboard with **WebSockets** (`Bluetooth stuff/maze_dashboard/`) — **Bleak** for BLE, or **pyserial** to a paired **Bluetooth SPP** serial port on macOS — plus a small **Bun** log viewer (`Bluetooth stuff/log-viewer/`) to command `TRAIN` / `START`, watch logs, and debug link issues.

**Diagrams in this repo:** high-level software structure is in [`system_architecture.png`](system_architecture.png); the PCB is exported as [`PCB Schematic.png`](PCB Schematic.png). Team and roles are in [`team.md`](team.md).

---

## Firmware: theory and implementation

The firmware implements a **maze-learning** pipeline that is **graph-free** on the MCU: the map is a **string of turns**, not an explicit adjacency list. That matches the course emphasis on embedded constraints and clear state machines.

### Line following without PID

Line tracking uses **rule-based steering** (`followLineNoPID`): the center sensor defines “on track,” left/right pairs request **differential spins** at tuned speeds (`BASE_SPEED`, `TURN_SPEED`, `ALIGN_SPEED`, etc.). There is **no PID controller**—the design trades fine continuous control for **bounded behavior** that is easier to reason about on inconsistent surfaces. After turns, **`alignToCenterStrict`** recenters on a **center-only** pattern before moving forward.

### Junction detection and training probes

A **node candidate** is inferred from reflective patterns (`nodeCandidateNow`): e.g. **all sensors on line** (wide intersection) or **outer sensors with inner body**. After a lockout window, the robot stops and runs **`inspectAndHandleNodeTraining`**: short forward motion **samples bursts** of left/right/straight visibility to set **exit masks** (bits for L, S, R). The **left-hand rule** (`selectTurn`) chooses the physical turn. **Finish** is a sustained **all-black** region confirmed as the end box (`confirmEndBox`).

### Recording and deferred resolution

For junctions with **multiple** plausible exits, the firmware may **defer** committing the recorded mask (`finalizePendingTrainingRecord`, `RECORD_RESOLVE_MS`) so post-turn sensor **witnesses** improve which mask is stored with the segment time. **Single-exit** corridors force a turn without the same ambiguity.

### Path optimization (dead-end folding)

The **raw** training path includes detours. **`buildOptimalPath`** repeatedly replaces triples where the **middle turn is `B`** by summing **turn angles** (L = 270°, S = 0°, R = 90°, B = 180°) and mapping the result back to a single letter. This removes redundant **out-and-back** structure while preserving net heading.

### Solving

**Solve** mode (`inspectAndHandleNodeSolving`) indexes **`optimalPath`** with `solveStepIndex`. Only at **true** decision nodes (multiple exits, or zero exits) does it consume the next stored turn; **forced** corridors follow the obvious exit. EEPROM (`EepromMazeData`) can persist raw and optimized paths across power cycles (`LOAD`, `CLEAR`, auto-load on boot).

### Commands and telemetry

**USB and wireless UART** share the same command set (`TRAIN`, `START`/`SOLVE`, `STATUS`, `DUMP`, `LOAD`, `CLEAR`, `STOP`). Status lines (`STATE`, `STAGE`, paths, node logs) go to both streams for the dashboard and debugging.

---

## Repository contents

| Deliverable | Location |
|-------------|----------|
| MCU firmware (final) | [`working_code.ino`](working_code.ino) |
| Backend (FastAPI, BLE, WebSockets) | [`Bluetooth stuff/maze_dashboard/`](Bluetooth%20stuff/maze_dashboard/) |
| Web UI (log viewer) | [`Bluetooth stuff/log-viewer/`](Bluetooth%20stuff/log-viewer/) |
| BLE helper scripts | [`Bluetooth stuff/ble_log_receiver.py`](Bluetooth%20stuff/ble_log_receiver.py), [`Bluetooth stuff/ble_scan.py`](Bluetooth%20stuff/ble_scan.py), [`Bluetooth stuff/requirements.txt`](Bluetooth%20stuff/requirements.txt) |
| Bill of materials | [`component_list(1).xlsx`](component_list%281%29.xlsx) |
| Software architecture diagram | [`system_architecture.png`](system_architecture.png) |
| PCB schematic (image export) | [`PCB Schematic.png`](PCB%20Schematic.png) |
| Team | [`team.md`](team.md) |
| 3D / mechanical design | [`3d design/`](3d%20design/) (Fusion 360 `.f3z`) |

**KiCAD:** the repository includes the **schematic image** above. Native KiCAD project files (`.kicad_pro`, `.kicad_sch`, `.kicad_pcb`) are not in this tree; add them under a clear folder (for example `hardware/kicad/`) when you commit them.

---

## Running the PC applications

**Maze dashboard**

```bash
cd "Bluetooth stuff/maze_dashboard"
python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt
uvicorn app:app --reload --host 127.0.0.1 --port 8765
```

**ZS-040 / HC-05 on macOS (Bluetooth Classic SPP)** — pair the module in **System Settings → Bluetooth**, then pick the outgoing serial device (names vary; often under `/dev/cu.*`):

```bash
ls /dev/cu.*
export MAZE_BT_SERIAL=/dev/cu.YourModule-DevB
export MAZE_BT_BAUD=9600
cd "Bluetooth stuff/maze_dashboard"
source .venv/bin/activate
uvicorn app:app --reload --host 127.0.0.1 --port 8765
```

After a successful connection, you can use `export MAZE_BT_SERIAL=last` to reuse `bt_serial_last.json` beside `app.py`. Leave `MAZE_BT_SERIAL` unset to use **BLE** (HM-10) with Bleak as before.

**Path preview (SVG map):** The dashboard builds a **turtle-geometry** polyline from path strings and from **training/solve** turn lines in the log (`FORCED_TURN`, `SOLVE_FORCED_TURN`, `SOLVE_NODE`, etc.). The preview appends **one grid segment after the last logged turn** so the **final leg** and **forced** corridor exits show the same corners you see on the track; that step is **visualization-only** and does not change what the robot records or executes. Regression checks for this live in `Bluetooth stuff/maze_dashboard/test_map_visualization.py` (run with `python -m unittest` from that folder, using the project venv).

**Log viewer** (see `Bluetooth stuff/log-viewer/package.json` for scripts)

```bash
cd "Bluetooth stuff/log-viewer"
bun install
# Use the same MAZE_BT_SERIAL in this shell so the Python helper can open the port
export MAZE_BT_SERIAL=/dev/cu.YourModule-DevB
bun run serve
```

---

*Course project repository: **ecng4504-maze-solver_team_5** — maze learning and solving per the ECNG 4504 charter.*
