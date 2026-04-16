# Team 5 — ECNG 4504

**Course:** Embedded Systems for Wireless Communications · Spring 2026  
**Institution:** American University in Cairo · School of Sciences and Engineering

## Members

| Name | ID | Email |
|------|-----|--------|
| Abanoub Emad Said Kamel | 900201630 | [abanoubemad@aucegypt.edu](mailto:abanoubemad@aucegypt.edu) |
| John Kamal | 900212208 | [johnk.kamal@aucegypt.edu](mailto:johnk.kamal@aucegypt.edu) |
| Mina Yasser Gerges | 900214039 | [Minayasser@aucegypt.edu](mailto:Minayasser@aucegypt.edu) |
| Omar ElShazli | 900201477 | [Elshazlio@aucegypt.edu](mailto:Elshazlio@aucegypt.edu) |

---

## Contributions (aligned with this repository)

### Mina Yasser Gerges

Led the **on-robot firmware** in `working_code.ino`: maze **training** and **solving** behavior (map build, optimal path execution, left-hand-rule navigation), sensor and motion tuning, and calibration for reliable runs. Integrated **BLE (HM-10)** with the main control flow and supported **robot-side** wireless testing so the PC tools could drive and monitor the vehicle end-to-end.

### Abanoub Emad Said Kamel

Led **hardware**: component selection and the **bill of materials** (`component_list(1).xlsx`), wiring, and motor/sensor interfacing. Prepared the **schematic** (exported as `PCB Schematic.png` in this repo). Contributed to **embedded software** alongside the team: MCU structure, line following, junction / turn handling, and merging software modules into a coherent sketch.

### John Kamal & Omar ElShazli

Led **host-side software** over **BLE** to the robot:

- **`Bluetooth stuff/maze_dashboard/`** — **FastAPI** application with **WebSockets**, **Bleak**-based BLE (Nordic UART / HM-10-style UART), command and status exchange, path and run data from the firmware, and a browser UI served from the same stack.
- **`Bluetooth stuff/log-viewer/`** — Bun-based **web** log viewer wired to **`ble_log_receiver.py`** for UART/BLE log streaming and debugging.
- Supporting scripts such as **`ble_discover.py`**, **`ble_log_receiver.py`**, and **`ble_scan.py`**, plus reliability testing of the wireless link with the vehicle.

---

*Repository deliverables (firmware, PC apps, BOM, schematic image, mechanical CAD) are listed in [`README.md`](README.md).*
