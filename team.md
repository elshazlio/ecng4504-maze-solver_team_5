# Team 5 — ECNG 4504

**Embedded Systems for Wireless Communications · Spring 2026**  
American University in Cairo · School of Sciences and Engineering

---

## Team members

| Name | Student ID | Email |
|------|------------|--------|
| Abanoub Emad Said Kamel | 900201630 | [abanoubemad@aucegypt.edu](mailto:abanoubemad@aucegypt.edu) |
| John Kamal | 900212208 | [johnk.kamal@aucegypt.edu](mailto:johnk.kamal@aucegypt.edu) |
| Mina Yasser Gerges | 900214039 | [minayasser@aucegypt.edu](mailto:minayasser@aucegypt.edu) |
| Omar ElShazli | 900201477 | [elshazlio@aucegypt.edu](mailto:elshazlio@aucegypt.edu) |

---

## About this project (repository contents)

This repository is the **Team 5** submission for an **autonomous maze-learning and maze-solving** robot.

### On-vehicle software

- **`working_code.ino`** — Arduino sketch for an **Arduino Mega** driving a **differential** motor pair, reading **five reflective line sensors**, and maintaining **training** and **solving** logic (internal path records, junction handling, EEPROM-backed checkpoints where implemented). A **Bluetooth serial** link (**Serial1**, typical HC-05-style module) connects the robot to a laptop at **9600 baud** (USB debug at **115200**).

### PC software

- **`Bluetooth stuff/maze_dashboard/`** — **Python** host: **FastAPI** application for commands, status, and BLE interaction with the robot during runs.
- **`Bluetooth stuff/log-viewer/`** — Small **web** front end (Bun) for viewing and working with UART/BLE logs from the HM-10 style workflow.
- **`Bluetooth stuff/ble_log_receiver.py`**, **`ble_scan.py`**, and **`Bluetooth stuff/requirements.txt`** — Lightweight helpers and dependencies for BLE discovery and logging.

### Hardware and mechanical documentation

- **`component_list(1).xlsx`** — Bill of materials / component list.
- **`PCB Schematic.png`** — Exported **schematic** image (KiCAD native project files are not included in this tree; only this image is submitted here).
- **`3d design/chasis.f3z`** — **Fusion 360** chassis / mechanical design archive.

For setup commands and paths, see the root **`README.md`**.
