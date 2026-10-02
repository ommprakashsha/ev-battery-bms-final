# EV Battery Health & Thermal Management System
**Linux Kernel Character Device Driver & Userspace Monitoring Daemon**

**Author:** Omm Prakash Sha  

---

## Overview

Modern Electric Vehicles (EVs) rely on multi-cell lithium-ion battery packs that operate under demanding electrical and thermal conditions. Failures in cell monitoring can lead to cell degradation, severe capacity loss, or catastrophic thermal runaway.

This project implements a multi-tiered Battery Management System (BMS) for Linux:
1. **Linux Kernel Driver (`kernel/bms_driver.ko`)**: Simulates a 6-cell lithium-ion battery pack using a 200ms kernel timer, protects state via IRQ-safe spinlocks, exposes diagnostics through `/proc/bms_status`, and provides an IOCTL interface (`/dev/bms_drv`).
2. **BMS Userspace Daemon (`src/bms_daemon_main.cpp`)**: Polls cell telemetry, filters sensor noise with a custom moving average template, runs Coulomb counting for State of Charge (SoC), monitors thermal rate-of-rise ($dT/dt$) for early runaway detection, and manages passive cell balancing.
3. **Live Terminal Monitor (`src/bms_dashboard.cpp`)**: A separate process that reads telemetry from POSIX shared memory via a lock-free sequence lock (seqlock) and listens for asynchronous alerts over POSIX message queues.
4. **Control Tool (`tools/bms_ctl.c`)**: A command-line utility to query driver telemetry, trigger manual balancing, and inject faults (overvoltage, overtemperature, undervoltage) to test safety responses.

---

## System Architecture

```
+-------------------------------------------------------------------------+
|                               USERSPACE                                 |
|                                                                         |
|  +---------------------------------------+      +--------------------+  |
|  |             bms_daemon                |      |   bms_dashboard    |  |
|  |                                       |      | (Live Terminal UI) |  |
|  |  +----------------+ +---------------+ |      +--------------------+  |
|  |  |  SoCCalculator | | CellBalancer  | |                 ^            |
|  |  | (Coulomb Count)| | (Passive 50mV)| |                 | Lock-free  |
|  |  +----------------+ +---------------+ |                 | Seqlock    |
|  |  +----------------+ +---------------+ |                 v            |
|  |  | ThermalRunaway | |  MovingAvg<T> | |      +--------------------+  |
|  |  |  (dT/dt Alert) | |  (FIR Filter) | |      | POSIX Shared Memory|  |
|  |  +----------------+ +---------------+ |      |     (/bms_shm)     |  |
|  |                                       |      +--------------------+  |
|  |  +----------------+ +---------------+ |                 ^            |
|  |  | AlertSender    | |   BmsLogger   | |                 |            |
|  |  | (POSIX MQ)     | |   (CSV File)  | |                 |            |
|  |  +-------+--------+ +---------------+ |                 |            |
|  +----------|------------------+---------+                 |            |
|             |                  |                           |            |
|             v                  +---------------------------+            |
|     +---------------+                                                   |
|     |POSIX Msg Queue|                                                   |
|     | (/bms_alerts) |                                                   |
|     +---------------+                                                   |
|             ^                                                           |
+-------------|-----------------------------------------------------------+
|             | IOCTL: GET_PACK_DATA, INJECT_FAULT, CLEAR_FAULTS, etc.    |
|             v                                                           |
|     +---------------+                                                   |
|     | /dev/bms_drv  |  <-----+ tools/bms_ctl (Manual Test / CLI Utility)|
+-----|---------------+---------------------------------------------------+
|     |                                                                   |
|     |                        KERNEL SPACE                               |
|     v                                                                   |
|  +-------------------------------------------------------------------+  |
|  |                          bms_driver.ko                            |  |
|  |                                                                   |  |
|  |  +--------------------+  +------------------+  +---------------+  |  |
|  |  | Kernel Timer 200ms |  | Spinlock (IRQ)   |  | /proc/bms_    |  |  |
|  |  | (Cell Simulation)  |  | (State Protect)  |  | status (Seq)  |  |  |
|  |  +--------------------+  +------------------+  +---------------+  |  |
|  +-------------------------------------------------------------------+  |
+-------------------------------------------------------------------------+
```

---

## Core Algorithms & Logic

### 1. State of Charge (SoC) - Coulomb Counting
Coulomb counting tracks the cumulative charge transferred into or out of each cell:

$$SoC(t) = SoC(0) - \frac{1}{Q_{nominal}} \int_{0}^{t} I(\tau) \, d\tau$$

- **Discrete update**: $\Delta Q = I \times \Delta t$
- Positive current indicates discharge; negative current indicates charging.
- Evaluated every 200ms step inside `src/SoCCalculator.hpp`.

### 2. Thermal Runaway Early Detection ($dT/dt$)
Standard threshold detection (e.g., $T > 65^\circ\text{C}$) often triggers too late. The daemon monitors the first-order temperature derivative:

$$\frac{dT}{dt} = \frac{T(k) - T(k-1)}{\Delta t} \times 60 \quad [^\circ\text{C}/\text{min}]$$

- **Critical Alert Condition**: Triggered when $T_{cell} > 65^\circ\text{C}$ **and** $\frac{dT}{dt} > 5^\circ\text{C}/\text{min}$.
- Dispatches emergency notifications over `/bms_alerts` to alert external systems.

### 3. Passive Cell Balancing
Manufacturing differences cause slight variations in cell capacity and internal resistance:
- The controller identifies the minimum voltage ($V_{min}$) across all operational cells.
- If any cell exceeds $V_{min} + 50\text{ mV}$, the balancing logic flags that cell and simulates a $500\text{ mA}$ bleed current through a shunt resistor.
- Total energy dissipated as heat is tracked in Joules.

### 4. Moving Average Template (`MovingAverage<T, N>`)
Sensor telemetry is passed through an in-memory ring-buffer moving average filter. An $O(1)$ running sum ensures minimal computation overhead and avoids dynamic heap allocations.

---

## Protection Limits & Operating Parameters

| Parameter | Warning Limit | Critical Threshold | Action |
|---|---|---|---|
| Cell Overvoltage (OV) | $> 4.15\text{ V}$ | $> 4.20\text{ V}$ | Inhibit charge |
| Cell Undervoltage (UV) | $< 2.90\text{ V}$ | $< 2.50\text{ V}$ | Inhibit discharge |
| Overtemperature (OT) | $> 55.0^\circ\text{C}$ | $> 65.0^\circ\text{C}$ | Reduce current / Disconnect |
| Undertemperature (UT) | $< 0.0^\circ\text{C}$ | $< -20.0^\circ\text{C}$ | Restrict fast charging |
| Overcurrent (OC) | $> 25.0\text{ A}$ | $> 30.0\text{ A}$ | Trip safety disconnect |
| Cell Imbalance ($\Delta V$) | $> 30\text{ mV}$ | $> 50\text{ mV}$ | Trigger passive balancing |

---

## Project Structure

```
ev-battery-bms/
├── CMakeLists.txt              # CMake build configuration for userspace
├── run.sh                      # Automation script (build, load, run, test)
├── README.md                   # Project documentation
│
├── kernel/                     # Kernel character device driver
│   ├── bms_driver.h            # Shared driver structs and IOCTL commands
│   ├── bms_driver.c            # Linux kernel character driver implementation
│   └── Makefile                # Kbuild module makefile
│
├── src/                        # Userspace BMS software stack
│   ├── BatteryCell.hpp         # Cell telemetry model and status helpers
│   ├── MovingAverage.hpp       # Generic C++ template moving average filter
│   ├── SoCCalculator.hpp       # Coulomb counting SoC estimator
│   ├── ThermalRunawayDetector.hpp # Observer-pattern thermal runaway detector
│   ├── CellBalancer.hpp        # Passive cell balancing controller
│   ├── BmsLogger.hpp           # Thread-safe telemetry CSV logger
│   ├── bms_daemon_main.cpp     # Main BMS daemon (IOCTL polling, IPC, threads)
│   └── bms_dashboard.cpp       # Terminal dashboard (shared memory monitor)
│
├── tools/                      # Debugging and driver utilities
│   └── bms_ctl.c               # CLI tool for IOCTL queries and fault injection
│
└── docs/                       # Technical references
    └── algorithms.md           # Mathematical models and equations
```

---

## Building and Running

### Prerequisites
A Linux system (Ubuntu 20.04+ or equivalent) with the kernel development headers installed:
```bash
sudo apt update
sudo apt install -y build-essential cmake linux-headers-$(uname -r)
```

### 1. Build Kernel Driver and Userspace Binaries
```bash
chmod +x run.sh
./run.sh build
```

This compiles:
- `kernel/bms_driver.ko` via Kbuild
- `build/bms_daemon`, `build/bms_dashboard`, and `build/bms_ctl` via CMake

### 2. Load the Kernel Module
```bash
./run.sh load
```
Verify driver initialization:
```bash
lsmod | grep bms_driver
cat /proc/bms_status
```

### 3. Run Daemon and Live Dashboard
To start the daemon in the background and launch the terminal UI:
```bash
./run.sh run
```

---

## Testing & Fault Injection

The `bms_ctl` command-line utility allows injecting hardware faults directly into the kernel driver to verify the software response:

```bash
# Display full pack status directly from driver
./build/bms_ctl status

# Inject overtemperature on Cell 2 (68.0 °C)
./build/bms_ctl inject 2 2 68.0

# Inject overvoltage on Cell 0 (4.25 V)
./build/bms_ctl inject 0 0 4.25

# Inject undervoltage on Cell 3 (2.40 V)
./build/bms_ctl inject 3 1 2.40

# Clear all injected faults
./build/bms_ctl clear
```

Alternatively, you can use the helper flags in `run.sh`:
```bash
./run.sh inject_ot     # Injects overtemperature fault
./run.sh inject_ov     # Injects overvoltage fault
./run.sh inject_uv     # Injects undervoltage fault
./run.sh clear         # Resets all faults
```

---

## Verification & Output

### 1. `/proc/bms_status` Output
```
=== EV Battery Management System Status ===
Pack Voltage   : 22.14 V
Pack Current   : 12.00 A
Avg Temperature: 26.2 C
Avg SoC        : 84.8 %
Active Faults  : 0
Sample Count   : 142
Balancing Mode : Passive

Cell Telemetry:
Cell  Voltage (V)  Temp (C)  Current (A)  SoC (%)   State     Faults
--------------------------------------------------------------------
  0      3.692       25.8       12.00       84.8    NORMAL    None
  1      3.688       26.1       12.00       84.9    NORMAL    None
  2      3.695       26.4       12.00       84.7    NORMAL    None
  3      3.684       26.0       12.00       84.8    NORMAL    None
  4      3.690       26.3       12.00       84.8    NORMAL    None
  5      3.691       26.6       12.00       84.9    NORMAL    None
===========================================
```

### 2. Live Dashboard Output (`bms_dashboard`)
```
================================================================
         EV Battery Management System - Status Monitor          
================================================================

  PACK TELEMETRY
  --------------------------------------------------------------
  Pack Voltage : 22.1 V    Pack Current:  12.0 A
  Avg Temp     : 26.3 C    Avg SoC     :  84.8 %
  Active Faults:    0      Sample Count: 284

  ID | Voltage | Temp  | SoC  | State    | SoC Indicator        | Faults
  ---+---------+-------+------+----------+----------------------+-------
  0  | 3.692 V | 25.8 C|  84% | NORMAL   | [################----] | -  
  1  | 3.688 V | 26.1 C|  85% | NORMAL   | [################----] | -  
  2  | 3.695 V | 26.4 C|  84% | NORMAL   | [################----] | -  
  3  | 3.684 V | 26.0 C|  85% | NORMAL   | [################----] | -  
  4  | 3.690 V | 26.3 C|  84% | NORMAL   | [################----] | -  
  5  | 3.691 V | 26.6 C|  85% | NORMAL   | [################----] | -  

  SYSTEM ALERTS
  --------------------------------------------------------------
  Normal operation - No active alerts

  Update Frame: 284 | Press Ctrl+C to exit
```

---

## License
Kernel module components are distributed under GPL-2.0. Userspace software components are distributed under the MIT License.
