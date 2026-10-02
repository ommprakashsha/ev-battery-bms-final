# EV Battery Health & Thermal Management System
## Capstone Project Presentation Deck & Viva Script

---

### Slide 1: Title Slide
- **Title**: EV Battery Health & Thermal Management System
- **Subtitle**: Linux Kernel Character Driver, Coulomb Counting SoC, and Real-Time Thermal Runaway Early Detection Daemon
- **Author**: Omm Prakash Sha
- **Domain**: Automotive Embedded Systems & Linux Systems Programming
- **Core Technologies**: Linux Kernel (Char Driver, IOCTL, Procfs, Spinlocks), C++17, POSIX IPC (Shared Memory with Seqlock, Message Queues)

> **Speaker Script**:
> "Good morning/afternoon everyone. Today I am presenting my Capstone Project: an EV Battery Health and Thermal Management System. The project bridges low-level Linux kernel device drivers with a multi-threaded C++ userspace monitoring daemon to detect thermal runaway and cell imbalance in real-time, preventing catastrophic battery fires in electric vehicles."

---

### Slide 2: Problem Statement & Industrial Context
- **The Challenge**: High-density Lithium-ion battery packs in electric vehicles (EVs) operate under extreme electrical and thermal stress.
- **Recent Catastrophes**: Incidents such as the 2022 Ola Electric scooter fires highlight how internal short circuits and thermal runaway cause catastrophic fires within seconds.
- **Technical Gaps in Standard Systems**:
  - Simple static threshold triggers (e.g. $T > 65^\circ\text{C}$) often alarm too late when thermal runaway has already begun.
  - Series cell strings suffer from capacity loss due to manufacturing tolerances and non-uniform thermal dissipation.
- **Our Goal**: Build an embedded software architecture capable of deterministic 200ms telemetry sampling, rate-of-rise ($dT/dt$) thermal gradient detection, and passive cell balancing.

> **Speaker Script**:
> "Li-ion cells have an operating temperature window between 15°C and 45°C. Beyond 65°C, exothermic chemical breakdown causes thermal runaway where temperature escalates uncontrollably. Traditional BMS systems only check if temperature exceeds a static threshold. In our architecture, we track both temperature magnitude and its rate-of-rise (dT/dt), enabling the system to detect thermal runaway early and trigger the safety contactor before fire breaks out."

---

### Slide 3: System Architecture Overview
- **Layer 1 - Linux Kernel Space**:
  - `bms_driver.ko`: Character device (`/dev/bms_drv`) simulating a 6-cell Li-ion string.
  - Periodic 200ms interrupt-safe kernel timer (`struct timer_list`).
  - Diagnostic procfs entry (`/proc/bms_status`) via `seq_file`.
  - 6 IOCTL commands for telemetry queries and fault injection.
- **Layer 2 - Userspace Monitoring Daemon**:
  - `bms_daemon`: C++ multi-threaded daemon communicating with `/dev/bms_drv`.
  - Coulomb counting SoC estimator and 50mV passive cell balancer.
  - Generic template moving average filter (`MovingAverage<T, N>`) for ADC noise reduction.
  - CSV telemetry logger (`/var/log/bms/bms_data.csv`).
- **Layer 3 - Inter-Process Communication & UI**:
  - POSIX Shared Memory (`/bms_shm`) with lock-free Sequence Lock (seqlock).
  - POSIX Message Queue (`/bms_alerts`) for asynchronous high-priority alarm broadcasts.
  - Terminal Monitor (`bms_dashboard`) & Web UI for live telemetry visualization.

> **Speaker Script**:
> "Our architecture enforces a clean separation of concerns. The kernel driver handles high-frequency, deterministic hardware simulation and exposes a standardized char device. The userspace C++ daemon runs the algorithms, logs data, and writes to shared memory. The dashboard runs in a completely separate process, reading telemetry without taking locks or blocking the main daemon."

---

### Slide 4: Linux Kernel Driver Implementation
- **Device Registration**:
  - Dynamic allocation via `alloc_chrdev_region()`, initialized with `cdev_init()` and exposed via `cdev_add()`.
  - Class and device created dynamically under `/sys/class/bms` and `/dev/bms_drv`.
- **Interrupt-Safe Timer**:
  - Driven by `struct timer_list` scheduled via `mod_timer()` every 200ms.
  - Executes in atomic softirq context where sleeping/blocking is strictly forbidden.
- **Concurrency & Spinlock Protection**:
  - Shared pack state protected by `spinlock_t bms_spinlock`.
  - Synchronized via `spin_lock_irqsave()` and `spin_unlock_irqrestore()` to prevent races between timer interrupt execution and userspace IOCTL system calls.
- **Soft-Float Representation**:
  - Avoids forbidden FPU operations in kernel mode by scaling values as fixed-point integers (millivolts, centi-degrees, centi-amps).

> **Speaker Script**:
> "In kernel space, you cannot use mutexes inside timer callbacks because timers run in interrupt context and cannot sleep. We used an IRQ-safe spinlock to guarantee atomic access to pack telemetry. Furthermore, because floating-point hardware registers are not preserved across kernel context switches without expensive FPU saving, we engineered an internal fixed-point representation that is converted to the struct ABI only on transfer."

---

### Slide 5: State of Charge (SoC) - Coulomb Counting Algorithm
- **Principle**:
  $$SoC(t) = SoC(0) - \frac{1}{Q_{nominal}} \int_{0}^{t} I(\tau) \, d\tau$$
- **Discrete Euler Integration** (calculated every $\Delta t$ seconds):
  $$\Delta Q = I \times \Delta t \quad [\text{Ampere-seconds}]$$
  $$Q_{used}(k) = Q_{used}(k-1) + \Delta Q$$
  $$SoC(k) = SoC(0) - \left( \frac{Q_{used}(k)}{Q_{nominal}} \right) \times 100\%$$
- **Sign Convention**: Positive current indicates discharge; negative current indicates charging/regenerative braking.
- **Protection**: Output clamped strictly in $[0.0, 100.0]\%$ via `std::clamp` (C++17).

> **Speaker Script**:
> "Coulomb counting is the industry-standard algorithm used in production electric vehicles. In SoCCalculator.hpp, we track the discrete charge delta on every polling cycle. By multiplying current in Amperes by the time step in seconds, we calculate charge consumed in Amp-seconds, updating remaining capacity. Clamping ensures no numerical overflow occurs due to noise."

---

### Slide 6: Thermal Runaway Early Detection ($dT/dt$)
- **Runaway Criterion**:
  $$\text{CRITICAL ALERT} \iff (T_{cell} > 65.0^\circ\text{C}) \quad \land \quad \left( \frac{dT}{dt} > 5.0^\circ\text{C}/\text{min} \right)$$
- **Rate-of-Rise Calculation**:
  $$\frac{dT}{dt} = \frac{T(k) - T(k-1)}{\Delta t} \times 60 \quad [^\circ\text{C}/\text{min}]$$
- **4-Stage Health Hierarchy**:
  - `NORMAL` ($\le 45^\circ\text{C}$): Nominal operating range.
  - `WARM` ($45^\circ\text{C} - 55^\circ\text{C}$): Reduce max charging current.
  - `HOT` ($55^\circ\text{C} - 65^\circ\text{C}$): Pause fast charging; activate maximum cooling.
  - `CRITICAL` ($> 65^\circ\text{C} \text{ and } dT/dt > 5^\circ\text{C}/\text{min}$): Emergency contactor disconnect.
- **Observer Design Pattern**:
  - `ThermalRunawayDetector` acts as Subject.
  - `ThermalObserver` interface notifies `AlertSender`, `BmsLogger`, and `SharedMemoryWriter`.

> **Speaker Script**:
> "The key innovation in our thermal engine is the derivative threshold. When a cell enters thermal runaway, its chemical breakdown causes an exponential temperature surge. By calculating dT/dt using backward difference, we detect the signature gradient before the temperature reaches destructive levels, immediately dispatching a high-priority POSIX message to disconnect high-voltage contactors."

---

### Slide 7: Passive Cell Balancing Controller
- **Problem**: In a 6S battery string, individual cell manufacturing tolerances cause voltage drift. The highest cell hits cutoff first, leaving other cells partially charged.
- **Balancing Algorithm**:
  1. Find minimum voltage across operational cells: $V_{min} = \min(V_0, V_1, ..., V_5)$.
  2. If cell $i$ has $V_i > V_{min} + 50\text{ mV}$, flag cell $i$ in balancing bitmask.
  3. Activate shunt resistor drawing $I_{bleed} = 500\text{ mA}$ to dissipate excess charge.
- **Energy Dissipation Tracking**:
  $$\Delta E = V_{cell} \times I_{bleed} \times \Delta t \quad [\text{Joules}]$$

> **Speaker Script**:
> "Passive cell balancing bleeds off excess energy from the highest-voltage cells through resistive shunts. Our CellBalancer class checks all non-dead cells against a 50 millivolt window above V_min. We also integrate power over time to track total heat dissipated in Joules, allowing thermal engineers to size heatsinks accordingly."

---

### Slide 8: C++ Templates & Lock-Free IPC Design
- **Generic C++ Template: `MovingAverage<T, N>`**:
  - Sliding-window FIR filter for smoothing ADC sensor noise.
  - Constant $O(1)$ running-sum circular ring buffer.
  - Zero dynamic heap allocation (uses `std::array<T, N>`).
- **POSIX Shared Memory (`/bms_shm`)**:
  - Memory-mapped buffer (`mmap`) enabling zero-copy telemetry sharing.
- **Lock-Free Sequence Lock (Seqlock)**:
  - Writer increments sequence to an **ODD** number prior to writing.
  - Writer writes telemetry struct.
  - Writer increments sequence to an **EVEN** number after writing.
  - Reader checks sequence before and after copying: if sequence changed or was odd, it retries.
  - Result: No mutex contention; reader never blocks the writer.
- **POSIX Message Queue (`/bms_alerts`)**:
  - Asynchronous, priority-tagged fault signaling (`mq_send` with priority 2 for critical alarms).

> **Speaker Script**:
> "In embedded systems, predictability is everything. Our MovingAverage filter uses templates and std::array to avoid any heap allocations on the stack. For IPC, standard mutexes can cause priority inversion or UI stutter. We implemented a lock-free Sequence Lock pattern in shared memory, allowing the 5Hz dashboard to read fresh data atomically without ever stalling the safety daemon."

---

### Slide 9: Driver Testing & Fault Injection (`bms_ctl`)
- **Control Tool**: Standalone C utility `tools/bms_ctl.c` communicating directly with `/dev/bms_drv`.
- **Test Scenarios**:
  - **Overvoltage Injection**: Inject 4.25V on Cell 0 $\to$ Daemon flags `OVERVOLTAGE` bit and warns charger to inhibit current.
  - **Thermal Runaway Injection**: Inject 68.0°C on Cell 2 $\to$ Derivative exceeds 5°C/min, trips `CRITICAL_ALERT`, opens emergency contactors.
  - **Undervoltage Injection**: Inject 2.40V on Cell 3 $\to$ Triggers `UNDERVOLTAGE` fault and loads disconnect.
  - **Reset Verification**: `bms_ctl clear` sends IOCTL to clear all fault bitmasks and restore baseline telemetry.

> **Speaker Script**:
> "Safety systems cannot just be written; they must be verified. We built the bms_ctl utility specifically to test our error handlers by injecting physical faults via IOCTL. When we inject 68°C into Cell 2, we can observe the daemon react within 200 milliseconds, log the fault, broadcast the emergency alert, and update the dashboard in real-time."

---

### Slide 10: Live Demonstration & Telemetry Output
- **Diagnostic Output (`/proc/bms_status`)**:
  ```
  === EV Battery Management System Status ===
  Pack Voltage   : 22.14 V  | Current: 12.00 A
  Avg Temperature: 26.2 C   | Avg SoC: 84.8 %
  Active Faults  : 0        | Mode   : Passive
  Cell 0: 3.692 V | 25.8 C | 84.8 % | NORMAL
  Cell 1: 3.688 V | 26.1 C | 84.9 % | NORMAL
  Cell 2: 3.695 V | 26.4 C | 84.7 % | NORMAL
  ```
- **Terminal UI (`bms_dashboard`)**: ANSI-color progress bars, pack metrics, active alert banners.
- **Web UI (`dashboard/index.html`)**: Real-time canvas telemetry charts, 6-cell interactive cards, and one-click IOCTL injection buttons.

> **Speaker Script**:
> "Here are the live outputs from the system. In Linux, viewing /proc/bms_status lets technicians view pack health directly from the terminal without any special tooling. Concurrently, the terminal and web dashboards provide an intuitive, high-visibility operational overview for vehicle diagnostics."

---

### Slide 11: Summary & Technical Achievements
- **Linux Kernel Engineering**: Character device driver registration, atomic 200ms timer execution, spinlock IRQ synchronization, and `seq_file` procfs diagnostics.
- **Modern C++ Architecture**: C++17 templates, RAII resource management, Observer pattern, and seqlock lock-free shared memory.
- **Automotive Safety Algorithms**: Coulomb counting integration, thermal gradient runaway detection ($dT/dt$), and dissipative cell balancing.
- **Codebase Quality**: Modular architecture, zero external framework dependencies, clean Makefile and CMake build system, and full automated test scripts (`run.sh`).

> **Speaker Script**:
> "To summarize: this project integrates low-level Linux kernel development with robust modern C++ software design. It delivers an end-to-end Battery Management System that addresses real-world EV challenges with high-performance, deterministic software."

---

### Slide 12: Viva Questions & Defense Script

#### Q1: Why did you use a spinlock instead of a mutex in the kernel driver?
> *"The kernel timer runs in atomic softirq context, where scheduling a context switch or sleeping is strictly illegal. A mutex can put the calling thread to sleep if contended, which would cause a kernel panic. A spinlock with `spin_lock_irqsave` disables local interrupts and spins for the short duration needed to update our pack telemetry, making it completely safe for interrupt context."*

#### Q2: What are the limitations of Coulomb counting, and how do production systems solve them?
> *"Coulomb counting integrates current over time, so sensor offset and quantization errors accumulate as integration drift. Furthermore, it assumes constant Coulombic efficiency. In commercial automotive BMS, Coulomb counting is fused with an Open-Circuit Voltage (OCV) look-up table and Extended Kalman Filtering (EKF) whenever the vehicle rests."*

#### Q3: How does your Sequence Lock (seqlock) work in shared memory?
> *"The seqlock uses an atomic 32-bit counter. When the daemon begins writing telemetry, it increments the sequence to an odd number. After updating all fields, it increments it to an even number. The dashboard reader reads the counter before and after copying memory. If the counter was odd or changed during the read, the reader knows a write occurred midway and simply retries. This provides lock-free, zero-contention reads."*

#### Q4: What makes rate-of-rise ($dT/dt$) better than a simple temperature cutoff?
> *"By the time a cell reaches an absolute cutoff like 75°C, exothermic breakdown may already be self-sustaining. The rate-of-rise detects the inflection point where internal temperature accelerates at more than 5°C per minute. Catching this early allows the system to isolate the pack and activate cooling seconds before irreversible thermal propagation occurs."*
