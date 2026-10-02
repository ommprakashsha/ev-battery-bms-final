# System Architecture & UML Design Specifications

This document outlines the software design, data flow, and UML diagrams for the **EV Battery Health & Thermal Management System**.

---

## 1. System Architecture & Data Flow Diagram

```
+-------------------------------------------------------------------------------------------------+
|                                           USERSPACE                                             |
|                                                                                                 |
|   +----------------------------------------------------------------+    +--------------------+  |
|   |                          bms_daemon                            |    |   bms_dashboard    |  |
|   |                                                                |    | (Live Terminal UI) |  |
|   |   +-------------------+              +---------------------+   |    +--------------------+  |
|   |   |   SoCCalculator   |              |  MovingAverage<T,N> |   |              ^             |
|   |   | (Coulomb Counting)|              | (Sliding Window FIR)|   |              |             |
|   |   +-------------------+              +---------------------+   |              | Lock-free   |
|   |             ^                                   ^              |              | Seqlock     |
|   |             |                                   |              |              | Read (5 Hz) |
|   |             v                                   v              |              v             |
|   |   +-------------------+              +---------------------+   |    +--------------------+  |
|   |   |ThermalRunawayDet. |              |    CellBalancer     |   |    | POSIX Shared Memory|  |
|   |   | (Rate-of-Rise dT) |              | (Passive Dissipative|   |    |     (/bms_shm)     |  |
|   |   +---------+---------+              +----------+----------+   |    +--------------------+  |
|   |             |                                   |              |              ^             |
|   |             v                                   v              |              |             |
|   |   +---------+---------+              +----------+----------+   |              |             |
|   |   |    AlertSender    |              |      BmsLogger      |   |              |             |
|   |   |    (POSIX MQ)     |              |     (CSV Engine)    |───┼──────────────+             |
|   |   +---------+---------+              +----------+----------+   | (Atomic Seqlock Writes)    |
|   +-------------|-----------------------------------|--------------+                            |
|                 |                                   |                                           |
|                 v                                   v                                           |
|        +-----------------+                 +-----------------+                                  |
|        | POSIX Msg Queue |                 |    CSV Audit    |                                  |
|        |  (/bms_alerts)  |                 |(/var/log/bms/..) |                                 |
|        +-----------------+                 +-----------------+                                  |
|                 ^                                                                               |
+-----------------|-------------------------------------------------------------------------------+
|                 | IOCTL Commands: GET_PACK_DATA, INJECT_FAULT, CLEAR_FAULTS, etc.               |
|                 v                                                                               |
|        +-----------------+   <====== [tools/bms_ctl] (CLI Diagnostics & Fault Injection)        |
|        |  /dev/bms_drv   |                                                                      |
+--------+-----------------+----------------------------------------------------------------------+
|                                         KERNEL SPACE                                            |
|                                                                                                 |
|   +-----------------------------------------------------------------------------------------+   |
|   |                                     bms_driver.ko                                       |   |
|   |                                                                                         |   |
|   |   +---------------------------+   +-------------------------+   +-------------------+   |   |
|   |   |     Kernel Softirq Timer  |   |   IRQ-Safe Spinlock     |   |   /proc/bms_status|   |   |
|   |   |   (200ms Periodic Ticks)  |   |   (Concurrency Control) |   |   (seq_file API)  |   |   |
|   |   +---------------------------+   +-------------------------+   +-------------------+   |   |
|   |                 |                               ^                         ^             |   |
|   |                 v                               |                         |             |   |
|   |   +---------------------------------------------+-------------------------+---------+   |   |
|   |   |                     struct bms_pack_data (6 Li-ion Cells)                       |   |   |
|   |   +---------------------------------------------------------------------------------+   |   |
|   +-----------------------------------------------------------------------------------------+   |
+-------------------------------------------------------------------------------------------------+
```

---

## 2. UML Class Diagram

```
+-----------------------------+
|     <<Interface>>           |
|    ThermalObserver          |
+-----------------------------+
| + onThermalEvent(id, ev, T) |
+-----------------------------+
              ^
              | implements
+-------------+---------------------------+
|          ThermalRunawayDetector         |
+-----------------------------------------+
| - cell_states_: CellThermalState[6]     |
| - observers_: vector<ThermalObserver*>  |
+-----------------------------------------+
| + addObserver(obs: ThermalObserver*)    |
| + update(cell_id, temp_C, dt_s): Event  |
| + getRateOfRise(cell_id): float         |
| + isThermalRunaway(cell_id): bool       |
| - notifyObservers(id, event, temp)      |
+-----------------------------------------+

+-----------------------------------------+       +-----------------------------------------+
|       MovingAverage<T, N>               |       |             SoCCalculator               |
+-----------------------------------------+       +-----------------------------------------+
| - buffer_: std::array<T, N>             |       | - capacity_as_: float                   |
| - head_: size_t                         |       | - soc_pct_: float                       |
| - count_: size_t                        |       | - charge_used_as_: float                |
| - sum_: T                               |       +-----------------------------------------+
+-----------------------------------------+       | + update(current_A, dt_s): float        |
| + push(value: T): void                  |       | + reset(initial_soc_pct): void          |
| + average(): T                          |       | + getSoC(): float                       |
| + isFull(): bool                        |       +-----------------------------------------+
+-----------------------------------------+

+-----------------------------------------+       +-----------------------------------------+
|              CellBalancer               |       |                BmsLogger                |
+-----------------------------------------+       +-----------------------------------------+
| - num_cells_: size_t                    |       | - file_: std::ofstream                  |
| - balancing_active_: bool               |       | - mutex_: std::mutex                    |
| - balance_mask_: uint8_t                |       | - record_count_: size_t                 |
| - total_energy_dissipated_J_: float     |       +-----------------------------------------+
+-----------------------------------------+       | + logCells(cells, count): void          |
| + checkBalance(cells, count): uint8_t   |       | + logFault(cell_id, desc): void         |
| + applyBalance(cells, count, dt): void  |       | + close(): void                         |
| + getTotalEnergyDissipatedJoules(): flt |       +-----------------------------------------+
+-----------------------------------------+
```

---

## 3. UML Sequence Diagram: Telemetry Polling & Fault Detection

```
bms_dashboard         bms_daemon            /dev/bms_drv          /bms_alerts MQ        /bms_shm
     |                    |                       |                     |                  |
     |                    |-- IOCTL(GET_PACK) --->|                     |                  |
     |                    |<-- struct pack_data --|                     |                  |
     |                    |                                             |                  |
     |                    |-- 1. Push MovingAvg(V, T)                   |                  |
     |                    |-- 2. Update SoCCalculator(I, dt)            |                  |
     |                    |-- 3. Calculate dT/dt rate                   |                  |
     |                    |                                             |                  |
     |                    |-- [If dT/dt > 5 C/min & T > 65C]            |                  |
     |                    |    (Thermal Runaway Event)                  |                  |
     |                    |----------------- mq_send(CRITICAL) -------->|                  |
     |                    |                                             |                  |
     |                    |-- 4. Check Cell Balance (dV > 50mV)         |                  |
     |                    |                                             |                  |
     |                    |-- 5. Seqlock: seq = seq + 1 (ODD) ---------------------------->|
     |                    |-- 6. Write Telemetry Payload --------------------------------->|
     |                    |-- 7. Seqlock: seq = seq + 1 (EVEN) --------------------------->|
     |                    |                                             |                  |
     |<----------------- mq_receive(ALERT) -----------------------------|                  |
     |                                                                                     |
     |-- 8. Read seqlock: seq1 ----------------------------------------------------------->|
     |-- 9. Copy shared state ------------------------------------------------------------->|
     |-- 10. Read seqlock: seq2 (Check seq1 == seq2 && even) ------------------------------>|
     |-- 11. Render Dashboard UI                                                           |
```

---

## 4. State Machine Diagram: Cell Health & Safety Transitions

```
               [Startup / Cold]
                      |
                      v
              +---------------+
              |    NORMAL     | <--------------------+
              | (T <= 45 C)   |                      |
              +---------------+                      |
                      |                              |
               T > 45 C |                            | T cools down
                      v                              |
              +---------------+                      |
              |     WARM      | ---------------------+
              | (45 - 55 C)   |                      |
              +---------------+                      |
                      |                              |
               T > 55 C |                            |
                      v                              |
              +---------------+                      |
              |      HOT      | ---------------------+
              | (55 - 65 C)   |
              +---------------+
                      |
        T > 65 C AND  |
      dT/dt > 5 C/min |
                      v
              +---------------+
              |   CRITICAL    | =======> [SAFETY TRIP: OPEN CONTACTORS]
              | (Runaway Trip)|          Broadcast on /bms_alerts MQ
              +---------------+
                      |
       Voltage < 2.0V | Permanent failure
                      v
              +---------------+
              |     DEAD      |
              +---------------+
```

---

## 5. Cell Balancing State Machine

```
              +----------------------+
              |  Balancing Inactive  |
              +----------------------+
                         |
      V_max - V_min > 50 mV |
                         v
              +----------------------+
              |   Flag Shunt Cells   |
              |  Set balancing mask  |
              +----------------------+
                         |
                         v
              +----------------------+
              | Active Bleed Circuit |
              | Discharge at 500 mA  |
              | Track Joules heat    |
              +----------------------+
                         |
      V_cell <= V_min + 50 mV|
                         v
              +----------------------+
              | Equalized / Standby  |
              +----------------------+
```
