# Battery Management System - Algorithms & Mathematical Models

This document details the core algorithms and mathematical formulations implemented across the BMS kernel driver and userspace daemon.

---

## 1. State of Charge (SoC) Estimation - Coulomb Counting

### Mathematical Principle
State of Charge represents the available battery capacity expressed as a percentage of its rated capacity. Coulomb counting integrates the net current flowing into or out of the cell over time:

$$SoC(t) = SoC(0) - \frac{1}{Q_{nominal}} \int_{0}^{t} I(\tau) \, d\tau$$

Where:
- $SoC(t)$: State of charge at time $t$ [0.0% - 100.0%]
- $SoC(0)$: Initial state of charge at startup
- $Q_{nominal}$: Nominal rated capacity in Ampere-seconds ($1 \text{ Ah} = 3600 \text{ As}$)
- $I(\tau)$: Instantaneous current in Amperes (sign convention: $I > 0$ for discharge, $I < 0$ for charge)

### Discrete-Time Implementation
In `src/SoCCalculator.hpp`, forward Euler integration is computed at each monitoring interval $\Delta t$:

$$\Delta Q = I \times \Delta t$$
$$Q_{used}(k) = Q_{used}(k-1) + \Delta Q$$
$$SoC(k) = SoC(0) - \left( \frac{Q_{used}(k)}{Q_{nominal}} \right) \times 100\%$$

The resulting value is bounded within $[0.0, 100.0]\%$.

---

## 2. Thermal Runaway Detection & Rate-of-Rise ($dT/dt$)

### Mechanism
Thermal runaway in lithium-ion cells begins when internal heat generation exceeds dissipation capacity, triggering exothermic chemical breakdown. To catch this condition prior to cell combustion, temperature magnitude alone is insufficient—rapid rate-of-rise ($dT/dt$) indicates self-heating.

### Detection Criteria
Implemented in `src/ThermalRunawayDetector.hpp`:

$$\text{Critical Runaway Condition} = (T_{cell} > 65^\circ\text{C}) \quad \land \quad \left( \frac{dT}{dt} > 5^\circ\text{C}/\text{min} \right)$$

### Discrete Gradient Calculation
Using a backward difference formulation over sampling step $\Delta t$:

$$\frac{dT}{dt} = \frac{T(k) - T(k-1)}{\Delta t} \times 60 \quad [^\circ\text{C}/\text{min}]$$

### Thermal State Thresholds
| State | Temperature Range | Action Required |
|---|---|---|
| `NORMAL` | $T \le 45^\circ\text{C}$ | Standard operation |
| `WARM` | $45^\circ\text{C} < T \le 55^\circ\text{C}$ | Reduce maximum discharge rate |
| `HOT` | $55^\circ\text{C} < T \le 65^\circ\text{C}$ | Disable charging, alert driver |
| `CRITICAL` | $T > 65^\circ\text{C} \text{ and } dT/dt > 5^\circ\text{C}/\text{min}$ | Emergency contactor disconnect |

---

## 3. Passive Cell Balancing

### Purpose
In a series string of cells, variations in manufacturing tolerances and temperature cause state-of-charge drift. During charging, the cell with the highest voltage reaches the upper cutoff threshold first, limiting the pack's usable capacity.

### Balancing Logic
Implemented in `src/CellBalancer.hpp`:

1. Identify the minimum terminal voltage among non-dead cells:
   $$V_{min} = \min_{i} \{ V_i \mid \text{Cell } i \text{ is healthy} \}$$

2. Identify cells requiring balancing:
   $$\text{Balance Cell } i \iff V_i > V_{min} + \Delta V_{threshold}$$
   Where $\Delta V_{threshold} = 50\text{ mV}$ ($0.050\text{ V}$).

3. Apply dissipative bleed resistor across flagged cells:
   - Bleed current: $I_{bleed} = 500\text{ mA}$
   - Dissipated heat power: $P = V_i \times I_{bleed}$
   - Dissipated energy: $\Delta E = P \times \Delta t$ [Joules]

---

## 4. Digital Filtering - Moving Average Filter

### Purpose
ADC readings from battery monitoring ICs (and simulated kernel ticks) contain measurement noise. A finite impulse response (FIR) sliding window smoother reduces transient spikes without introducing dynamic memory allocations.

### Implementation
Implemented in `src/MovingAverage.hpp` as a circular ring buffer with an $O(1)$ running sum:

$$\bar{x}_k = \frac{1}{N} \sum_{i=0}^{N-1} x_{k-i}$$

Updating the running sum on sample arrival:
$$S_k = S_{k-1} - x_{k-N} + x_k$$
$$\bar{x}_k = \frac{S_k}{N}$$
