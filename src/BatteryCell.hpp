#pragma once

#include <cstdint>
#include <string>

/*
 * Cell health and temperature states.
 * Mirrors the kernel-side CellState enum in kernel/bms_driver.h
 */
enum class CellState : int {
    NORMAL   = 0,   // Normal operating temperature (<= 45 C)
    WARM     = 1,   // Elevated temperature (45 - 55 C)
    HOT      = 2,   // High temperature (55 - 65 C)
    CRITICAL = 3,   // Critical runaway threshold (> 65 C)
    DEAD     = 4    // Cell failure or voltage permanently out of bounds
};

/*
 * Data structure representing a single battery cell's telemetry.
 * Populated from kernel driver IOCTL responses.
 *
 * Fault flags bitmask layout:
 *   Bit 0 - Overvoltage  (V > 4.2V)
 *   Bit 1 - Undervoltage (V < 2.5V)
 *   Bit 2 - Overtemp     (T > 65C)
 *   Bit 3 - Undertemp    (T < -20C)
 *   Bit 4 - Overcurrent  (|I| > 30A)
 */
struct BatteryCell {
    uint8_t   cell_id;        // Cell index (0 to 5)
    float     voltage_V;      // Terminal voltage [2.5V - 4.2V]
    float     temperature_C;  // Cell temperature in Celsius
    float     current_A;      // Current in Amperes (positive = discharge)
    float     soc_percent;    // State of Charge [0.0% - 100.0%]
    CellState state;          // State evaluated from telemetry
    uint8_t   fault_flags;    // Active fault bitmask

    // Helper checks for individual fault conditions
    bool isOvervoltage()  const noexcept { return (fault_flags & (1u << 0u)) != 0u; }
    bool isUndervoltage() const noexcept { return (fault_flags & (1u << 1u)) != 0u; }
    bool isOvertemp()     const noexcept { return (fault_flags & (1u << 2u)) != 0u; }
    bool isUndertemp()    const noexcept { return (fault_flags & (1u << 3u)) != 0u; }
    bool isOvercurrent()  const noexcept { return (fault_flags & (1u << 4u)) != 0u; }
    bool hasFault()       const noexcept { return fault_flags != 0u; }

    // Returns a readable string for the current state
    std::string stateString() const {
        switch (state) {
            case CellState::NORMAL:   return "NORMAL";
            case CellState::WARM:     return "WARM";
            case CellState::HOT:      return "HOT";
            case CellState::CRITICAL: return "CRITICAL";
            case CellState::DEAD:     return "DEAD";
            default:                  return "UNKNOWN";
        }
    }

    // Returns comma-separated names of active faults
    std::string faultString() const {
        if (!hasFault()) {
            return "OK";
        }
        std::string s;
        if (isOvervoltage())  s += (s.empty() ? "" : ", ") + std::string("OVERVOLTAGE");
        if (isUndervoltage()) s += (s.empty() ? "" : ", ") + std::string("UNDERVOLTAGE");
        if (isOvertemp())     s += (s.empty() ? "" : ", ") + std::string("OVERTEMP");
        if (isUndertemp())    s += (s.empty() ? "" : ", ") + std::string("UNDERTEMP");
        if (isOvercurrent())  s += (s.empty() ? "" : ", ") + std::string("OVERCURRENT");
        return s;
    }
};
