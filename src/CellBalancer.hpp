#pragma once

#include "BatteryCell.hpp"
#include <cstddef>
#include <cstdint>
#include <limits>

/*
 * CellBalancer
 *
 * Implements passive (resistive bleed) cell balancing.
 *
 * In a series-connected pack, manufacturing variations cause differences in cell
 * capacity and internal resistance, leading to state-of-charge divergence.
 *
 * Balancing logic:
 *   1. Determine the minimum voltage (V_min) among all healthy cells.
 *   2. If a cell has V_cell > V_min + BALANCE_THRESHOLD_V (default: 50mV),
 *      activate the bleed resistor circuit for that cell.
 *   3. Bleed resistor draws a fixed current (I_bleed), dissipating excess energy as heat:
 *        Power = V_cell * I_bleed
 *        Delta_E = Power * dt
 */
class CellBalancer {
public:
    static constexpr float BALANCE_THRESHOLD_V = 0.050f; // 50 mV threshold
    static constexpr float BLEED_CURRENT_A     = 0.500f; // 500 mA bleed current

    explicit CellBalancer(std::size_t num_cells = 6)
        : num_cells_(num_cells),
          balancing_active_(false),
          balance_mask_(0),
          total_energy_dissipated_J_(0.0f)
    {}

    /*
     * Check which cells need balancing.
     * Returns a bitmask where bit i = 1 indicates cell i requires balancing.
     */
    uint8_t checkBalance(const BatteryCell cells[], std::size_t count) {
        if (!cells || count == 0) {
            balancing_active_ = false;
            balance_mask_     = 0;
            return 0;
        }

        // Find the minimum voltage among non-dead cells
        float min_v = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < count && i < num_cells_; ++i) {
            if (cells[i].state != CellState::DEAD && cells[i].voltage_V < min_v) {
                min_v = cells[i].voltage_V;
            }
        }

        if (min_v == std::numeric_limits<float>::max()) {
            balancing_active_ = false;
            balance_mask_     = 0;
            return 0;
        }

        // Compare each cell against V_min + threshold
        uint8_t mask = 0;
        for (std::size_t i = 0; i < count && i < num_cells_ && i < 8; ++i) {
            if (cells[i].state != CellState::DEAD &&
                cells[i].voltage_V > (min_v + BALANCE_THRESHOLD_V)) {
                mask |= static_cast<uint8_t>(1u << i);
            }
        }

        balance_mask_     = mask;
        balancing_active_ = (mask != 0);
        return mask;
    }

    /*
     * Apply balancing step over dt_s seconds.
     * Simulates voltage reduction and tracks dissipated energy.
     */
    void applyBalance(BatteryCell cells[], std::size_t count, float dt_s) {
        if (!cells || count == 0 || dt_s <= 0.0f) {
            return;
        }

        for (std::size_t i = 0; i < count && i < num_cells_ && i < 8; ++i) {
            if (balance_mask_ & (1u << i)) {
                // Approximate voltage reduction: dV = (I_bleed * dt) / C_eff
                // Assuming an effective cell capacitance for small voltage drops
                constexpr float C_eff = 500.0f; // Equivalent Farads
                float dv = (BLEED_CURRENT_A * dt_s) / C_eff;
                cells[i].voltage_V -= dv;

                // Accumulate dissipated energy in Joules: E = V * I * dt
                float power_w = cells[i].voltage_V * BLEED_CURRENT_A;
                total_energy_dissipated_J_ += power_w * dt_s;
            }
        }
    }

    bool isBalancing() const noexcept {
        return balancing_active_;
    }

    uint8_t getBalanceMask() const noexcept {
        return balance_mask_;
    }

    float getTotalEnergyDissipatedJoules() const noexcept {
        return total_energy_dissipated_J_;
    }

    void resetEnergyCounter() noexcept {
        total_energy_dissipated_J_ = 0.0f;
    }

private:
    std::size_t num_cells_;
    bool        balancing_active_;
    uint8_t     balance_mask_;
    float       total_energy_dissipated_J_;
};
