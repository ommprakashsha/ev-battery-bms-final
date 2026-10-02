#pragma once

#include <algorithm>

/*
 * SoCCalculator
 *
 * Implements Coulomb counting (current integration) to estimate
 * State of Charge (SoC):
 *
 *   SoC(t) = SoC(0) - (1 / Q_nominal) * integral(I(t) * dt)
 *
 * Sign convention:
 *   Current > 0 : Discharging (battery is providing power, SoC decreases)
 *   Current < 0 : Charging (battery is absorbing power, SoC increases)
 */
class SoCCalculator {
public:
    /*
     * capacity_ah: Rated cell capacity in Ampere-hours (e.g., 50.0 Ah)
     * initial_soc_pct: Initial State of Charge in percentage [0.0 - 100.0]
     */
    explicit SoCCalculator(float capacity_ah, float initial_soc_pct = 85.0f)
        : capacity_as_(capacity_ah * 3600.0f),
          soc_pct_(std::clamp(initial_soc_pct, 0.0f, 100.0f)),
          soc_initial_(soc_pct_),
          charge_used_as_(0.0f)
    {}

    /*
     * Integrate current over a time step dt_seconds.
     * current_A: Current in Amperes
     * dt_seconds: Time step in seconds
     * Returns: Updated SoC percentage
     */
    float update(float current_A, float dt_seconds) {
        // Integrate charge: delta_Q = I * dt (in Amp-seconds)
        charge_used_as_ += current_A * dt_seconds;

        // Calculate updated SoC and clamp to [0.0, 100.0]
        float delta_soc = (charge_used_as_ / capacity_as_) * 100.0f;
        soc_pct_ = std::clamp(soc_initial_ - delta_soc, 0.0f, 100.0f);

        return soc_pct_;
    }

    // Reset estimator with a known reference SoC (e.g. from OCV lookup)
    void reset(float initial_soc_pct = 85.0f) {
        soc_initial_    = std::clamp(initial_soc_pct, 0.0f, 100.0f);
        soc_pct_        = soc_initial_;
        charge_used_as_ = 0.0f;
    }

    float getSoC() const noexcept {
        return soc_pct_;
    }

    float getCapacityAmpSeconds() const noexcept {
        return capacity_as_;
    }

    float getChargeUsedAmpSeconds() const noexcept {
        return charge_used_as_;
    }

private:
    float capacity_as_;     // Rated capacity in Ampere-seconds (Ah * 3600)
    float soc_pct_;         // Current estimated SoC (0 - 100%)
    float soc_initial_;     // Starting SoC
    float charge_used_as_;  // Net accumulated charge in Amp-seconds
};
