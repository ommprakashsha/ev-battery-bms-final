#pragma once

#include <vector>
#include <cmath>

/*
 * Severity level of thermal conditions
 */
enum class ThermalEvent {
    NORMAL,          // Temperature <= 45 C
    WARM_ALERT,      // 45 C < T <= 55 C
    HOT_ALERT,       // 55 C < T <= 65 C
    CRITICAL_ALERT   // T > 65 C AND rate of temperature rise dT/dt > 5 C/min
};

/*
 * Observer interface for thermal event notifications
 */
class ThermalObserver {
public:
    virtual ~ThermalObserver() = default;
    virtual void onThermalEvent(int cell_id, ThermalEvent event, float temp_C) = 0;
};

/*
 * ThermalRunawayDetector
 *
 * Monitors individual cell temperatures and calculates rate-of-rise (dT/dt).
 * Uses the Observer pattern to dispatch alerts when threshold conditions are met.
 *
 * Thermal runaway rule:
 *   Trigger CRITICAL_ALERT when cell temperature exceeds 65 C AND
 *   rate of rise dT/dt exceeds 5 C per minute.
 */
class ThermalRunawayDetector : public ThermalObserver {
public:
    static constexpr int   MAX_CELLS          = 6;
    static constexpr float WARM_THRESHOLD     = 45.0f; // Celsius
    static constexpr float HOT_THRESHOLD      = 55.0f; // Celsius
    static constexpr float CRITICAL_THRESHOLD = 65.0f; // Celsius
    static constexpr float RUNAWAY_RATE       = 5.0f;  // Celsius per minute

    ThermalRunawayDetector() {
        for (int i = 0; i < MAX_CELLS; ++i) {
            cell_states_[i].prev_temp_C  = 25.0f;
            cell_states_[i].rate_per_min = 0.0f;
            cell_states_[i].current      = ThermalEvent::NORMAL;
            cell_states_[i].initialized  = false;
        }
    }

    // Register an observer to receive notifications
    void addObserver(ThermalObserver* obs) {
        if (obs) {
            observers_.push_back(obs);
        }
    }

    /*
     * Process new temperature reading for a given cell.
     * cell_id: Cell index (0 to MAX_CELLS - 1)
     * temp_C: Current temperature in Celsius
     * dt_seconds: Time elapsed since last reading
     */
    ThermalEvent update(int cell_id, float temp_C, float dt_seconds) {
        if (cell_id < 0 || cell_id >= MAX_CELLS) {
            return ThermalEvent::NORMAL;
        }

        auto& st = cell_states_[cell_id];

        // Compute temperature rate of rise: (T - T_prev) / dt * 60 (deg C/min)
        if (st.initialized && dt_seconds > 0.001f) {
            float delta_temp = temp_C - st.prev_temp_C;
            st.rate_per_min  = (delta_temp / dt_seconds) * 60.0f;
        } else {
            st.rate_per_min = 0.0f;
            st.initialized  = true;
        }
        st.prev_temp_C = temp_C;

        // Classify thermal condition
        ThermalEvent new_event = ThermalEvent::NORMAL;
        if (temp_C > CRITICAL_THRESHOLD && st.rate_per_min > RUNAWAY_RATE) {
            new_event = ThermalEvent::CRITICAL_ALERT;
        } else if (temp_C > HOT_THRESHOLD) {
            new_event = ThermalEvent::HOT_ALERT;
        } else if (temp_C > WARM_THRESHOLD) {
            new_event = ThermalEvent::WARM_ALERT;
        }

        // Notify observers on state transition or recurring critical alerts
        if (new_event != st.current || new_event == ThermalEvent::CRITICAL_ALERT) {
            st.current = new_event;
            notifyObservers(cell_id, new_event, temp_C);
        }

        return new_event;
    }

    float getRateOfRise(int cell_id) const {
        if (cell_id >= 0 && cell_id < MAX_CELLS) {
            return cell_states_[cell_id].rate_per_min;
        }
        return 0.0f;
    }

    bool isThermalRunaway(int cell_id) const {
        if (cell_id >= 0 && cell_id < MAX_CELLS) {
            return cell_states_[cell_id].current == ThermalEvent::CRITICAL_ALERT;
        }
        return false;
    }

    // Default observer hook
    void onThermalEvent(int /*cell_id*/, ThermalEvent /*event*/, float /*temp_C*/) override {
        // Can be overridden by subclasses if needed
    }

private:
    struct CellThermalState {
        float        prev_temp_C  = 25.0f;
        float        rate_per_min = 0.0f;
        ThermalEvent current      = ThermalEvent::NORMAL;
        bool         initialized  = false;
    };

    CellThermalState              cell_states_[MAX_CELLS];
    std::vector<ThermalObserver*> observers_;

    void notifyObservers(int cell_id, ThermalEvent event, float temp_C) {
        for (auto* obs : observers_) {
            if (obs) {
                obs->onThermalEvent(cell_id, event, temp_C);
            }
        }
    }
};
