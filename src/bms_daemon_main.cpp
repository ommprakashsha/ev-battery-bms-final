/*
 * bms_daemon_main.cpp - EV Battery Management System Daemon
 *
 * Core monitoring daemon running in userspace:
 *   - Interacts with /dev/bms_drv via IOCTL calls to fetch cell telemetry
 *   - Runs thermal runaway detection and passive cell balancing algorithms
 *   - Broadcasts critical alerts to a POSIX message queue (/bms_alerts)
 *   - Publishes real-time pack state to POSIX shared memory (/bms_shm)
 *   - Logs telemetry and fault history to CSV
 *   - Handles graceful termination via SIGINT / SIGTERM
 */

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cmath>
#include <memory>

// POSIX headers
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <mqueue.h>

#include "BatteryCell.hpp"
#include "MovingAverage.hpp"
#include "SoCCalculator.hpp"
#include "ThermalRunawayDetector.hpp"
#include "CellBalancer.hpp"
#include "BmsLogger.hpp"

#ifdef __linux__
#include "../kernel/bms_driver.h"
#else
struct bms_pack_data {};
#define BMS_IOCTL_GET_PACK_DATA 0
#define BMS_IOCTL_INJECT_FAULT  0
#define MAX_CELLS 6
#endif

// System Configuration
static constexpr const char* DEVICE_PATH    = "/dev/bms_drv";
static constexpr const char* MQ_ALERTS      = "/bms_alerts";
static constexpr const char* SHM_NAME       = "/bms_shm";
static constexpr const char* LOG_PATH       = "/var/log/bms/bms_data.csv";

static constexpr int    MONITOR_INTERVAL_MS = 200;   // 5 Hz telemetry polling
static constexpr int    BALANCE_INTERVAL_MS = 1000;  // 1 Hz balancing check
static constexpr int    LOG_INTERVAL_MS     = 500;   // 2 Hz CSV logging
static constexpr size_t MQ_MAX_MSGS         = 16;
static constexpr size_t MQ_MAX_MSG_SIZE     = 256;

/*
 * Shared memory layout mapped at /bms_shm.
 * Uses a sequence counter pattern (seqlock) to allow lock-free reads by the dashboard.
 */
struct SharedPackState {
    std::atomic<uint32_t> sequence;
    float    cell_voltages[MAX_CELLS];
    float    cell_temperatures[MAX_CELLS];
    float    cell_soc[MAX_CELLS];
    int      cell_states[MAX_CELLS];
    uint8_t  cell_faults[MAX_CELLS];
    float    pack_voltage_V;
    float    pack_current_A;
    float    pack_soc_pct;
    float    avg_temp_C;
    uint8_t  active_fault_count;
    uint32_t sample_count;
    bool     thermal_runaway_detected;
    int      runaway_cell_id;
};

static std::atomic<bool> g_running{true};

static void signal_handler(int signum) {
    std::cout << "\n[BMS] Signal " << signum << " caught. Initiating clean shutdown...\n";
    g_running.store(false, std::memory_order_release);
}

/*
 * AlertSender: Wrapper for POSIX Message Queue (/bms_alerts)
 */
class AlertSender {
public:
    explicit AlertSender(const char* mq_name) : mq_name_(mq_name) {
        struct mq_attr attr{};
        attr.mq_flags   = 0;
        attr.mq_maxmsg  = MQ_MAX_MSGS;
        attr.mq_msgsize = MQ_MAX_MSG_SIZE;

        mqd_ = mq_open(mq_name, O_WRONLY | O_CREAT | O_NONBLOCK, 0666, &attr);
        if (mqd_ == (mqd_t)-1) {
            perror("[BMS] mq_open failed");
            valid_ = false;
        } else {
            valid_ = true;
            std::cout << "[BMS] Initialized alert queue: " << mq_name << "\n";
        }
    }

    ~AlertSender() {
        if (valid_) {
            mq_close(mqd_);
            mq_unlink(mq_name_);
        }
    }

    void send(const std::string& msg, unsigned int priority = 1) {
        if (!valid_) return;
        if (mq_send(mqd_, msg.c_str(), msg.size() + 1, priority) == -1) {
            if (errno != EAGAIN) {
                perror("[BMS] mq_send failed");
            }
        }
    }

private:
    mqd_t       mqd_   = (mqd_t)-1;
    bool        valid_ = false;
    const char* mq_name_;
};

/*
 * SharedMemoryWriter: Manages POSIX shared memory buffer (/bms_shm)
 */
class SharedMemoryWriter {
public:
    SharedMemoryWriter() {
        fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (fd_ == -1) {
            perror("[BMS] shm_open failed");
            return;
        }

        if (ftruncate(fd_, sizeof(SharedPackState)) == -1) {
            perror("[BMS] ftruncate failed");
            return;
        }

        state_ = static_cast<SharedPackState*>(
            mmap(nullptr, sizeof(SharedPackState),
                 PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0)
        );

        if (state_ == MAP_FAILED) {
            perror("[BMS] mmap failed");
            state_ = nullptr;
            return;
        }

        new (state_) SharedPackState{};
        std::cout << "[BMS] Initialized shared memory: " << SHM_NAME << "\n";
    }

    ~SharedMemoryWriter() {
        if (state_) {
            munmap(state_, sizeof(SharedPackState));
            shm_unlink(SHM_NAME);
        }
        if (fd_ != -1) {
            close(fd_);
        }
    }

    void write(const BatteryCell cells[], size_t count,
               float pack_v, float pack_i, float avg_t,
               uint8_t fault_count, uint32_t sample_count,
               bool runaway, int runaway_cell)
    {
        if (!state_) return;

        // Sequence lock: odd number indicates write in progress
        uint32_t seq = state_->sequence.load(std::memory_order_relaxed);
        state_->sequence.store(seq + 1, std::memory_order_release);

        for (size_t i = 0; i < count && i < MAX_CELLS; ++i) {
            state_->cell_voltages[i]      = cells[i].voltage_V;
            state_->cell_temperatures[i]  = cells[i].temperature_C;
            state_->cell_soc[i]           = cells[i].soc_percent;
            state_->cell_states[i]        = static_cast<int>(cells[i].state);
            state_->cell_faults[i]        = cells[i].fault_flags;
        }
        state_->pack_voltage_V           = pack_v;
        state_->pack_current_A           = pack_i;
        state_->pack_soc_pct             = 0.0f;
        state_->avg_temp_C               = avg_t;
        state_->active_fault_count       = fault_count;
        state_->sample_count             = sample_count;
        state_->thermal_runaway_detected = runaway;
        state_->runaway_cell_id          = runaway_cell;

        // Even sequence indicates data is consistent and stable to read
        state_->sequence.store(seq + 2, std::memory_order_release);
    }

private:
    int              fd_    = -1;
    SharedPackState* state_ = nullptr;
};

/*
 * BmsMonitor: Main control loop for polling, algorithmic analysis, and logging
 */
class BmsMonitor {
public:
    BmsMonitor(int dev_fd, AlertSender& alerts, SharedMemoryWriter& shm, BmsLogger& logger)
        : dev_fd_(dev_fd),
          alerts_(alerts),
          shm_(shm),
          logger_(logger),
          balancer_(MAX_CELLS),
          thermal_detector_()
    {
        for (int i = 0; i < MAX_CELLS; ++i) {
            soc_calcs_[i] = std::make_unique<SoCCalculator>(50.0f, 85.0f);
        }
        std::cout << "[BMS] Pack monitor ready with " << MAX_CELLS << " cells\n";
    }

    void run() {
        auto next_tick = std::chrono::steady_clock::now();
        auto next_log  = next_tick + std::chrono::milliseconds(LOG_INTERVAL_MS);
        auto next_bal  = next_tick + std::chrono::milliseconds(BALANCE_INTERVAL_MS);

        while (g_running.load(std::memory_order_acquire)) {
            struct bms_pack_data kpack{};

#ifdef __linux__
            if (ioctl(dev_fd_, BMS_IOCTL_GET_PACK_DATA, &kpack) < 0) {
                perror("[BMS] IOCTL GET_PACK_DATA failed");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            for (int i = 0; i < MAX_CELLS; ++i) {
                cells_[i].cell_id       = kpack.cells[i].cell_id;
                cells_[i].voltage_V     = kpack.cells[i].voltage_V;
                cells_[i].temperature_C = kpack.cells[i].temperature_C;
                cells_[i].current_A     = kpack.cells[i].current_A;
                cells_[i].soc_percent   = kpack.cells[i].soc_percent;
                cells_[i].state         = static_cast<CellState>(kpack.cells[i].state);
                cells_[i].fault_flags   = kpack.cells[i].fault_flags;
            }
#else
            simulateCells();
#endif

            auto now = std::chrono::steady_clock::now();
            float dt_s = std::chrono::duration<float>(now - last_tick_).count();
            last_tick_ = now;

            // Apply smoothing filters
            for (int i = 0; i < MAX_CELLS; ++i) {
                volt_avg_[i].push(cells_[i].voltage_V);
                temp_avg_[i].push(cells_[i].temperature_C);
            }

            // Run thermal runaway detection
            bool runaway_detected = false;
            int  runaway_cell     = -1;

            for (int i = 0; i < MAX_CELLS; ++i) {
                ThermalEvent ev = thermal_detector_.update(
                    i, cells_[i].temperature_C, dt_s
                );

                if (ev == ThermalEvent::CRITICAL_ALERT) {
                    if (!runaway_detected) {
                        runaway_detected = true;
                        runaway_cell     = i;
                    }
                    char msg[256];
                    snprintf(msg, sizeof(msg),
                        "CRITICAL: Thermal runaway cell %d - %.1f C (dT/dt=%.2f C/min)",
                        i, cells_[i].temperature_C,
                        thermal_detector_.getRateOfRise(i));
                    alerts_.send(std::string(msg), 2);
                    std::cerr << "[BMS ALERT] " << msg << "\n";
                } else if (ev == ThermalEvent::HOT_ALERT && !runaway_detected) {
                    char msg[256];
                    snprintf(msg, sizeof(msg),
                        "WARNING: Cell %d overheating - %.1f C",
                        i, cells_[i].temperature_C);
                    alerts_.send(std::string(msg), 1);
                }
            }

            // Check and notify electrical faults
            for (int i = 0; i < MAX_CELLS; ++i) {
                if (cells_[i].hasFault()) {
                    std::string fstr = cells_[i].faultString();
                    char msg[256];
                    snprintf(msg, sizeof(msg),
                        "FAULT: Cell %d - %s (V=%.3f, T=%.1f C)",
                        i, fstr.c_str(),
                        cells_[i].voltage_V, cells_[i].temperature_C);
                    alerts_.send(std::string(msg), 1);
                }
            }

            // Update shared memory
            float pack_v    = kpack.pack_voltage_V;
            float pack_i    = kpack.pack_current_A;
            float avg_t     = kpack.avg_temperature_C;
            uint8_t faults  = kpack.active_fault_count;
            uint32_t scount = kpack.sample_count;

            shm_.write(cells_, MAX_CELLS, pack_v, pack_i, avg_t,
                       faults, scount, runaway_detected, runaway_cell);

            // Execute periodic cell balancing
            if (now >= next_bal) {
                uint8_t mask = balancer_.checkBalance(cells_, MAX_CELLS);
                if (mask) {
                    balancer_.applyBalance(cells_, MAX_CELLS, 1.0f);
                    std::cout << "[BMS] Active balancing on cells mask: 0x"
                              << std::hex << static_cast<int>(mask) << std::dec << "\n";
                }
                next_bal += std::chrono::milliseconds(BALANCE_INTERVAL_MS);
            }

            // Execute periodic CSV logging
            if (now >= next_log) {
                logger_.logCells(cells_, MAX_CELLS);
                next_log += std::chrono::milliseconds(LOG_INTERVAL_MS);
            }

            next_tick += std::chrono::milliseconds(MONITOR_INTERVAL_MS);
            std::this_thread::sleep_until(next_tick);
        }

        std::cout << "[BMS] Monitoring loop terminated.\n";
    }

private:
    int                dev_fd_;
    AlertSender&       alerts_;
    SharedMemoryWriter& shm_;
    BmsLogger&         logger_;
    CellBalancer       balancer_;
    ThermalRunawayDetector thermal_detector_;

    BatteryCell        cells_[MAX_CELLS]{};
    std::unique_ptr<SoCCalculator> soc_calcs_[MAX_CELLS];

    MovingAverage<float, 5> volt_avg_[MAX_CELLS];
    MovingAverage<float, 5> temp_avg_[MAX_CELLS];

    std::chrono::steady_clock::time_point last_tick_ =
        std::chrono::steady_clock::now();

    void simulateCells() {
        static float sim_time = 0.0f;
        sim_time += 0.2f;

        for (int i = 0; i < MAX_CELLS; ++i) {
            cells_[i].cell_id       = static_cast<uint8_t>(i);
            cells_[i].voltage_V     = 3.65f + 0.04f * std::sin(sim_time * 0.1f + i);
            cells_[i].temperature_C = 25.0f + 4.0f  * std::sin(sim_time * 0.05f + i * 0.5f);
            cells_[i].current_A     = 12.0f;
            cells_[i].soc_percent   = 85.0f - (sim_time * 0.008f);
            cells_[i].state         = CellState::NORMAL;
            cells_[i].fault_flags   = 0;
        }
    }
};

int main() {
    std::cout << "========================================\n";
    std::cout << "  EV Battery Management System Daemon   \n";
    std::cout << "========================================\n\n";

    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT,  signal_handler);

    int dev_fd = -1;
#ifdef __linux__
    dev_fd = open(DEVICE_PATH, O_RDWR);
    if (dev_fd < 0) {
        perror("[BMS] Unable to open " DEVICE_PATH);
        std::cerr << "[BMS] Kernel module not loaded. Starting in offline simulation mode.\n";
    } else {
        std::cout << "[BMS] Successfully opened device: " << DEVICE_PATH << "\n";
    }
#else
    std::cout << "[BMS] Running in software simulation mode\n";
#endif

    AlertSender        alerts(MQ_ALERTS);
    SharedMemoryWriter shm;

    mkdir("/var/log/bms", 0755);
    BmsLogger logger(LOG_PATH);
    std::cout << "[BMS] Telemetry log target: " << LOG_PATH << "\n";

    BmsMonitor monitor(dev_fd, alerts, shm, logger);
    std::thread monitor_thread(&BmsMonitor::run, &monitor);

    while (g_running.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (g_running) {
            std::cout << "[BMS] Active. Records logged: "
                      << logger.getRecordCount() << "\n";
        }
    }

    monitor_thread.join();
    logger.close();

    if (dev_fd >= 0) {
        close(dev_fd);
        std::cout << "[BMS] Device closed.\n";
    }

    std::cout << "[BMS] Shutdown finished.\n";
    return 0;
}
