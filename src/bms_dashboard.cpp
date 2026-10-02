/*
 * bms_dashboard.cpp - Real-Time Terminal Dashboard
 *
 * Independent monitoring process:
 *   - Attaches to POSIX shared memory (/bms_shm) created by bms_daemon
 *   - Reads battery pack state using a lock-free sequence lock (seqlock)
 *   - Subscribes to the POSIX message queue (/bms_alerts) for real-time fault warnings
 *   - Renders a formatted terminal UI showing cell voltages, temperatures, SoC, and alerts
 */

#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <vector>
#include <deque>
#include <string>
#include <mutex>

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <mqueue.h>

static constexpr const char* SHM_NAME   = "/bms_shm";
static constexpr const char* MQ_ALERTS  = "/bms_alerts";
static constexpr int MAX_CELLS          = 6;
static constexpr int MAX_ALERT_LINES    = 8;

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

// ANSI color escape codes
#define CLEAR_SCREEN  "\033[2J\033[H"
#define BOLD          "\033[1m"
#define RESET         "\033[0m"
#define RED           "\033[31m"
#define BRIGHT_RED    "\033[91m"
#define GREEN         "\033[32m"
#define BRIGHT_GREEN  "\033[92m"
#define YELLOW        "\033[33m"
#define CYAN          "\033[36m"
#define BRIGHT_CYAN   "\033[96m"
#define WHITE         "\033[37m"
#define DIM           "\033[2m"
#define BG_RED        "\033[41m"

static std::string progressBar(float fraction, int width = 20, const char* color = GREEN) {
    fraction = std::max(0.0f, std::min(1.0f, fraction));
    int filled = static_cast<int>(fraction * width);
    std::string bar = "[";
    bar += color;
    for (int i = 0; i < filled; ++i) bar += "#";
    bar += RESET;
    for (int i = filled; i < width; ++i) bar += "-";
    bar += "]";
    return bar;
}

static const char* stateStr(int s) {
    switch (s) {
        case 0:  return "NORMAL  ";
        case 1:  return "WARM    ";
        case 2:  return "HOT     ";
        case 3:  return "CRITICAL";
        case 4:  return "DEAD    ";
        default: return "UNKNOWN ";
    }
}

static const char* stateColor(int s) {
    switch (s) {
        case 0:  return BRIGHT_GREEN;
        case 1:  return YELLOW;
        case 2:  return "\033[33m";
        case 3:  return BRIGHT_RED;
        case 4:  return RED;
        default: return WHITE;
    }
}

static const char* voltColor(float v) {
    if (v > 4.15f || v < 2.90f) return BRIGHT_RED;
    if (v > 4.10f || v < 3.10f) return YELLOW;
    return BRIGHT_GREEN;
}

static const char* tempColor(float t) {
    if (t > 60.0f || t < -5.0f) return BRIGHT_RED;
    if (t > 45.0f || t < 5.0f)  return YELLOW;
    return BRIGHT_GREEN;
}

static std::atomic<bool>       g_running{true};
static std::deque<std::string> g_alerts;
static std::mutex              g_alert_mutex;

static void alertReceiverThread() {
    mqd_t mq = mq_open(MQ_ALERTS, O_RDONLY | O_NONBLOCK);
    if (mq == (mqd_t)-1) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        mq = mq_open(MQ_ALERTS, O_RDONLY | O_NONBLOCK);
        if (mq == (mqd_t)-1) return;
    }

    char buf[256];
    while (g_running) {
        ssize_t bytes = mq_receive(mq, buf, sizeof(buf), nullptr);
        if (bytes > 0) {
            buf[bytes] = '\0';
            std::lock_guard<std::mutex> lock(g_alert_mutex);
            g_alerts.push_back(std::string(buf));
            while (g_alerts.size() > MAX_ALERT_LINES) {
                g_alerts.pop_front();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    mq_close(mq);
}

static void signal_handler(int) {
    g_running.store(false);
}

static void renderDashboard(const SharedPackState* state, uint64_t frame) {
    SharedPackState snap{};
    uint32_t seq1, seq2;

    // Seqlock read loop: re-read if writer incremented sequence midway
    do {
        seq1 = state->sequence.load(std::memory_order_acquire);
        if (seq1 & 1) {
            std::this_thread::yield();
            continue;
        }
        memcpy(&snap.cell_voltages,     &state->cell_voltages,     sizeof(snap.cell_voltages));
        memcpy(&snap.cell_temperatures, &state->cell_temperatures, sizeof(snap.cell_temperatures));
        memcpy(&snap.cell_soc,          &state->cell_soc,          sizeof(snap.cell_soc));
        memcpy(&snap.cell_states,       &state->cell_states,       sizeof(snap.cell_states));
        memcpy(&snap.cell_faults,       &state->cell_faults,       sizeof(snap.cell_faults));
        snap.pack_voltage_V           = state->pack_voltage_V;
        snap.pack_current_A           = state->pack_current_A;
        snap.avg_temp_C               = state->avg_temp_C;
        snap.active_fault_count       = state->active_fault_count;
        snap.sample_count             = state->sample_count;
        snap.thermal_runaway_detected = state->thermal_runaway_detected;
        snap.runaway_cell_id          = state->runaway_cell_id;
        seq2 = state->sequence.load(std::memory_order_acquire);
    } while (seq1 != seq2 || (seq1 & 1));

    std::ostringstream out;
    out << CLEAR_SCREEN;

    out << BOLD << CYAN
        << "================================================================\n"
        << "         EV Battery Management System - Status Monitor          \n"
        << "================================================================\n"
        << RESET;

    if (snap.thermal_runaway_detected) {
        out << BG_RED << BOLD
            << "  *** THERMAL RUNAWAY DETECTED ON CELL "
            << snap.runaway_cell_id
            << " - EMERGENCY DISCONNECT ***  "
            << RESET << "\n";
    }

    out << "\n" << BOLD << "  PACK TELEMETRY\n" << RESET;
    out << "  --------------------------------------------------------------\n";

    float avg_soc = 0.0f;
    for (int i = 0; i < MAX_CELLS; ++i) avg_soc += snap.cell_soc[i];
    avg_soc /= MAX_CELLS;

    out << std::fixed << std::setprecision(1);
    out << "  Pack Voltage : " << BRIGHT_CYAN
        << std::setw(6) << snap.pack_voltage_V << " V" << RESET
        << "    Pack Current: "
        << (snap.pack_current_A > 0 ? RED : GREEN)
        << std::setw(6) << snap.pack_current_A << " A" << RESET << "\n";

    out << "  Avg Temp     : " << tempColor(snap.avg_temp_C)
        << std::setw(6) << snap.avg_temp_C << " C" << RESET
        << "    Avg SoC     : "
        << (avg_soc < 20 ? RED : avg_soc < 40 ? YELLOW : BRIGHT_GREEN)
        << std::setw(6) << avg_soc << " %" << RESET << "\n";

    out << "  Active Faults: "
        << (snap.active_fault_count > 0 ? BRIGHT_RED : BRIGHT_GREEN)
        << std::setw(6) << (int)snap.active_fault_count << RESET
        << "    Sample Count: " << DIM << snap.sample_count << RESET << "\n";

    out << "\n" << BOLD
        << "  ID | Voltage | Temp  | SoC  | State    | SoC Indicator        | Faults\n"
        << RESET
        << "  ---+---------+-------+------+----------+----------------------+-------\n";

    for (int i = 0; i < MAX_CELLS; ++i) {
        float v   = snap.cell_voltages[i];
        float t   = snap.cell_temperatures[i];
        float soc = snap.cell_soc[i];
        int   st  = snap.cell_states[i];
        uint8_t ff = snap.cell_faults[i];

        std::string faults;
        if (ff & (1<<0)) faults += "OV ";
        if (ff & (1<<1)) faults += "UV ";
        if (ff & (1<<2)) faults += "OT ";
        if (ff & (1<<3)) faults += "UT ";
        if (ff & (1<<4)) faults += "OC ";
        if (faults.empty()) faults = "-  ";

        const char* bar_col = soc < 20 ? RED : soc < 40 ? YELLOW : GREEN;

        out << "  " << i << "  | "
            << voltColor(v)  << std::setw(5) << v   << " V" << RESET << " | "
            << tempColor(t)  << std::setw(4) << t   << " C" << RESET << " | "
            << std::setw(4)  << soc << "%" << " | "
            << stateColor(st) << stateStr(st)       << RESET << " | "
            << progressBar(soc / 100.0f, 20, bar_col)           << " | "
            << (ff ? RED : DIM) << faults << RESET << "\n";
    }

    out << "\n" << BOLD << "  SYSTEM ALERTS\n" << RESET
        << "  --------------------------------------------------------------\n";

    {
        std::lock_guard<std::mutex> lock(g_alert_mutex);
        if (g_alerts.empty()) {
            out << "  " << BRIGHT_GREEN << "Normal operation - No active alerts" << RESET << "\n";
        } else {
            for (const auto& alert : g_alerts) {
                bool critical = alert.find("CRITICAL") != std::string::npos ||
                                alert.find("RUNAWAY")  != std::string::npos;
                out << "  " << (critical ? BRIGHT_RED : YELLOW)
                    << "  [!] " << alert << RESET << "\n";
            }
        }
    }

    out << "\n" << DIM << "  Update Frame: " << frame
        << " | Press Ctrl+C to exit\n" << RESET;

    std::cout << out.str() << std::flush;
}

int main() {
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT,  signal_handler);

    int fd = shm_open(SHM_NAME, O_RDONLY, 0);
    if (fd == -1) {
        perror("[Dashboard] shm_open failed - make sure bms_daemon is running");
        return 1;
    }

    auto* state = static_cast<const SharedPackState*>(
        mmap(nullptr, sizeof(SharedPackState), PROT_READ, MAP_SHARED, fd, 0)
    );
    if (state == MAP_FAILED) {
        perror("[Dashboard] mmap failed");
        close(fd);
        return 1;
    }
    close(fd);

    std::thread alert_thread(alertReceiverThread);

    uint64_t frame = 0;
    while (g_running) {
        renderDashboard(state, ++frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    alert_thread.join();
    munmap(const_cast<SharedPackState*>(state), sizeof(SharedPackState));

    std::cout << "\n[Dashboard] Exited cleanly.\n";
    return 0;
}
