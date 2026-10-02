#pragma once

#include "BatteryCell.hpp"

#include <string>
#include <fstream>
#include <mutex>
#include <cstddef>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

/*
 * BmsLogger
 *
 * Thread-safe CSV logger for battery cell telemetry and fault occurrences.
 * Uses std::mutex to protect file I/O operations across threads.
 */
class BmsLogger {
public:
    explicit BmsLogger(const std::string& log_path)
        : log_path_(log_path),
          record_count_(0)
    {
        // Check if file already exists and has content before opening in append mode
        bool need_header = true;
        {
            std::ifstream check(log_path, std::ios::ate | std::ios::binary);
            if (check.is_open() && check.tellg() > 0) {
                need_header = false;
            }
        }

        file_.open(log_path, std::ios::out | std::ios::app);
        if (file_.is_open() && need_header) {
            writeHeader();
        }
    }

    ~BmsLogger() {
        close();
    }

    // Disable copy semantics
    BmsLogger(const BmsLogger&) = delete;
    BmsLogger& operator=(const BmsLogger&) = delete;

    // Log telemetry rows for all cells
    void logCells(const BatteryCell cells[], std::size_t count) {
        if (!file_.is_open() || !cells || count == 0) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        std::string ts = currentTimestamp();

        for (std::size_t i = 0; i < count; ++i) {
            file_ << ts << ","
                  << record_count_ << ","
                  << static_cast<int>(cells[i].cell_id) << ","
                  << std::fixed << std::setprecision(3) << cells[i].voltage_V << ","
                  << std::fixed << std::setprecision(1) << cells[i].temperature_C << ","
                  << std::fixed << std::setprecision(2) << cells[i].current_A << ","
                  << std::fixed << std::setprecision(1) << cells[i].soc_percent << ","
                  << cells[i].stateString() << ","
                  << "0x" << std::hex << static_cast<int>(cells[i].fault_flags) << std::dec << ","
                  << cells[i].faultString() << "\n";
            record_count_++;
        }
        file_.flush();
    }

    // Log a specific fault event record
    void logFault(int cell_id, const std::string& fault_desc) {
        if (!file_.is_open()) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        file_ << "FAULT,"
              << currentTimestamp() << ","
              << cell_id << ","
              << "\"" << fault_desc << "\"\n";
        file_.flush();
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_.is_open()) {
            file_.flush();
            file_.close();
        }
    }

    std::size_t getRecordCount() const {
        return record_count_;
    }

private:
    std::string    log_path_;
    std::ofstream  file_;
    mutable std::mutex mutex_;
    std::size_t    record_count_;

    void writeHeader() {
        file_ << "timestamp,record_id,cell_id,voltage_V,temperature_C,current_A,soc_pct,state,fault_flags,fault_names\n";
        file_.flush();
    }

    std::string currentTimestamp() const {
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
#if defined(_WIN32)
        gmtime_s(&tm_buf, &t);
#else
        gmtime_r(&t, &tm_buf);
#endif
        std::ostringstream ss;
        ss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%SZ");
        return ss.str();
    }
};
