#pragma once

#include <array>
#include <numeric>
#include <cstddef>

/*
 * MovingAverage template
 * 
 * Implements a simple fixed-size circular buffer moving average filter.
 * Used for smoothing cell voltage and temperature readings to filter out ADC noise.
 * 
 * Template arguments:
 *   T: Arithmetic type (float, double, int)
 *   N: Window size (number of samples to average)
 */
template<typename T, std::size_t N>
class MovingAverage {
    static_assert(N > 0, "Window size N must be greater than zero");

public:
    MovingAverage() = default;

    // Push a new sample into the circular buffer
    void push(T value) {
        // Subtract the oldest value being replaced and add the new one
        sum_ -= buffer_[head_];
        buffer_[head_] = value;
        sum_ += value;

        head_ = (head_ + 1) % N;
        if (count_ < N) {
            count_++;
        }
    }

    // Returns the current moving average of the samples collected so far
    T average() const {
        if (count_ == 0) {
            return T{0};
        }
        return sum_ / static_cast<T>(count_);
    }

    // Returns the most recently inserted sample
    T latest() const {
        if (count_ == 0) {
            return T{0};
        }
        std::size_t idx = (head_ == 0) ? (N - 1) : (head_ - 1);
        return buffer_[idx];
    }

    // Check if the window is fully populated with N samples
    bool isFull() const {
        return count_ == N;
    }

    // Current number of samples in buffer
    std::size_t size() const {
        return count_;
    }

    // Window capacity
    constexpr std::size_t capacity() const {
        return N;
    }

    // Reset buffer state
    void reset() {
        buffer_.fill(T{0});
        head_ = 0;
        count_ = 0;
        sum_ = T{0};
    }

private:
    std::array<T, N> buffer_{};
    std::size_t      head_  = 0;
    std::size_t      count_ = 0;
    T                sum_   = T{0};
};
