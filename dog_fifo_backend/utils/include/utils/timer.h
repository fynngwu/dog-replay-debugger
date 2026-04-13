//
// Created by gx on 23-11-23.
// Extracted as standalone utility by wufy on 2026/4/12.
//

#ifndef TIMER_H
#define TIMER_H

#include <algorithm>
#include <chrono>
#include <deque>
#include <fstream>
#include <map>
#include <numeric>
#include <string>

#include <iostream>

/// Utility for timing function execution
class Timer {
   public:
    struct TimerRecord {
        TimerRecord() = default;
        TimerRecord(const std::string& name, double time_usage) {
            func_name_ = name;
            time_usage_in_ms_.emplace_back(time_usage);
        }
        std::string func_name_;
        std::deque<double> time_usage_in_ms_;
    };

    /**
     * Evaluate and record function execution time
     * @tparam F    Callable type
     * @param func  Function to execute
     * @param func_name  Name for recording
     * @param print Whether to print the timing immediately
     */
    template <class F>
    static void Evaluate(F&& func, const std::string& func_name, bool print = false) {
        auto t1 = std::chrono::steady_clock::now();
        std::forward<F>(func)();
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

        if (records_.find(func_name) != records_.end()) {
            records_[func_name].time_usage_in_ms_.emplace_back(time_used);
            while (records_[func_name].time_usage_in_ms_.size() > 2000) {
                records_[func_name].time_usage_in_ms_.pop_front();
            }
        } else {
            records_.insert({func_name, TimerRecord(func_name, time_used)});
        }

        if (print) {
            std::cout << "[INFO] func <" << func_name << "> timer: " << time_used << " ms" << std::endl;
        }
    }

    /// Print all recorded timing stats (average, median, 95th percentile)
    static void PrintAll();

    /// Dump timing records to CSV file for analysis
    static void DumpIntoFile(const std::string& file_name);

    /// Get the mean execution time of a specific function
    static double GetMeanTime(const std::string& func_name);

    /// Clear all records
    static void Clear() { records_.clear(); }

   private:
    inline static std::map<std::string, TimerRecord> records_;
};

// ---------------------------------------------------------------------------
// Inline / template implementations
// ---------------------------------------------------------------------------

inline void Timer::PrintAll() {
    std::cout << ">>> ===== Printing run time =====" << std::endl;
    for (auto& r : records_) {
        auto& rec = r.second.time_usage_in_ms_;
        std::sort(rec.begin(), rec.end());

        std::cout << "> [" << r.first
                  << "] average: "
                  << std::accumulate(rec.begin(), rec.end(), 0.0) /
                         static_cast<double>(rec.size())
                  << " ms, med: " << rec[rec.size() * 0.5]
                  << ", 95%: " << rec[rec.size() * 0.95]
                  << ", calls: " << rec.size() << std::endl;
    }
    std::cout << ">>> ===== Printing run time end =====" << std::endl;
}

inline void Timer::DumpIntoFile(const std::string& file_name) {
    std::ofstream ofs(file_name, std::ios::out);
    if (!ofs.is_open()) {
        std::cerr << "[ERROR] Failed to open file: " << file_name << std::endl;
        return;
    }

    std::cout << "[INFO] Dump Time Records into file: " << file_name << std::endl;

    size_t max_length = 0;
    for (const auto& iter : records_) {
        ofs << iter.first << ", ";
        if (iter.second.time_usage_in_ms_.size() > max_length) {
            max_length = iter.second.time_usage_in_ms_.size();
        }
    }
    ofs << std::endl;

    for (size_t i = 0; i < max_length; ++i) {
        for (const auto& iter : records_) {
            if (i < iter.second.time_usage_in_ms_.size()) {
                ofs << iter.second.time_usage_in_ms_[i] << ",";
            } else {
                ofs << ",";
            }
        }
        ofs << std::endl;
    }
    ofs.close();
}

inline double Timer::GetMeanTime(const std::string& func_name) {
    if (records_.find(func_name) == records_.end()) {
        return 0.0;
    }
    auto& r = records_[func_name];
    return std::accumulate(r.time_usage_in_ms_.begin(), r.time_usage_in_ms_.end(), 0.0) /
           static_cast<double>(r.time_usage_in_ms_.size());
}

#endif  // TIMER_H
