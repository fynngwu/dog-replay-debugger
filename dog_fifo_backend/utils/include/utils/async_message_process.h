//
// Created by xiang on 2022/2/9.
// Extracted as standalone utility by wufy on 2026/4/12.
//

#ifndef ASYNC_MESSAGE_PROCESS_H
#define ASYNC_MESSAGE_PROCESS_H

#include <condition_variable>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>

using UL = std::unique_lock<std::mutex>;

/**
 * Async message processing class with internal thread and queue.
 * Callbacks are executed serially on a dedicated thread.
 *
 * Useful for decoupling modules — avoids scattered locks and condition variables.
 *
 * @tparam T  Message type
 *
 * NOTE: When skip_num=1, no frames are actually skipped.
 *       When skip_num=2, it processes one frame and skips one.
 */
template <typename T>
class AsyncMessageProcess {
   public:
    using ProcFunc = std::function<void(const T&)>;

    AsyncMessageProcess() = default;
    AsyncMessageProcess(ProcFunc proc_func, std::string name = "");
    ~AsyncMessageProcess() { Quit(); }

    /// Set the processing callback
    void SetProcFunc(ProcFunc proc_func) { custom_func_ = proc_func; }

    /// Set maximum queue length
    void SetMaxSize(size_t size) { max_size_ = size; }

    /// Start the processing thread
    void Start();

    /// Add a message to the queue
    void AddMessage(const T& msg);

    /// Stop the processing thread
    void Quit();

    /// Reset skip counter so the next message will be processed immediately
    void CleanSkipCnt();

    void SetName(std::string name) { name_ = std::move(name); }
    void SetSkipParam(bool enable_skip, int skip_num) {
        enable_skip_ = enable_skip;
        skip_num_ = skip_num;
    }

    AsyncMessageProcess(const AsyncMessageProcess&) = delete;
    void operator=(const AsyncMessageProcess&) = delete;

   private:
    void ProcLoop();

    std::thread proc_;
    std::mutex mutex_;
    std::condition_variable cv_msg_;
    std::deque<T> msg_buffer_;
    bool update_flag_ = false;
    bool exit_flag_ = false;
    size_t max_size_ = 100;
    std::string name_;

    bool enable_skip_ = false;
    int skip_num_ = 0;
    int skip_cnt_ = 0;

    ProcFunc custom_func_;
};

// ---------------------------------------------------------------------------
// Template implementations (header-only)
// ---------------------------------------------------------------------------

template <typename T>
AsyncMessageProcess<T>::AsyncMessageProcess(ProcFunc proc_func, std::string name) {
    custom_func_ = std::move(proc_func);
    name_ = name;
}

template <typename T>
void AsyncMessageProcess<T>::CleanSkipCnt() {
    UL lock(mutex_);
    skip_cnt_ = 0;
}

template <typename T>
void AsyncMessageProcess<T>::Start() {
    exit_flag_ = false;
    update_flag_ = false;
    proc_ = std::thread([this]() { ProcLoop(); });
}

template <typename T>
void AsyncMessageProcess<T>::ProcLoop() {
    while (!exit_flag_) {
        UL lock(mutex_);
        cv_msg_.wait(lock, [this]() { return update_flag_; });

        auto buffer = msg_buffer_;
        msg_buffer_.clear();
        update_flag_ = false;
        lock.unlock();

        for (const auto& msg : buffer) {
            custom_func_(msg);
        }
    }
}

template <typename T>
void AsyncMessageProcess<T>::AddMessage(const T& msg) {
    UL lock(mutex_);
    if (enable_skip_) {
        if (skip_cnt_ != 0) {
            skip_cnt_++;
            skip_cnt_ = skip_cnt_% skip_num_;
            return;
        }
        skip_cnt_++;
        skip_cnt_ = skip_cnt_ % skip_num_;
    }

    msg_buffer_.push_back(msg);
    while (msg_buffer_.size() > max_size_) {
        std::cerr << "[WARN] " << name_ << " exceeds max queue size: " << max_size_ << std::endl;
        msg_buffer_.pop_front();
    }

    update_flag_ = true;
    cv_msg_.notify_one();
}

template <typename T>
void AsyncMessageProcess<T>::Quit() {
    update_flag_ = true;
    exit_flag_ = true;
    cv_msg_.notify_one();

    if (proc_.joinable()) {
        proc_.join();
    }
}

#endif  // ASYNC_MESSAGE_PROCESS_H
