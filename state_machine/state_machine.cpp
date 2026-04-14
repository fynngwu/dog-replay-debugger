#include "state_machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace dog {

using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

StateMachine::StateMachine(DogDriver& driver) : driver_(driver) {
#ifdef HAS_TENSORRT
    policy_runner_ = std::make_unique<PolicyRunner>(driver_, "");
#endif
}

StateMachine::~StateMachine() { Stop(); }

bool StateMachine::CanTransition(Mode from, Mode to) {
    if (from == to) return true;
    if (from == Mode::INIT && to == Mode::EXECUTE) return true;
    if (from == Mode::INIT && to == Mode::POLICY) return true;
    if (from == Mode::EXECUTE && to == Mode::STOP) return true;
    if (from == Mode::POLICY && to == Mode::STOP) return true;
    if (from == Mode::STOP && to == Mode::INIT) return true;
    return false;
}

std::string StateMachine::ModeToString(Mode mode) {
    switch (mode) {
        case Mode::INIT: return "INIT";
        case Mode::EXECUTE: return "EXECUTE";
        case Mode::POLICY: return "POLICY";
        case Mode::STOP: return "STOP";
    }
    return "UNKNOWN";
}

std::string StateMachine::TransitionError(Mode from, Mode to) {
    std::string expected;
    if (from == Mode::INIT) expected = "EXECUTE or POLICY";
    else if (from == Mode::EXECUTE) expected = "STOP";
    else if (from == Mode::POLICY) expected = "STOP";
    else expected = "INIT";
    return "invalid transition: " + ModeToString(from) + " -> " + ModeToString(to) +
           ", expected: " + ModeToString(from) + " -> " + expected;
}

std::string StateMachine::RequestMode(Mode mode) {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (!CanTransition(current_mode_, mode)) {
            return TransitionError(current_mode_, mode);
        }
        current_mode_ = mode;
        mode_requested_ = true;
    }
    cv_mode_.notify_one();
    return "";
}

std::string StateMachine::EnqueueTarget(const std::array<float, NUM_JOINTS>& joints) {
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (current_mode_ != Mode::EXECUTE && current_mode_ != Mode::POLICY) {
            return "cannot enqueue target in " + ModeToString(current_mode_) + " mode";
        }
    }

    std::array<float, NUM_JOINTS> clamped = joints;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        clamped[i] = std::clamp(joints[i], kJointMin[i], kJointMax[i]);
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (target_queue_.size() >= MAX_QUEUE_SIZE) {
            return "queue full (max " + std::to_string(MAX_QUEUE_SIZE) + ")";
        }
        target_queue_.push(clamped);
    }

    cv_queue_.notify_one();
    return "";
}

Mode StateMachine::GetCurrentMode() const {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    return current_mode_;
}

DogDriver::JointState StateMachine::GetJointStates() const {
    return driver_.GetJointStates();
}

DogDriver::IMUData StateMachine::GetIMUData() const {
    return driver_.GetIMUData();
}

void StateMachine::ProcessInit() {
    std::cout << "[state] INIT: enabling motors..." << std::endl;
    driver_.EnableAll();

    for (int i = 0; i < NUM_JOINTS; ++i) {
        driver_.EnableAutoReport(i);
    }

    std::this_thread::sleep_for(Ms(10));

    auto current = driver_.GetJointStates().position;

    std::cout << "[state] INIT: interpolating to zero over "
              << INIT_DURATION_SEC << "s..." << std::endl;

    const int num_steps = static_cast<int>(INIT_DURATION_SEC * 1000 / INIT_INTERVAL_MS);
    auto next = Clock::now() + Ms(INIT_INTERVAL_MS);

    for (int step = 1; step <= num_steps; ++step) {
        float t = static_cast<float>(step) / num_steps;
        std::array<float, NUM_JOINTS> target;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            target[i] = current[i] * (1.0f - t);
        }
        driver_.SetAllJointPositions(target);
        std::this_thread::sleep_until(next);
        next += Ms(INIT_INTERVAL_MS);
    }

    driver_.SetAllJointPositions(std::array<float, NUM_JOINTS>{});
    std::cout << "[state] INIT: complete, waiting for command..." << std::endl;
}

void StateMachine::ProcessPolicy() {
    std::cout << "[state] POLICY: starting..." << std::endl;
#ifdef HAS_TENSORRT
    if (!policy_runner_->IsReady()) {
        std::cerr << "[state] POLICY: engine not loaded" << std::endl;
        return;
    }
    policy_runner_->Run(
        [this](const std::array<float, NUM_JOINTS>& target) {
            std::string err = EnqueueTarget(target);
            if (!err.empty()) {
                std::cerr << "[policy] enqueue failed: " << err << std::endl;
            }
        },
        [this]() { return static_cast<PolicyMode>(GetCurrentMode()); }
    );
#else
    std::cerr << "[state] POLICY: not available (built without TensorRT)" << std::endl;
#endif
    std::cout << "[state] POLICY: done" << std::endl;
}

void StateMachine::ProcessStop() {
    std::cout << "[state] STOP: disabling all motors..." << std::endl;
    driver_.DisableAll();
    std::cout << "[state] STOP: complete, waiting for INIT..." << std::endl;
}

void StateMachine::ExecuteThreadFunc() {
    auto last_sent = driver_.GetJointStates().position;

    while (running_) {
        std::array<float, NUM_JOINTS> target;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_queue_.wait(lock, [this] { return !target_queue_.empty() || !running_; });
            if (!running_) break;
            target = target_queue_.front();
            target_queue_.pop();
        }

        while (running_) {
            bool reached = true;
            std::array<float, NUM_JOINTS> cmd = last_sent;
            for (int i = 0; i < NUM_JOINTS; ++i) {
                float diff = target[i] - cmd[i];
                if (std::abs(diff) > REACH_THRESHOLD) {
                    reached = false;
                    cmd[i] += std::clamp(diff, -MAX_STEP_RAD, MAX_STEP_RAD);
                }
            }
            driver_.SetAllJointPositions(cmd);
            last_sent = cmd;
            if (reached) break;
            std::this_thread::sleep_for(Ms(EXECUTE_INTERVAL_MS));
        }
    }

    std::cout << "[execute thread] done" << std::endl;
}

void StateMachine::Run() {
    execute_thread_ = std::thread(&StateMachine::ExecuteThreadFunc, this);

    ProcessInit();

    while (running_) {
        std::unique_lock<std::mutex> lock(mode_mutex_);
        cv_mode_.wait(lock, [this] { return mode_requested_ || !running_; });
        if (!running_) break;
        mode_requested_ = false;
        Mode mode = current_mode_;
        lock.unlock();

        switch (mode) {
            case Mode::INIT: ProcessInit(); break;
            case Mode::EXECUTE: break;
            case Mode::POLICY: ProcessPolicy(); break;
            case Mode::STOP: ProcessStop(); break;
        }
    }

    if (execute_thread_.joinable()) {
        cv_queue_.notify_all();
        execute_thread_.join();
    }
}

void StateMachine::Stop() {
    running_ = false;
    cv_mode_.notify_all();
    cv_queue_.notify_all();
}

}  // namespace dog
