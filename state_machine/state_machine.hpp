#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <memory>
#include <thread>

#include "dog_driver.hpp"

#ifdef HAS_TENSORRT
#include "policy_runner.hpp"
#endif

namespace dog {

enum class Mode { INIT, EXECUTE, POLICY, STOP };

class StateMachine {
public:
    static constexpr int NUM_JOINTS = DogDriver::NUM_JOINTS;
    static constexpr float INIT_DURATION_SEC = 2.5f;
    static constexpr int INIT_INTERVAL_MS = 10;
    static constexpr int EXECUTE_INTERVAL_MS = 2;
    static constexpr float MAX_STEP_RAD = 0.02f;
    static constexpr float REACH_THRESHOLD = 0.001f;
    static constexpr size_t MAX_QUEUE_SIZE = 10;

    static constexpr std::array<float, NUM_JOINTS> kJointMin = {
        -0.7853982f, -0.7853982f, -0.7853982f, -0.7853982f,
        -1.2217658f, -1.2217305f, -0.8726999f, -0.8726999f,
        -1.0217299f, -1.0217299f, -0.6f,       -0.6f
    };

    static constexpr std::array<float, NUM_JOINTS> kJointMax = {
         0.7853982f,  0.7853982f,  0.7853982f,  0.7853982f,
         0.8726683f,  0.8726683f,  1.2217342f,  1.2217305f,
         0.6f,        0.6f,        1.0217287f,  1.0217287f
    };

    explicit StateMachine(DogDriver& driver);
    ~StateMachine();

    StateMachine(const StateMachine&) = delete;
    StateMachine& operator=(const StateMachine&) = delete;

    std::string RequestMode(Mode mode);
    std::string EnqueueTarget(const std::array<float, NUM_JOINTS>& joints);
    Mode GetCurrentMode() const;
    DogDriver::JointState GetJointStates() const;
    DogDriver::IMUData GetIMUData() const;

    void Run();
    void Stop();

private:
    static bool CanTransition(Mode from, Mode to);
    static std::string ModeToString(Mode mode);
    static std::string TransitionError(Mode from, Mode to);

    void ProcessInit();
    void ProcessPolicy();
    void ProcessStop();
    void ExecuteThreadFunc();

    DogDriver& driver_;
    mutable std::mutex mode_mutex_;
    mutable std::mutex queue_mutex_;
    std::condition_variable cv_mode_;
    std::condition_variable cv_queue_;
    Mode current_mode_ = Mode::INIT;
    bool mode_requested_ = false;
    std::atomic<bool> running_{true};
    std::queue<std::array<float, NUM_JOINTS>> target_queue_;
    std::thread execute_thread_;
#ifdef HAS_TENSORRT
    std::unique_ptr<PolicyRunner> policy_runner_;
#endif
};

}  // namespace dog
