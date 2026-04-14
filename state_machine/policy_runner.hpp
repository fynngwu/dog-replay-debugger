#pragma once

#include <array>
#include <functional>
#include <memory>
#include <string>

#include "dog_driver.hpp"
#include "observations.hpp"

#ifdef HAS_TENSORRT
#include "tensorrt_inference.hpp"

namespace dog {

enum class PolicyMode { INIT, EXECUTE, POLICY, STOP };

class PolicyRunner {
public:
    static constexpr int HISTORY_LENGTH = 10;
    static constexpr int OBS_DIM = 45;
    static constexpr int ACTION_DIM = 12;
    static constexpr int INPUT_DIM = HISTORY_LENGTH * OBS_DIM;
    static constexpr int POLICY_INTERVAL_MS = 20;

    PolicyRunner(DogDriver& driver, const std::string& engine_path);
    ~PolicyRunner();

    bool IsReady() const;

    void Run(std::function<void(const std::array<float, DogDriver::NUM_JOINTS>&)> send_target,
             std::function<PolicyMode()> get_current_mode);

private:
    DogDriver& driver_;
    std::unique_ptr<InferenceEngine> inference_;
    std::shared_ptr<DriverIMUAdapter> imu_;
    std::shared_ptr<Gamepad> gamepad_;
    std::shared_ptr<JointComponent> joint_comp_;
    std::shared_ptr<ActionComponent> action_comp_;
    std::shared_ptr<CommandComponent> command_comp_;
    std::unique_ptr<RoboObs> obs_;
    bool ready_ = false;
};

}  // namespace dog
#endif
