#ifdef HAS_TENSORRT
#include "policy_runner.hpp"
#include "state_machine.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace dog {

using Clock = std::chrono::steady_clock;
using Ms = std::chrono::milliseconds;

PolicyRunner::PolicyRunner(DogDriver& driver, const std::string& engine_path)
    : driver_(driver) {

    std::string path = engine_path.empty() ? "/home/ares/pure_cpp/policy.engine" : engine_path;
    try {
        inference_ = std::make_unique<InferenceEngine>(path, INPUT_DIM, ACTION_DIM);
    } catch (const std::exception& e) {
        std::cerr << "[policy] failed to load engine: " << e.what() << std::endl;
        return;
    }

    imu_ = std::make_shared<IMUComponent>("/dev/ttyCH341USB0");
    gamepad_ = std::make_shared<Gamepad>("/dev/input/js0");
    joint_comp_ = std::make_shared<JointComponent>(driver_);
    action_comp_ = std::make_shared<ActionComponent>(ACTION_DIM);
    command_comp_ = std::make_shared<CommandComponent>(3, gamepad_);

    obs_ = std::make_unique<RoboObs>(HISTORY_LENGTH);
    obs_->AddComponent(imu_);
    obs_->AddComponent(command_comp_);
    obs_->AddComponent(joint_comp_);
    obs_->AddComponent(action_comp_);

    ready_ = true;
    std::cout << "[policy] engine loaded: " << path << std::endl;
}

PolicyRunner::~PolicyRunner() = default;

bool PolicyRunner::IsReady() const {
    return ready_;
}

void PolicyRunner::Run(std::function<void(const std::array<float, DogDriver::NUM_JOINTS>&)> send_target,
                        std::function<PolicyMode()> get_current_mode) {
    if (!ready_) {
        std::cerr << "[policy] not ready, skipping" << std::endl;
        return;
    }

    std::vector<float> action_vec(ACTION_DIM, 0.0f);
    auto next = Clock::now() + Ms(POLICY_INTERVAL_MS);

    std::cout << "[policy] loop started" << std::endl;

    obs_->UpdateObs();
    auto first_obs = obs_->GetSingleObs();
    for (int i = 1; i < HISTORY_LENGTH; ++i) {
        obs_->history.push_back(first_obs);
    }

    while (get_current_mode() == PolicyMode::POLICY) {
        obs_->UpdateObs();
        auto obs_vec = obs_->GetWholeObs();

        try {
            inference_->infer(obs_vec, action_vec);
        } catch (const std::exception& e) {
            std::cerr << "[policy] inference error: " << e.what() << std::endl;
            std::this_thread::sleep_until(next);
            next += Ms(POLICY_INTERVAL_MS);
            continue;
        }

        action_comp_->SetAction(action_vec);

        std::array<float, DogDriver::NUM_JOINTS> target;
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
            target[i] = action_vec[i] * 0.25f;
        }

        send_target(target);

        std::this_thread::sleep_until(next);
        next += Ms(POLICY_INTERVAL_MS);
    }

    std::cout << "[policy] loop exited" << std::endl;
}

}  // namespace dog
#endif
