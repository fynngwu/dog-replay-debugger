#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "motor_config.hpp"
#include "robstride.hpp"
#include "utils/async_message_process.h"

namespace dog {

struct MitConfig {
    float kp = kDefaultKp;
    float kd = kDefaultKd;
    float vel_limit = kDefaultVelLimit;
    float torque_limit = kDefaultTorqueLimit;
};

struct MotorSnapshot {
    bool initialized = false;
    bool enabled = false;
    bool worker_started = false;
    bool has_active_target = false;
    bool init_in_progress = false;
    size_t queue_size = 0;
    std::vector<float> joint_positions;
    std::vector<float> joint_torques;
    std::vector<float> active_target_joint_positions;
    std::vector<float> last_sent_joint_positions;
    MitConfig mit_config;
    std::string last_error;
};

enum class MotorState {
    kUninitialized = 0,
    kDisabled = 1,
    kStand = 2,
    kRunning = 3,
};

class MotorIO {
public:
    MotorIO();
    ~MotorIO();

    bool Initialize(std::string& err);
    bool StartWorker(std::string& err);
    void StopWorker();

    // 阻塞式初始化平滑过渡到零点位置（不走 worker）
    bool SmoothInitToOffset(float duration_sec, int hz, std::string& err);
    
    bool DisableAllAndClear(std::string& err);
    bool EnqueueJointTarget(const std::vector<float>& target_joint, std::string& err, size_t& queue_size_after);
    bool SetMITConfig(float kp, float kd, float vel_limit, float torque_limit, std::string& err);
    bool SetZeroJoint(int joint_index, std::string& err);

    MotorSnapshot GetSnapshot() const;
    std::string StateJsonLine() const;

private:
    struct JointTargetMessage {
        uint64_t generation = 0;
        std::vector<float> target_joint;
    };

    bool EnableAll(std::string& err);
    void EnableAllAutoReport();
    bool SendJointTarget(const std::vector<float>& rel_joint_target, std::string& err);
    void ProcessTargetMessage(const JointTargetMessage& msg);

    // 阻塞式平滑过渡专用
    std::vector<float> ReadCurrentAbsolutePositions() const;
    bool SendAbsoluteTargets(const std::vector<float>& abs_targets, std::string& err);
    bool SmoothMoveAbsolute(const std::vector<float>& from_abs,
                            const std::vector<float>& to_abs,
                            float duration_sec,
                            int hz,
                            std::string& err);

    std::shared_ptr<RobstrideController> controller_;
    std::vector<std::shared_ptr<CANInterface>> can_ifaces_;
    std::vector<int> motor_indices_;
    std::unique_ptr<AsyncMessageProcess<JointTargetMessage>> target_processor_;

    mutable std::mutex state_mutex_;
    MotorState state_ = MotorState::kUninitialized;

    size_t queued_target_count_ = 0;
    uint64_t command_generation_ = 0;
    std::vector<float> active_target_joint_;
    std::vector<float> last_sent_joint_;
    MitConfig mit_config_;
    std::string last_error_;
};

}  // namespace dog
