#include "motor_io.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

#include "can_interface.hpp"
#include "protocol.hpp"

namespace dog {
namespace {

float ClampAbsoluteTargetRad(int joint_index, float abs_target) {
    const float lower = kJointOffsets[joint_index] + kJointMin[joint_index];
    const float upper = kJointOffsets[joint_index] + kJointMax[joint_index];
    return std::clamp(abs_target, lower, upper);
}

std::vector<float> ZeroJointVector() {
    return std::vector<float>(kNumMotors, 0.0f);
}

// Smoothstep 插值: h = t*t*(3 - 2*t)，比线性更柔和
float Smoothstep(float t) {
    return t * t * (3.0f - 2.0f * t);
}

bool IsEnabledState(MotorState state) {
    return state == MotorState::kStand || state == MotorState::kRunning;
}

}  // namespace

MotorIO::MotorIO()
    : controller_(std::make_shared<RobstrideController>()),
      motor_indices_(kNumMotors, -1),
      active_target_joint_(kNumMotors, 0.0f),
      last_sent_joint_(kNumMotors, 0.0f) {
    target_processor_ = std::make_unique<AsyncMessageProcess<JointTargetMessage>>(
        [this](const JointTargetMessage& msg) { ProcessTargetMessage(msg); },
        "motor_target_worker");
    target_processor_->SetMaxSize(static_cast<size_t>(kMaxQueueSize));
}

MotorIO::~MotorIO() {
    StopWorker();
}

bool MotorIO::Initialize(std::string& err) {
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (state_ != MotorState::kUninitialized) {
            return true;
        }

        can_ifaces_.clear();
        for (const char* name : kCanIfNames) {
            auto iface = std::make_shared<CANInterface>(name);
            if (!iface || iface->GetName() == nullptr) {
                err = std::string("failed to create CAN interface: ") + name;
                last_error_ = err;
                return false;
            }
            can_ifaces_.push_back(iface);
            if (controller_->BindCAN(iface) != 0) {
                err = std::string("BindCAN failed for ") + name;
                last_error_ = err;
                return false;
            }
        }

        for (int j = 0; j < kNumCanIfaces; ++j) {
            for (int i = 0; i < 3; ++i) {
                const int global_idx = j + i * 4;
                auto info = std::make_unique<RobstrideController::MotorInfo>();
                info->motor_id = kMotorIds[global_idx];
                info->host_id = 0xFD;
                info->max_torque = kDefaultTorqueLimit;
                info->max_speed = kDefaultVelLimit;
                info->max_kp = 500.0f;
                info->max_kd = 5.0f;
                const int idx = controller_->BindMotor(kCanIfNames[j], std::move(info));
                if (idx < 0) {
                    err = std::string("BindMotor failed for joint ") + kJointNames[global_idx];
                    last_error_ = err;
                    return false;
                }
                motor_indices_[global_idx] = idx;
            }
        }

        for (int idx : motor_indices_) {
            MIT_params mit{};
            mit.kp = mit_config_.kp;
            mit.kd = mit_config_.kd;
            mit.vel_limit = mit_config_.vel_limit;
            mit.torque_limit = mit_config_.torque_limit;
            if (controller_->SetMITParams(idx, mit) != 0) {
                err = "SetMITParams failed during initialize";
                last_error_ = err;
                return false;
            }
        }

        queued_target_count_ = 0;
        active_target_joint_ = ZeroJointVector();
        target_processor_->SetProcFunc(
            [this](const JointTargetMessage& msg) { ProcessTargetMessage(msg); });
        target_processor_->SetName("motor_target_worker");
        target_processor_->SetMaxSize(static_cast<size_t>(kMaxQueueSize));
    }

    target_processor_->Start();

    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_ = MotorState::kDisabled;
        last_error_.clear();
    }
    return true;
}

bool MotorIO::StartWorker(std::string& err) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ == MotorState::kUninitialized) {
        err = "MotorIO not initialized";
        last_error_ = err;
        return false;
    }
    last_error_.clear();
    return true;
}

void MotorIO::StopWorker() {
    AsyncMessageProcess<JointTargetMessage>* proc = nullptr;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (state_ == MotorState::kUninitialized) {
            return;
        }
        ++command_generation_;
        state_ = MotorState::kDisabled;
        queued_target_count_ = 0;
        active_target_joint_ = ZeroJointVector();
        proc = target_processor_.get();
    }
    if (proc) {
        proc->Quit();
    }
}

bool MotorIO::EnableAll(std::string& err) {
    for (int i = 0; i < kNumMotors; ++i) {
        if (controller_->EnableMotor(motor_indices_[i]) != 0) {
            err = std::string("EnableMotor failed for ") + kJointNames[i];
            last_error_ = err;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

void MotorIO::EnableAllAutoReport() {
    for (int idx : motor_indices_) {
        controller_->EnableAutoReport(idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

bool MotorIO::SendJointTarget(const std::vector<float>& rel_joint_target, std::string& err) {
    if (rel_joint_target.size() != static_cast<size_t>(kNumMotors)) {
        err = "expected 12 joint targets";
        return false;
    }
    for (int i = 0; i < kNumMotors; ++i) {
        float motor_delta = rel_joint_target[i];
        if (i >= 8) {
            motor_delta *= kKneeRatio;
        }
        const float abs_target = ClampAbsoluteTargetRad(
            i, kJointDirection[i] * motor_delta + kJointOffsets[i]);
        if (controller_->SendMITCommand(motor_indices_[i], abs_target) != 0) {
            err = std::string("SendMITCommand failed for ") + kJointNames[i];
            return false;
        }
    }
    return true;
}

// ========== 阻塞式初始化专用函数 ==========

std::vector<float> MotorIO::ReadCurrentAbsolutePositions() const {
    std::vector<float> abs_positions(kNumMotors, 0.0f);
    for (int i = 0; i < kNumMotors; ++i) {
        const motor_state st = controller_->GetMotorState(motor_indices_[i]);
        abs_positions[i] = st.position;
    }
    return abs_positions;
}

bool MotorIO::SendAbsoluteTargets(const std::vector<float>& abs_targets, std::string& err) {
    if (abs_targets.size() != static_cast<size_t>(kNumMotors)) {
        err = "expected 12 absolute targets";
        last_error_ = err;
        return false;
    }
    for (int i = 0; i < kNumMotors; ++i) {
        const float clamped = ClampAbsoluteTargetRad(i, abs_targets[i]);
        if (controller_->SendMITCommand(motor_indices_[i], clamped) != 0) {
            err = std::string("SendMITCommand failed for ") + kJointNames[i];
            last_error_ = err;
            return false;
        }
    }
    return true;
}

bool MotorIO::SmoothMoveAbsolute(const std::vector<float>& from_abs,
                                 const std::vector<float>& to_abs,
                                 float duration_sec,
                                 int hz,
                                 std::string& err) {
    const int total_steps = static_cast<int>(duration_sec * hz);
    if (total_steps <= 0) {
        err = "invalid duration or hz";
        last_error_ = err;
        return false;
    }

    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(1000000 / hz);

    for (int step = 0; step <= total_steps; ++step) {
        const float t = static_cast<float>(step) / total_steps;
        const float h = Smoothstep(t);

        std::vector<float> interp_abs(kNumMotors);
        for (int i = 0; i < kNumMotors; ++i) {
            interp_abs[i] = from_abs[i] + h * (to_abs[i] - from_abs[i]);
        }

        if (!SendAbsoluteTargets(interp_abs, err)) {
            return false;
        }

        // 精确定时，保证平滑过渡
        const auto next_tick = clock::now() + period;
        std::this_thread::sleep_until(next_tick);
    }

    return true;
}

bool MotorIO::SmoothInitToOffset(float duration_sec, int hz, std::string& err) {
    // 1) 仅允许 disabled -> stand
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (state_ == MotorState::kUninitialized) {
            err = "MotorIO not initialized";
            last_error_ = err;
            return false;
        }
        if (state_ != MotorState::kDisabled) {
            err = "init only allowed in disabled state";
            last_error_ = err;
            return false;
        }
        ++command_generation_;
        queued_target_count_ = 0;
        active_target_joint_ = ZeroJointVector();
    }

    // 3) EnableAll()
    if (!EnableAll(err)) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_ = MotorState::kDisabled;
        return false;
    }

    // 4) EnableAllAutoReport()
    EnableAllAutoReport();

    // 5) 等待反馈稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 6) 读取当前绝对位置
    const std::vector<float> from_abs = ReadCurrentAbsolutePositions();

    // 7) 目标位置 = offsets (零点对应的绝对位置)
    std::vector<float> to_abs(kNumMotors);
    for (int i = 0; i < kNumMotors; ++i) {
        to_abs[i] = kJointOffsets[i];
    }

    // 8) 阻塞式平滑过渡
    if (!SmoothMoveAbsolute(from_abs, to_abs, duration_sec, hz, err)) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_ = MotorState::kDisabled;
        return false;
    }

    // 9) 发送相对 joint 全零 hold，保持当前位置
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (!SendJointTarget(ZeroJointVector(), err)) {
            last_error_ = err;
            state_ = MotorState::kDisabled;
            return false;
        }
        // 同步 last_sent_joint_ = zeros
        last_sent_joint_ = ZeroJointVector();
        state_ = MotorState::kStand;
        last_error_.clear();
    }

    return true;
}

// ========== 运行期队列执行 ==========

bool MotorIO::DisableAllAndClear(std::string& err) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ == MotorState::kUninitialized) {
        err = "MotorIO not initialized";
        last_error_ = err;
        return false;
    }
    ++command_generation_;
    
    queued_target_count_ = 0;
    active_target_joint_ = ZeroJointVector();
    for (int idx : motor_indices_) {
        if (controller_->DisableMotor(idx) != 0) {
            err = "DisableMotor failed";
            last_error_ = err;
            state_ = MotorState::kDisabled;
            return false;
        }
    }
    state_ = MotorState::kDisabled;
    last_error_.clear();
    return true;
}

bool MotorIO::EnqueueJointTarget(const std::vector<float>& target_joint, std::string& err, size_t& queue_size_after) {
    JointTargetMessage msg;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (state_ == MotorState::kUninitialized) {
            err = "MotorIO not initialized";
            last_error_ = err;
            queue_size_after = 0;
            return false;
        }
        if (state_ == MotorState::kDisabled) {
            err = "state is disabled, run init first";
            last_error_ = err;
            queue_size_after = 0;
            return false;
        }
        if (target_joint.size() != static_cast<size_t>(kNumMotors)) {
            err = "expected 12 joint targets";
            last_error_ = err;
            queue_size_after = queued_target_count_;
            return false;
        }
        if (queued_target_count_ >= static_cast<size_t>(kMaxQueueSize)) {
            err = "queue_full";
            last_error_ = err;
            queue_size_after = queued_target_count_;
            return false;
        }

        msg.generation = command_generation_;
        msg.target_joint = target_joint;
        if (state_ == MotorState::kStand) {
            state_ = MotorState::kRunning;
        }
        ++queued_target_count_;
        queue_size_after = queued_target_count_;
        last_error_.clear();
    }
    target_processor_->AddMessage(msg);
    return true;
}

bool MotorIO::SetMITConfig(float kp, float kd, float vel_limit, float torque_limit, std::string& err) {
    if (kp < 0.0f || kd < 0.0f || vel_limit <= 0.0f || torque_limit <= 0.0f) {
        err = "invalid MIT params";
        std::lock_guard<std::mutex> lk(state_mutex_);
        last_error_ = err;
        return false;
    }

    std::lock_guard<std::mutex> lk(state_mutex_);
    mit_config_.kp = kp;
    mit_config_.kd = kd;
    mit_config_.vel_limit = vel_limit;
    mit_config_.torque_limit = torque_limit;
    for (int idx : motor_indices_) {
        MIT_params mit{};
        mit.kp = mit_config_.kp;
        mit.kd = mit_config_.kd;
        mit.vel_limit = mit_config_.vel_limit;
        mit.torque_limit = mit_config_.torque_limit;
        if (controller_->SetMITParams(idx, mit) != 0) {
            err = "SetMITParams failed";
            last_error_ = err;
            return false;
        }
    }
    last_error_.clear();
    return true;
}

bool MotorIO::SetZeroJoint(int joint_index, std::string& err) {
    if (joint_index < 0 || joint_index >= kNumMotors) {
        err = "joint index out of range";
        std::lock_guard<std::mutex> lk(state_mutex_);
        last_error_ = err;
        return false;
    }

    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ == MotorState::kUninitialized) {
        err = "MotorIO not initialized";
        last_error_ = err;
        return false;
    }
    if (state_ != MotorState::kStand) {
        err = "setzero only allowed in stand state";
        last_error_ = err;
        return false;
    }
    if (queued_target_count_ > 0) {
        err = "queue not empty, cannot setzero";
        last_error_ = err;
        return false;
    }
    
    ++command_generation_;
    queued_target_count_ = 0;
    active_target_joint_ = ZeroJointVector();
    if (controller_->SetZero(motor_indices_[joint_index]) != 0) {
        err = std::string("SetZero failed for ") + kJointNames[joint_index];
        last_error_ = err;
        return false;
    }
    last_error_.clear();
    return true;
}

MotorSnapshot MotorIO::GetSnapshot() const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    MotorSnapshot snap;
    snap.initialized = state_ != MotorState::kUninitialized;
    snap.enabled = IsEnabledState(state_);
    snap.worker_started = state_ != MotorState::kUninitialized;
    snap.has_active_target = state_ == MotorState::kRunning;
    snap.init_in_progress = false;
    snap.queue_size = queued_target_count_;
    snap.active_target_joint_positions = active_target_joint_;
    snap.last_sent_joint_positions = last_sent_joint_;
    snap.mit_config = mit_config_;
    snap.last_error = last_error_;
    snap.joint_positions.resize(kNumMotors, 0.0f);
    snap.joint_torques.resize(kNumMotors, 0.0f);
    for (int i = 0; i < kNumMotors; ++i) {
        const motor_state st = controller_->GetMotorState(motor_indices_[i]);
        float joint_pos = kJointDirection[i] * (st.position - kJointOffsets[i]);
        if (i >= 8) {
            joint_pos /= kKneeRatio;
        }
        snap.joint_positions[i] = joint_pos;
        snap.joint_torques[i] = st.torque;
    }
    return snap;
}

std::string MotorIO::StateJsonLine() const {
    const MotorSnapshot snap = GetSnapshot();
    std::ostringstream oss;
    oss << '{'
        << "\"ok\":true,"
        << "\"enabled\":" << (snap.enabled ? "true" : "false") << ','
        << "\"worker_started\":" << (snap.worker_started ? "true" : "false") << ','
        << "\"busy\":" << (snap.has_active_target ? "true" : "false") << ','
        << "\"init_in_progress\":" << (snap.init_in_progress ? "true" : "false") << ','
        << "\"queue_size\":" << snap.queue_size << ','
        << "\"kp\":" << snap.mit_config.kp << ','
        << "\"kd\":" << snap.mit_config.kd << ','
        << "\"joint_positions\":" << JsonArray(snap.joint_positions) << ','
        << "\"joint_torques\":" << JsonArray(snap.joint_torques) << ','
        << "\"target_joint_positions\":" << JsonArray(snap.active_target_joint_positions) << ','
        << "\"last_sent_joint_positions\":" << JsonArray(snap.last_sent_joint_positions) << ','
        << "\"last_error\":\"" << JsonEscape(snap.last_error) << "\""
        << "}\n";
    return oss.str();
}

void MotorIO::ProcessTargetMessage(const JointTargetMessage& msg) {
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (msg.generation == command_generation_ && queued_target_count_ > 0) {
            --queued_target_count_;
        }
        if (state_ == MotorState::kUninitialized) {
            return;
        }
        if (msg.generation != command_generation_) {
            return;
        }
        if (state_ != MotorState::kRunning) {
            return;
        }
        active_target_joint_ = msg.target_joint;
        for (int i = 0; i < kNumMotors; ++i) {
            const motor_state st = controller_->GetMotorState(motor_indices_[i]);
            float joint_pos = kJointDirection[i] * (st.position - kJointOffsets[i]);
            if (i >= 8) {
                joint_pos /= kKneeRatio;
            }
            last_sent_joint_[i] = joint_pos;
        }
    }

    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();

    while (true) {
        std::vector<float> next_joint;
        bool reached_before_send = true;

        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            if (state_ != MotorState::kRunning ||
                msg.generation != command_generation_) {
                return;
            }

            next_joint = last_sent_joint_;
            for (int i = 0; i < kNumMotors; ++i) {
                const float diff = active_target_joint_[i] - last_sent_joint_[i];
                if (std::abs(diff) >= kReachEpsRad) {
                    reached_before_send = false;
                }
                const float step = std::clamp(diff, -kQueueStepRad, kQueueStepRad);
                next_joint[i] = last_sent_joint_[i] + step;
            }
        }

        if (reached_before_send) {
            return;
        }

        std::string send_err;
        if (!SendJointTarget(next_joint, send_err)) {
            std::lock_guard<std::mutex> lk(state_mutex_);
            active_target_joint_ = ZeroJointVector();
            last_error_ = send_err;
            return;
        }

        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            if (state_ != MotorState::kRunning ||
                msg.generation != command_generation_) {
                return;
            }
            last_error_.clear();
            last_sent_joint_ = next_joint;
            bool finished = true;
            for (int i = 0; i < kNumMotors; ++i) {
                if (std::abs(active_target_joint_[i] - last_sent_joint_[i]) >= kReachEpsRad) {
                    finished = false;
                    break;
                }
            }
            if (finished) {
                return;
            }
        }

        next_tick += std::chrono::milliseconds(kWorkerPeriodMs);
        std::this_thread::sleep_until(next_tick);
        if (clock::now() - next_tick > std::chrono::milliseconds(20)) {
            next_tick = clock::now();
        }
    }
}

}  // namespace dog
