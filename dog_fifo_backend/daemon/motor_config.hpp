#pragma once

#include <array>
#include <string>

namespace dog {

constexpr int kNumMotors = 12;
constexpr int kNumCanIfaces = 4;
constexpr float kQueueStepRad = 0.02f;
constexpr float kReachEpsRad = 1e-4f;
constexpr int kWorkerPeriodMs = 2;
constexpr int kStatePeriodMs = 20;  // 50 Hz
constexpr int kMaxQueueSize = 100;

// MIT 控制参数
constexpr float kDefaultKp = 40.0f;
constexpr float kDefaultKd = 0.5f;
constexpr float kDefaultVelLimit = 44.0f;
constexpr float kDefaultTorqueLimit = 17.0f;

// 机械参数
constexpr float kKneeRatio = 1.667f;
constexpr float kActionScale = 0.25f;

// CAN 接口名称
inline constexpr std::array<const char*, kNumCanIfaces> kCanIfNames = {
    "can0", "can1", "can2", "can3"
};

// 电机 ID (12 个电机)
inline constexpr std::array<int, kNumMotors> kMotorIds = {
    1, 5, 9, 13,
    2, 6, 10, 14,
    3, 7, 11, 15
};

// 关节名称
inline constexpr std::array<const char*, kNumMotors> kJointNames = {
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee"
};

// 关节方向 (虚拟关节空间 -> 电机轴方向)
inline constexpr std::array<float, kNumMotors> kJointDirection = {
    -1.0f, -1.0f, -1.0f, -1.0f,
    -1.0f, -1.0f, -1.0f, -1.0f,
    +1.0f, +1.0f, +1.0f, +1.0f
};

// 关节零点偏移
constexpr float kHipAOffset = 0.37f;
constexpr float kHipFOffset = 0.13f;
constexpr float kKneeOffset = 1.06f * kKneeRatio;

inline constexpr std::array<float, kNumMotors> kJointOffsets = {
    kHipAOffset, -kHipAOffset, -kHipAOffset, kHipAOffset,
    kHipFOffset, kHipFOffset, -kHipFOffset, -kHipFOffset,
    kKneeOffset, kKneeOffset, -kKneeOffset, -kKneeOffset
};

// 关节限位 (基于 XML)
inline constexpr std::array<float, kNumMotors> kJointMin = {
    -0.7853982f, -0.7853982f, -0.7853982f, -0.7853982f,
    -1.2217658f, -1.2217305f, -0.8726999f, -0.8726999f,
    -1.0217299f * kKneeRatio, -1.0217299f * kKneeRatio, -0.6f, -0.6f
};

inline constexpr std::array<float, kNumMotors> kJointMax = {
    0.7853982f, 0.7853982f, 0.7853982f, 0.7853982f,
    0.8726683f, 0.8726683f, 1.2217342f, 1.2217305f,
    0.6f, 0.6f, 1.0217287f * kKneeRatio, 1.0217287f * kKneeRatio
};

}  // namespace dog
