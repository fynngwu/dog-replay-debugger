# Dog 机器人驱动库设计稿

## 目标

将当前 `dog_fifo_backend` 从“面向 daemon 的后端程序”重组为“面向单台机器狗的可复用 C++17 驱动库”，对外只暴露两个高层对象：

- `dog::MotorDriver`
- `dog::ImuDriver`

调用者应当只需要阅读公开头文件，就能理解如何：

- 使能/失能电机
- 打开电机自动上报
- 发送 joint-space 关节目标
- 读取 joint-space 关节状态
- 对单关节做零点标定
- 读取 IMU gyro / quaternion / projected gravity

## 背景与现状

当前 `dog_fifo_backend` 主要由三部分组成：

1. **电机 CAN 传输与协议层**
   - `driver/include/can_interface.hpp`
   - `driver/include/robstride.hpp`
   - `driver/src/can_interface.cpp`
   - `driver/src/robstride.cpp`
2. **IMU 串口与 WIT 协议层**
   - `driver/include/serial.h`
   - `driver/src/serial.c`
   - `driver/src/wit_c_sdk.c`
   - `driver/src/observations.cpp`
3. **机器人关节语义与 daemon 行为层**
   - `daemon/motor_config.hpp`
   - `daemon/motor_io.hpp`
   - `daemon/motor_io.cpp`
   - `daemon/tcp_server.cpp`
   - `daemon/main.cpp`

其中与本次库化最相关的事实如下：

- `daemon/motor_io.cpp` 已经承担了“关节空间 → 电机命令”和“电机反馈 → 关节观测”的机器人语义转换。
- `driver/src/robstride.cpp` 和 `driver/src/can_interface.cpp` 是稳定的底层电机协议/通信实现。
- `driver/src/observations.cpp` 已包含 gyro 单位换算和 projected gravity 计算逻辑。
- 当前 `CMakeLists.txt` 只产出 daemon 可执行文件，还没有独立库目标。

## 范围

### 本次设计要做的事

- 将项目重组为 **一个库目标 + 两个公开类**。
- 将电机接口固定为面向这台机器狗的 **joint-space API**。
- 将 IMU 接口固定为面向这台机器狗控制栈需要的观测 API。
- 为电机和 IMU 分别提供真机 CLI 测试程序。
- 在公开头文件中写清楚 joint 定义、使用顺序和语义边界。

### 本次设计明确不做的事

- 不做通用多机器人配置系统。
- 不保留 daemon 的 TCP 协议作为公开库 API。
- 不保留 `MotorIO` 中的 queue / worker / active target 语义。
- 不引入 mock/fake 硬件抽象层作为主测试路径。
- 不把实现塞进 header，不做 header-only 库。

## 总体方案

采用“**薄的机器人专用驱动库**”方案：

- 对外只有两个高层类：`dog::MotorDriver` 与 `dog::ImuDriver`
- 对内继续复用现有底层驱动：CAN、Robstride、Serial、WIT SDK
- 将当前 `motor_io` 里的机器人 joint 语义转换保留下来，但删去 daemon 专属的 worker/queue/state JSON 行为
- 将机器人常量（CAN 名称、motor id、joint 顺序、方向、offset、limit、knee ratio）作为单机器人固定配置保留在库内部共享配置头中

目标是让外部调用者始终站在“这台狗的关节和 IMU”视角工作，而不是直接接触底层电机轴、CAN 帧或 WIT SDK 全局状态。

## 公开 API 设计

### MotorDriver

公开头文件路径：`include/dog/motor_driver.hpp`

建议接口：

```cpp
#pragma once

#include <array>
#include <string>

namespace dog {

constexpr int kNumJoints = 12;

enum class JointId {
    LF_HipA = 0,
    LR_HipA = 1,
    RF_HipA = 2,
    RR_HipA = 3,
    LF_HipF = 4,
    LR_HipF = 5,
    RF_HipF = 6,
    RR_HipF = 7,
    LF_Knee = 8,
    LR_Knee = 9,
    RF_Knee = 10,
    RR_Knee = 11,
};

struct JointState {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    float torque_nm = 0.0f;
};

struct MitConfig {
    float kp = 40.0f;
    float kd = 0.5f;
    float vel_limit = 44.0f;
    float torque_limit = 17.0f;
};

class MotorDriver {
public:
    MotorDriver();
    ~MotorDriver();

    bool Initialize(std::string& err);

    bool EnableAll(std::string& err);
    bool DisableAll(std::string& err);
    bool EnableAutoReport(std::string& err);

    bool SetMitConfig(const MitConfig& config, std::string& err);

    bool SetJointPositions(const std::array<float, kNumJoints>& joint_pos_rad,
                           std::string& err);
    bool SetJointPosition(JointId joint,
                          float joint_pos_rad,
                          std::string& err);

    bool SetZero(JointId joint, std::string& err);

    JointState GetJointState(JointId joint) const;
    std::array<JointState, kNumJoints> GetAllJointStates() const;

    const char* GetJointName(JointId joint) const;
};

}  // namespace dog
```

#### MotorDriver 语义

- `Initialize()` 只做底层资源准备：创建 CAN、绑定电机、写入默认 MIT 参数；**不触发运动**。
- `EnableAll()` / `DisableAll()` 负责整机电机使能状态切换。
- `EnableAutoReport()` 打开电机侧自动反馈，使读取状态更稳定。
- `SetJointPositions()` 以 **12 维 joint-space rad** 为输入，立即发送当前整机关节目标。
- `SetJointPosition()` 是单关节便捷接口，内部基于最近一次目标构造整机 12 维命令并立即发送。
- `SetZero()` 调用底层电机零点标定，不等价于“把关节运动到 0 rad”。
- `GetJointState()` / `GetAllJointStates()` 返回关节空间下的当前位置、速度和底层控制器上报的 torque estimate。

#### MotorDriver 必须在头文件中写清楚的注释

1. **关节命名与物理含义**
   - `LF/LR/RF/RR` 分别表示左前、左后、右前、右后腿
   - `HipA` 表示外展/内收关节
   - `HipF` 表示前后摆动关节
   - `Knee` 表示膝关节
2. **这是 joint-space API**
   - 输入输出单位是机器人关节空间，不是电机轴空间
   - 库内部自动处理方向、offset、knee ratio 与限位夹紧
3. **推荐调用顺序**
   - Construct → `Initialize()` → `EnableAll()` → `EnableAutoReport()` → `SetJoint...()` / `GetJointState()` → `DisableAll()`
4. **`SetZero()` 的风险语义**
   - 这是零点标定操作，只应在机械姿态正确时使用

### ImuDriver

公开头文件路径：`include/dog/imu_driver.hpp`

建议接口：

```cpp
#pragma once

#include <array>
#include <string>

namespace dog {

struct ImuData {
    std::array<float, 4> quaternion_wxyz{};
    std::array<float, 3> gyro_rad_s{};
    std::array<float, 3> acc_raw{};
    std::array<float, 3> projected_gravity{};
};

class ImuDriver {
public:
    explicit ImuDriver(const char* device = "/dev/ttyUSB0");
    ~ImuDriver();

    bool Initialize(std::string& err);
    bool Update(std::string& err);

    ImuData GetData() const;
};

}  // namespace dog
```

#### ImuDriver 语义

- `Initialize()` 完成串口打开、WIT SDK 初始化、回调注册和传感器输出配置。
- `Update()` 从串口读取数据并驱动内部解析器更新缓存。
- `GetData()` 返回最近一次成功解析的观测结果。
- `quaternion_wxyz` 顺序与当前实现一致，即 `[w, x, y, z]`。
- `gyro_rad_s` 使用弧度每秒。
- `acc_raw` 直接暴露当前传感器解析出的原始加速度三轴值；本次设计不额外承诺它已经换算到标准 SI 单位。
- `projected_gravity` 保持与当前控制栈一致的坐标约定。

## 内部实现设计

### 共享配置

新增共享配置头，例如：`config/robot_config.hpp`

该头文件承载当前 `daemon/motor_config.hpp` 中与这台机器人强绑定的常量：

- CAN 接口名
- 电机 ID
- 关节顺序
- 关节方向
- 关节 offset
- 关节限位
- `kKneeRatio`
- 默认 MIT 参数

这份配置由：

- `include/dog/motor_driver.hpp` 用于公开 joint 枚举和注释对齐
- `src/motor_driver.cpp` 用于实际换算和下发

从而确保“头文件描述”和“底层实现行为”引用同一份机器人定义。

### MotorDriver 内部职责

`src/motor_driver.cpp` 内部复用：

- `driver/include/can_interface.hpp`
- `driver/include/robstride.hpp`
- 共享机器人配置头

建议内部成员：

- `std::shared_ptr<RobstrideController> controller_`
- `std::vector<std::shared_ptr<CANInterface>> can_ifaces_`
- `std::vector<int> motor_indices_`
- `MitConfig mit_config_`
- `std::array<float, 12> last_commanded_joint_positions_`

这里保留 `last_commanded_joint_positions_` 的原因是：

- `SetJointPosition()` 需要在不暴露 queue/worker 的前提下，构造“当前整机目标”
- 该状态足以支持单关节便捷命令
- 不再需要 `MotorIO` 中的 `target_queue_`、`active_target_joint_`、`worker_thread_`

### 电机命令下发路径

`SetJointPositions()` 必须保留当前 `daemon/motor_io.cpp` 已验证的 joint-space 转换逻辑：

1. 输入是 12 维 joint-space 关节角（rad）
2. 若为膝关节，先乘 `kKneeRatio`
3. 乘 `kJointDirection`
4. 加 `kJointOffsets`
5. 按 `kJointMin` / `kJointMax` 夹紧
6. 调用 `RobstrideController::SendMITCommand()` 逐关节下发

这保证了公开 API 继续面向机器人关节，而不是退回到底层电机轴绝对角接口。

### 电机状态读回路径

`GetJointState()` / `GetAllJointStates()` 必须保留当前 `daemon/motor_io.cpp` 的 joint-space 读回逻辑：

- `position_rad`：由底层电机位置减 offset、乘方向、对膝关节除以 `kKneeRatio`
- `velocity_rad_s`：沿用与位置一致的方向和膝关节比例换算规则
- `torque_nm`：直接暴露底层控制器上报值，不在本次设计中引入额外传动侧换算

### SetZero 语义边界

`SetZero(JointId joint)` 必须明确映射到底层电机的 `SetZero` 操作。

它的含义是“重定义该关节对应电机的零点”，而不是“让关节运动到零位姿”。

因此：

- `SetZero(joint)` = 标定
- `SetJointPosition(joint, 0.0f)` = 在当前零点定义下发送关节目标

这两个动作必须在头文件和 CLI 帮助信息中严格区分。

### ImuDriver 内部职责

`src/imu_driver.cpp` 内部复用：

- `driver/include/serial.h`
- `driver/src/wit_c_sdk.c`
- 当前 `driver/src/observations.cpp` 中成熟的数据转换逻辑

建议内部维护：

- 串口 fd
- 初始化状态
- 最新一帧原始 IMU 寄存器/解析缓存
- 最新一帧转换后的 `ImuData`

其中 `ImuData` 的字段语义需要严格贴合现有实现：四元数按 `[w, x, y, z]` 存储，gyro 转为 rad/s，projected gravity 沿用当前策略坐标系；加速度字段先保持“原始解析值”语义，避免在未核实单位前写成 `m/s^2`。

### IMU 数据路径

`ImuDriver` 必须保留现有实现中已经使用的两个核心变换：

1. **gyro 单位转换**
   - 从度每秒转换为弧度每秒
2. **projected gravity 计算**
   - 由四元数推导重力方向
   - 保持与当前控制策略一致的轴约定与符号约定

本次设计不重新定义 IMU 坐标语义，而是将当前已经在用的结果稳定地封装进库接口。

### IMU 实例边界

由于 WIT C SDK 当前是全局回调/全局状态风格，本次设计接受如下约束：

- 单进程只支持一个 `ImuDriver` 实例

这与当前单机器人、单 IMU 的使用场景一致，避免为了并不存在的多 IMU 需求引入额外复杂度。

## 目录与构建产物

建议目标目录结构：

```text
.
├── CMakeLists.txt
├── include/
│   └── dog/
│       ├── imu_driver.hpp
│       └── motor_driver.hpp
├── src/
│   ├── imu_driver.cpp
│   └── motor_driver.cpp
├── config/
│   └── robot_config.hpp
├── driver/
│   ├── include/
│   └── src/
└── tests_cli/
    ├── imu_cli.cpp
    └── motor_cli.cpp
```

建议新的 CMake 产物为：

1. **静态库** `dog_robot`
2. **电机 CLI 测试程序** `motor_cli`
3. **IMU CLI 测试程序** `imu_cli`

由于库会直接编译 `driver/src/serial.c` 与 `driver/src/wit_c_sdk.c`，`CMakeLists.txt` 需要把项目语言声明为同时支持 `C` 和 `CXX`，而不能继续只写 `project(dog_fifo_backend CXX)`。

库目标链接源码建议包括：

- `src/motor_driver.cpp`
- `src/imu_driver.cpp`
- `driver/src/can_interface.cpp`
- `driver/src/robstride.cpp`
- `driver/src/serial.c`
- `driver/src/wit_c_sdk.c`

公开 include 路径只暴露：

- `include/dog/motor_driver.hpp`
- `include/dog/imu_driver.hpp`

调用者不需要直接包含 `can_interface.hpp`、`robstride.hpp`、`serial.h` 或 WIT SDK 相关文件。

## CLI 测试设计

### motor_cli

文件路径：`tests_cli/motor_cli.cpp`

目标是基于公开 API 做真机验证，而不是直接测试内部类。

建议命令：

```bash
motor_cli enable
motor_cli disable
motor_cli autoreport
motor_cli read --all
motor_cli read --joint LF_Knee
motor_cli set --joint LF_Knee --pos 0.30
motor_cli zero --joint LF_Knee
```

每条命令的验证目标：

- `enable`
  - 验证 `Initialize()` 与 `EnableAll()` 成功完成
- `disable`
  - 验证 `DisableAll()` 行为正确
- `autoreport`
  - 验证自动反馈开启后状态读取稳定
- `read`
  - 验证公开 joint-space 状态读回接口
- `set`
  - 验证 joint-space 目标下发链路
- `zero`
  - 验证零点标定命令走到底层 `SetZero`

### imu_cli

文件路径：`tests_cli/imu_cli.cpp`

建议命令：

```bash
imu_cli read
imu_cli stream
imu_cli check-gravity
```

每条命令的验证目标：

- `read`
  - 打印 quaternion、gyro、acc_raw、projected gravity，确认初始化与单次采样链路
- `stream`
  - 连续输出数据，观察静止/转动时 gyro 是否符合预期
- `check-gravity`
  - 输出 projected gravity，并给出人工检查准则，确认姿态变化时符号和主方向变化合理

### CLI 的定位

CLI 程序是本次重构的主要验证手段：

- 它验证“别人只看公开头文件就能正确调用库”的路径
- 它直接服务于真实硬件验证需求
- 它优先于 mock/fake 测试架构

## 错误处理策略

两个公开类统一采用当前代码库已存在的风格：

- 动作类接口返回 `bool`
- 错误详情通过 `std::string& err` 输出
- 读取类接口返回当前缓存结果，不抛异常

例如：

- `Initialize(std::string& err)`
- `EnableAll(std::string& err)`
- `SetJointPositions(..., std::string& err)`
- `Update(std::string& err)`

这样做的原因：

- 与当前 `MotorIO` 风格一致
- 便于 CLI 直接转发错误信息
- 避免为简单硬件接口引入异常语义

## 与旧 daemon 的关系

本次重构的优先目标是“先得到一个干净可复用的库”和“先有真机 CLI 验证路径”。

因此旧 daemon 的处理策略是：

- 不把 TCP 协议、JSON state line、worker queue 语义放进公开 API
- 重构第一阶段先完成库与 CLI
- 后续若仍需 daemon，可让 daemon 改为调用 `dog_robot` 库，而不是继续让库暴露 daemon 专属概念

## 明确的设计取舍

### 保留

- 单机器人写死配置
- 面向 joint-space 的电机接口
- 立即发送当前目标的 `set_joint` 语义
- 真机 CLI 测试
- 公开头文件强注释
- 一个库目标、两个公开类

### 不保留

- `MotorIO` 的 queue / worker / active target 行为
- daemon 的 TCP 协议
- JSON 状态推流
- 后台平滑插值发送作为公开 API
- 为未来多机器人/多 IMU 做的预留抽象

## 成功标准

当以下条件同时满足时，本次设计视为成功：

1. 调用者只阅读 `include/dog/motor_driver.hpp` 和 `include/dog/imu_driver.hpp` 即可理解使用方式。
2. 电机调用者不需要知道 CAN 帧、Robstride 协议、offset、direction、knee ratio 的内部细节。
3. IMU 调用者不需要知道串口细节和 WIT SDK 全局状态。
4. `motor_cli` 可以完成 enable / disable / autoreport / read / set / zero 真机验证。
5. `imu_cli` 可以完成 gyro 读取和 projected gravity 正确性验证。
6. 旧 daemon 专属概念不会污染新的公开库接口。
