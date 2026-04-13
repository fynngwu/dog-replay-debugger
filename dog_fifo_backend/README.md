# dog_fifo_backend

一个极简后端：
- **1 个 command 线程**：长连接收命令
- **1 个 motor worker 线程**：2ms FIFO 开环分段发送 MIT command
- **1 个 state 线程**：50Hz 推流状态

`robostride` 与 `can_interface` 原样保留，不做修改。

## 目录结构

```text
.
├── CMakeLists.txt
├── build.sh
├── README.md
├── daemon/
│   ├── main.cpp
│   ├── motor_config.hpp
│   ├── motor_io.hpp
│   ├── motor_io.cpp
│   ├── protocol.hpp
│   ├── tcp_server.hpp
│   └── tcp_server.cpp
└── driver/
    ├── include/
    └── src/
```

## 编译

```bash
chmod +x build.sh
./build.sh
```

生成：

```bash
build/dog_fifo_backend
```

## 启动

默认端口：
- command: `47001`
- state: `47002`

```bash
./build/dog_fifo_backend
```

或自定义端口：

```bash
./build/dog_fifo_backend 47001 47002
```

## 先改这个配置文件

真实上机前，先检查并修改：

```text
daemon/motor_config.hpp
```

重点项：
- `kCanIfNames`
- `kMotorIds`
- `kJointOffsets`
- `kJointMin`
- `kJointMax`
- `kKneeRatio`
- `kJointDirection`

当前包里给的是**可编译默认值**，其中 offset / 限位 / knee ratio 需要按你的实机填写。

## 线程模型

### 1. command 线程
- 长连接
- 按 `\n` 分帧
- 每条命令返回一条 JSON

### 2. worker 线程
- `2ms` 周期
- 无 target 时 `idle`
- 收到 `setjoint` 后按 FIFO 执行
- 每次切到下一个 target 时，会先读一次当前实测 joint position 作为 ramp 起点
- 到达判定：`last_sent_joint` 与目标误差 `< 1e-4`
- 每次发送 **12 个关节**
- 完全开环，不用 obs 判到达

### 3. state 线程
- 50Hz
- 长连接持续推 JSON line

## 命令协议（command port）

所有命令都以 `\n` 结尾。

### 1. ping

```text
ping
```

返回：

```json
{"ok":true,"msg":"pong"}
```

### 2. init

语义：
- enable all
- enable auto report
- 清空 queue
- 将当前实测 joint 作为 ramp 起点
- 同步走到 offset-relative 的全 0 joint pose
- **等真正走到再返回 ok**

```text
init
```

返回：

```json
{"ok":true,"msg":"initialized to offset"}
```

### 3. disable

语义：
- 清空 queue
- 清掉 active target
- 不再发新指令
- disable all

```text
disable
```

### 4. setjoint / set_joint

12 维整帧原子入队。

```text
setjoint q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12
```

或

```text
set_joint q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12
```

成功返回：

```json
{"ok":true,"msg":"queued","queue_size":3}
```

队列最大长度固定为 `10`，满了返回：

```json
{"ok":false,"code":"queue_full","msg":"queue_full","queue_size":10}
```

### 5. setzero / set_zero

语义：**真正调用底层驱动 `SetZero`，做单关节零点标定**。

```text
setzero 8
```

或

```text
set_zero 8
```

其中 joint index 范围：`0..11`

顺序：
- 0..3: `LF_HipA LR_HipA RF_HipA RR_HipA`
- 4..7: `LF_HipF LR_HipF RF_HipF RR_HipF`
- 8..11: `LF_Knee LR_Knee RF_Knee RR_Knee`

### 6. set_mit_param

```text
set_mit_param 25 0.5 44 17
```

参数为：
- `kp`
- `kd`
- `vel_limit`
- `torque_limit`

会立即写入所有电机，并在 state 推流里回传当前值。

## 状态推流协议（state port）

连接后每 `20ms` 推一行 JSON：

```json
{
  "ok": true,
  "enabled": true,
  "worker_started": true,
  "busy": true,
  "queue_size": 2,
  "kp": 25.0,
  "kd": 0.5,
  "vel_limit": 44.0,
  "torque_limit": 17.0,
  "joint_positions": [...],
  "joint_torques": [...],
  "target_joint_positions": [...],
  "last_sent_joint_positions": [...],
  "last_error": ""
}
```

字段说明：
- `joint_positions`：已经做了方向与 knee ratio 处理后的虚拟关节空间位置
- `joint_torques`：直接来自底层反馈 torque
- `target_joint_positions`：当前 active target（如果 idle，则通常是上一轮清空后的值）
- `last_sent_joint_positions`：worker 内部最近一次真正发出去的 ramp joint
- `busy`：当前是否正在执行 active target
- `queue_size`：FIFO 中剩余等待数量

## 说明

### 为什么无 target 时不继续发最后目标
这是按你的要求实现的：
- 没有 active target
- queue 为空
- worker 就 idle，不继续发最后命令

### 为什么切下一个 target 时要重新读 obs
这是按你的要求实现的：
- 每次从 FIFO 取出新的 active target
- 会重新读取当前实测 joint position
- 用这个位置作为新的 ramp 起点

### 关于 torque
当前 `joint_torques` 直接返回 robostride 的 torque 反馈，没有再做方向 / 齿比换算。
如果你之后想改成虚拟关节空间 torque，可以在 `MotorIO::GetSnapshot()` 里统一改。

## 已知边界

1. 当前包不做更复杂的 fault/offline 状态机。
2. `setzero` 是直接调底层 `SetZero`，请只在你确认机械姿态正确时使用。
3. 为了保持函数风格简洁，worker 内部的 `compute next step / target reached` 直接写在 loop 里，没有再拆碎函数。
