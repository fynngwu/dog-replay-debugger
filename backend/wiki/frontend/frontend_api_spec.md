# 前端对接 Spec

本页描述 Qt 前端与 C++ daemon 的对接接口。

## 架构说明

后端由 C++ daemon 组成：
- **命令端口**：`47001`
- **状态推流端口**：`47002`

前端实现位于项目根目录：
- `adapters/robot_adapter.py` - Robot 通信适配层
- `replay_core/replay_engine.py` - 回放引擎（本地实现）

---

## 1. 连接模型

### 命令通道
- TCP
- 单行文本命令
- UTF-8
- 必须以 `\n` 结尾
- 每条命令返回一条 JSON 行

### 状态推流
- TCP
- 服务端每 **50ms** 推一次（20Hz）
- 每条状态是一条 JSON 行
- 前端应常驻连接并持续消费

---

## 2. 前端调用路径

```
Qt Widget (MainWindow)
    ↓
ReplayEngine
    ↓
RobotAdapter
    ↓
TCP cmd/state sockets
    ↓
Jetson C++ daemon
```

---

## 3. 支持的命令

### 连接与测试
- `ping` - 连通性测试

### 机器人控制
- `init [duration_sec]` - 初始化机器人
- `enable` - 使能电机
- `disable` - 失能电机
- `set_joint <12 floats>` - 设置关节目标

### 单电机调试
- `joint_test <idx[,idx...]> <target_rad>` - 测试指定关节
- `joint_sine <idx[,idx...]> <amp_rad> <freq_hz> <duration_sec>` - 正弦摆动测试

### MIT 参数
- `set_mit_param <kp> <kd> <vel_limit> <torque_limit>` - 设置 MIT 控制参数

### 状态
- `get_state` - 获取一次状态快照

---

## 4. 状态字段

前端应从状态流或 `get_state` 中读取：
- `state.joint_positions` (长度 12)
- `state.joint_velocities` (长度 12)
- `state.joint_torques` (长度 12)
- `state.target_joint_positions` (长度 12)
- `state.offline_motors` (字符串数组)
- `motion.active` - 是否有运动在执行
- `motion.name` - 运动名称
- `motion.last_error` - 上次错误
- `fault` - 故障信息（可选）

---

## 5. 关节索引表

| idx | joint_name |
|---:|---|
| 0 | LF_HipA |
| 1 | LR_HipA |
| 2 | RF_HipA |
| 3 | RR_HipA |
| 4 | LF_HipF |
| 5 | LR_HipF |
| 6 | RF_HipF |
| 7 | RR_HipF |
| 8 | LF_Knee |
| 9 | LR_Knee |
| 10 | RF_Knee |
| 11 | RR_Knee |

---

## 6. 错误处理

错误返回统一形式：

```json
{
  "ok": false,
  "msg": "human readable message",
  "error": {
    "code": "bad_command",
    "message": "human readable message",
    "detail": "optional extra detail"
  }
}
```

---

## 7. 前端线程建议

不要把 socket 或长时间 I/O 放在 UI 主线程。

推荐：
- 状态监听：后台线程
- 曲线刷新：主线程定时器
- 命令调用：worker + signal/slot

---

## 8. 已知边界

1. 状态推流服务当前是 **单客户端串行 accept 模型**
2. 命令口也更适合由一个长期连接的客户端独占
3. `joint_torques` 来自底层控制器反馈，语义更接近 **电机/控制器原始 torque telemetry**
