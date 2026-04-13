# Dog Replay Debugger 前端架构

## 整体架构

```
                    ┌──────────────────────────────────────────────────────────┐
                    │                    PySide6 GUI Thread                    │
                    │                     (Main Thread)                        │
                    │                                                        │
                    │  MainWindow ─── QTimer(20ms) ──> refresh()               │
                    │      │                   │                               │
                    │      ├── ControlPanel    ├── JointDashboard              │
                    │      ├── StatusPanel     │     └── JointCard x12         │
                    │      ├── KneeDebugPanel  ├── BackendStatePanel x2        │
                    │      ├── CurvesPanel     └── QPlainTextEdit (logs)       │
                    │      └── QPlainTextEdit (logs)                           │
                    └───────────┬──────────────────────────────────────────────┘
                                │ get_snapshot()
                                ▼
                    ┌───────────────────────────┐
                    │      ReplayEngine          │  (无自有线程，仅做逻辑调度)
                    │  ┌───────────────────────┐ │
                    │  │   SharedStateBus       │ │  (线程安全的共享状态中心)
                    │  │   RLock 保护所有状态   │ │
                    │  └───────────────────────┘ │
                    └───┬───────────┬───────────┬─┘
                        │           │           │
              ┌─────────┘           │           └─────────┐
              ▼                     ▼                     ▼
     ┌─────────────────┐  ┌──────────────────┐  ┌──────────────────┐
     │  MuJoCoAdapter   │  │  RobotAdapter     │  │  Play Thread      │
     │                  │  │                   │  │  (ReplayEngine    │
     │ ┌──sim_thread──┐ │  │ ┌──tx_thread──┐  │  │   _play_loop)     │
     │ │ PD控制+步进 │ │  │ │ 定时发送    │  │  │                   │
     │ │ 500Hz 仿真  │ │  │ │ joint命令   │  │  │ │ 逐帧推进cursor  │
     │ └──────────────┘ │  │ └─────────────┘  │  │ │ 调用bus更新    │
     │ ┌──viewer_thread┐ │  │ ┌──rx_thread──┐  │  │ └─────────────────┘
     │ │ passive viewer│ │  │ │ 接收state   │  │  │
     │ │ 100Hz sync   │ │  │ │ 解析JSON    │  │  │
     │ └──────────────┘ │  │ │ 更新bus     │  │  │
     └─────────────────┘  │ └─────────────┘  │  │
                          └──────────────────┘  │
                                                │  SSH后台操作也用独立daemon thread
                                                └──────────────────┘
```

## 线程模型

整个应用有 **5 类后台线程** + **1 个 GUI 主线程**，全部通过 `SharedStateBus` (RLock) 做线程安全的状态交换。

| 线程 | 创建位置 | 生命周期 | 频率 | 功能 |
|------|---------|----------|------|------|
| **GUI Main Thread** | `main()` QApplication | 进程生命周期 | QTimer 50Hz (20ms) | UI 渲染、事件处理、从 bus 读 snapshot 刷新所有面板 |
| **Play Thread** | `ReplayEngine.start()` | `start()` 到 `stop()`/播完 | ~500Hz (sleep 2ms) | 按 wall-clock 推进 replay cursor，写 bus target |
| **MuJoCo Sim Thread** | `MujocoAdapter.load_model()` | 加载模型到 `close()` | ~500Hz (模型 timestep) | PD 力矩控制 + `mj_step`，读 bus target，写 mujoco_state |
| **MuJoCo Viewer Thread** | `MujocoAdapter.load_model()` | 加载模型到关闭/异常 | ~100Hz | `viewer.sync()` 渲染 MuJoCo 3D 窗口 |
| **Robot TX Thread** | `RobotAdapter.connect()` | connect 到 disconnect | ~50Hz (tx_period=20ms) | 读取 bus target，通过 cmd socket 发送 `set_joint` 命令 |
| **Robot RX Thread** | `RobotAdapter.connect()` | connect 到 disconnect | 独立接收 | 从 state socket 接收 JSON，解析关节状态/后端状态，写 bus |
| **SSH Worker Thread** | `MainWindow._run_remote_backend_job()` | 一次性 daemon | 一次性 | `subprocess.run(ssh ...)` 远程 kill/start backend |

### 线程同步机制

所有线程间通信通过 **`SharedStateBus`** 的 `RLock` 实现：

```
写入方 (后台线程)                    读取方 (GUI 线程)
─────────────────                   ─────────────────
Play Thread      ──set_cursor_target──>  bus._cursor, bus._target
MuJoCo Sim       ──update_mujoco_state─>  bus._mujoco_state
Robot TX         ──get_target──────────>  读 bus._target (只读)
Robot RX         ──update_robot_state──>  bus._robot_state, bus._backend_state
GUI refresh()    ──snapshot()─────────>  读所有状态，生成不可变 RuntimeSnapshot
```

`snapshot()` 方法会复制所有 numpy 数组和字符串，确保 GUI 拿到的是独立副本。

---

## 文件功能说明

### `gui/` 前端 UI 层

| 文件 | 类/函数 | 功能 |
|------|--------|------|
| `main_window.py` | `MainWindow` | **顶层窗口**。组装所有子面板，绑定信号槽，管理录制逻辑，20ms QTimer 驱动全局刷新。包含 SSH 后台操作的 daemon thread 创建。 |
| | `main()` | 应用入口，创建 QApplication 和 MainWindow |
| `view_model.py` | `joint_rows()` | 纯函数，从 `RuntimeSnapshot` 提取 12 个关节的 (name, target, mujoco, robot, torque) 行数据 |
| `control_panel.py` | `ControlPanel` | 左侧控制面板。包含：文件路径/加载、机器人连接、SSH 远程操作、MIT 参数、回放控制(Play/Stop/Step/Frame slider/Speed)、录制按钮。纯 UI 组件，不包含逻辑。 |
| `status_panel.py` | `StatusPanel` | 顶部状态栏。显示 Frame、Replay状态、Speed、Robot连接、TX/RX频率、State age、Enabled/Busy/Queue、KP/KD、MuJoCo状态、Viewer状态。 |
| `joint_dashboard.py` | `JointDashboard` | 12 关节实时监控面板。按腿分组 (FL/FR/RL/RR)，每腿 3 个 `JointCard`。支持点击选中关节。 |
| `joint_card.py` | `JointCard` | 单个关节卡片组件。显示 Target/MuJoCo/Robot/Torque 四个字段，力矩超阈值变色警告。可点击选中(边框高亮)。 |
| `joint_table.py` | `JointTable` | 关节数据表格组件（当前未被使用，是历史遗留的表格视图）。 |
| `curves_panel.py` | `CurvesPanel` | 关节历史曲线图。基于 pyqtgraph，用 deque 缓冲最近 400 个采样点，绘制 target/mujoco/robot/torque 四条曲线。支持切换关节。 |
| `knee_debug_panel.py` | `KneeDebugPanel` | 关节调试面板（所有 12 关节）。每关节一个 `JointEditorRow`（滑块+SpinBox+Send按钮+SetZero按钮）。支持批量操作：Send all / Load stream pose / Load target / Zero editors。包含独立的 `BackendStatePanel` 实例。 |
| | `JointEditorRow` | 单关节编辑器行。滑块与 SpinBox 双向同步，实时显示 clamp 预览值和当前流式状态。 |
| `backend_state_panel.py` | `BackendStatePanel` | 后端流式状态面板。显示 ok/enabled/worker/busy/init/queue/kp/kd/age 摘要 + 12 关节详细表格 (StatePos/Torque/ActiveTarget/LastSent/UI Target) + last_error + Raw JSON。 |
| `styles.py` | `MODERN_STYLE`, `JOINT_CARD_STYLE`, `DASHBOARD_STYLE`, `STATUS_PANEL_STYLE` | 全局 QSS 暗色主题样式定义。 |
| `__init__.py` | — | 包初始化文件。 |

### `replay_core/` 核心逻辑层

| 文件 | 类/函数 | 功能 |
|------|--------|------|
| `replay_engine.py` | `ReplayEngine` | **核心引擎**。持有 `SharedStateBus`、`Sequence`、Robot/MuJoCo 适配器。负责 CSV 加载、回放控制 (start/stop/seek/step/prev)、机器人命令转发、SSH 远程操作。管理 play thread 的创建与停止。 |
| `sync_bus.py` | `SharedStateBus` | **线程安全共享状态总线**。所有后台线程与 GUI 线程的唯一通信枢纽。用 `RLock` 保护所有字段。提供 `snapshot()` 方法一次性生成不可变的 `RuntimeSnapshot` 副本。 |
| `types.py` | `RuntimeSnapshot`, `ReplaySequence`, `ReplayFrame`, `JointState`, `BackendState` | 数据类型定义。`RuntimeSnapshot` 是 GUI 刷新的核心数据结构，包含当前帧所有状态。 |
| `csv_loader.py` | `load_replay_csv()`, `CsvLoadError` | CSV 文件加载。自动检测列名格式 (target_rel_*/scaled_action_*)，解析时间戳，构建 `ReplaySequence`。 |
| `joint_limits.py` | `clamp_relative_targets()`, `relative_to_absolute()`, `clamp_single_relative_target()` 等 | 关节限位。实现 relative <-> absolute 坐标转换，clamp 到 XML 定义的关节范围。KNEE_RATIO = 1.667，Knee 关节有特殊缩放。 |
| `constants.py` | `JOINT_NAMES`, `NUM_JOINTS`, `POLICY_TO_SIM`, `SIM_TO_POLICY` | 常量定义。12 关节名称、策略顺序↔仿真顺序的映射数组。 |
| `config.py` | `EngineConfig` | 引擎配置数据类。play_loop_sleep、robot 超时/端口、默认帧间隔。 |
| `metrics.py` | `RateTracker` | 频率统计工具。滑动窗口法计算事件频率 (Hz)。 |

### `adapters/` 适配器层

| 文件 | 类 | 功能 |
|------|----|------|
| `robot_adapter.py` | `RobotAdapter` | **真实机器人通信适配器**。管理 cmd socket (命令发送) 和 state socket (状态接收)。连接时创建 TX/RX 两个后台线程。TX 线程轮询 bus target 发送；RX 线程接收 JSON 更新 robot/backend state。支持自动重连 (超时 3 次后)。 |
| `mujoco_adapter.py` | `MujocoAdapter` | **MuJoCo 仿真适配器**。加载 XML 模型，创建 sim_thread (PD 控制力矩 + mj_step) 和 viewer_thread (passive viewer 渲染)。自动检测 freejoint (浮动基座 vs 固定基座)。仿真频率 = 模型 timestep (0.002s = 500Hz)。 |
| `null_robot_adapter.py` | `NullRobotAdapter` | **空机器人适配器**。未连接时的默认适配器，所有操作返回空/失败。 |

---

## 依赖关系图

```
main_window.py
├── control_panel.py          (组装 UI)
├── joint_dashboard.py        (组装 UI)
│   └── joint_card.py         (创建 12 个卡片)
│       └── styles.py         (延迟导入卡片样式)
├── status_panel.py           (组装 UI)
│   └── styles.py             (延迟导入状态栏样式)
├── knee_debug_panel.py       (组装 UI)
│   └── backend_state_panel.py (内嵌实例)
├── backend_state_panel.py    (组装 UI, Overview tab 中再创建一个)
├── styles.py                 (全局样式 MODERN_STYLE)
├── curves_panel.py           (组装 UI, 当前未被添加到界面)
│   └── (optional) pyqtgraph
├── replay_core/replay_engine.py  (核心引擎)
│   ├── replay_core/sync_bus.py       (共享状态)
│   │   └── replay_core/types.py      (数据类型)
│   │       └── replay_core/constants.py
│   ├── replay_core/csv_loader.py     (CSV 加载)
│   │   └── replay_core/types.py, constants.py
│   ├── replay_core/joint_limits.py   (关节限位)
│   │   └── replay_core/constants.py
│   ├── replay_core/config.py         (引擎配置)
│   │   └── replay_core/constants.py
│   ├── adapters/mujoco_adapter.py     (仿真)
│   │   ├── replay_core/sync_bus.py
│   │   ├── replay_core/constants.py (POLICY_TO_SIM 等)
│   │   └── replay_core/metrics.py
│   ├── adapters/null_robot_adapter.py (默认空适配器)
│   └── adapters/robot_adapter.py      (真实机器人)
│       ├── replay_core/sync_bus.py
│       ├── replay_core/constants.py
│       ├── replay_core/metrics.py
│       └── replay_core/types.py (BackendState)
├── replay_core/constants.py  (JOINT_NAMES, 用于录制 CSV header)
└── joint_table.py             (未使用)

knee_debug_panel.py (额外依赖)
├── backend_state_panel.py
└── replay_core/joint_limits.py (clamp_single_relative_target, XML_MIN, XML_MAX)

joint_dashboard.py (额外依赖)
├── joint_card.py
└── replay_core/constants.py (JOINT_NAMES)

view_model.py
├── replay_core/constants.py (JOINT_NAMES)
└── replay_core/types.py (RuntimeSnapshot)

curves_panel.py
└── replay_core/constants.py (JOINT_NAMES)
```

## 数据流

```
                    CSV 文件
                       │
                       ▼ load_replay_csv()
              ┌─────────────────┐
              │ ReplaySequence  │  (frames: List[ReplayFrame])
              │   .target_rel   │  (12 个关节的目标角度, relative 坐标)
              └────────┬────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
    Play Thread   seek/step    manual_target
    (自动推进)    (手动跳帧)   (手动输入)
          │            │            │
          └────────────┼────────────┘
                       │ set_cursor_target(cursor, raw, clamped)
                       ▼
              ┌─────────────────┐
              │ SharedStateBus  │  .cursor, ._target_raw, ._target
              └──┬──────────┬───┘
                 │          │
          ┌──────┘          └──────┐
          ▼                        ▼
    MuJoCo Sim Thread         Robot TX Thread
    读 .target (policy order)  读 .target
    转换 sim order, PD力矩     直接发送
    mj_step()                  set_joint 命令
          │                        │
          ▼                        ▼
    update_mujoco_state()    (backend 收到命令)
          │                        │
          │                        ▼
          │                 Robot RX Thread
          │                 收到 state JSON
          │                 update_robot_state()
          │                 update_backend_state()
          │                        │
          └────────────┬───────────┘
                       ▼
              ┌─────────────────┐
              │ SharedStateBus  │  (所有状态已更新)
              └────────┬────────┘
                       │ snapshot()  [GUI Thread, 每 20ms]
                       ▼
              ┌─────────────────┐
              │ RuntimeSnapshot │  (不可变快照)
              └────────┬────────┘
                       │
          ┌────────────┼────────────┬──────────────┐
          ▼            ▼            ▼              ▼
    ControlPanel  JointDashboard  StatusPanel  BackendStatePanel
    (frame spin,  (12 cards)      (状态栏)     (关节表格+
     slider)                                Raw JSON)
          │            │
          ▼            ▼
    KneeDebugPanel
    (editor rows +
     state panel)
```

## 备注

- `curves_panel.py` 已实现但**未被添加到 MainWindow 的 UI 布局中**，属于已完成但未集成的组件。
- `joint_table.py` 是历史遗留的表格视图，当前未被任何面板引用。
- `view_model.py` 的 `joint_rows()` 函数当前未被使用（原为 joint_table 服务的辅助函数）。
