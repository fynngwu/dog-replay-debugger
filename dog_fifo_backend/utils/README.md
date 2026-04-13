# utils

Lightweight C++ utility library — header-only, zero external dependencies.

## Components

| Component | Header | Description |
|---|---|---|
| `AsyncMessageProcess<T>` | `utils/async_message_process.h` | Thread-safe async message queue with callback processing, frame skipping |
| `Timer` | `utils/timer.h` | Function execution timer with statistics (mean, median, 95th percentile, CSV export) |

## Quick Start

```bash
git submodule add https://github.com/fynngwu/utils.git third_party/utils
```

```cmake
target_include_directories(your_target PRIVATE third_party/utils/include)
```

```cpp
#include <utils/async_message_process.h>
#include <utils/timer.h>

// ===================== Timer =====================
// 包裹任意函数/lambda，自动记录执行时间
Timer::Evaluate([&]() { MapIncremental(); }, "Incremental Mapping");
Timer::Evaluate(
    [&, this]() {
        scan_count_++;
        double timestamp = ToSec(msg->header.stamp);
        // ...
    },
    "Preprocess (Standard)");

// 打印所有统计：平均、中位数、95分位、调用次数
Timer::PrintAll();

// 导出CSV，方便作图分析
Timer::DumpIntoFile("timing.csv");

// ===================== AsyncMessageProcess =====================
// 类成员声明
AsyncMessageProcess<Keyframe::Ptr> kf_thread_;

// 初始化：设置回调、名字、启动线程
kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
kf_thread_.SetName("handle loop closure");
kf_thread_.Start();

// 外部模块投递消息，回调在独立线程串行执行
kf_thread_.AddMessage(kf);

// 退出
kf_thread_.Quit();
```

## Build Test

```bash
cd test && g++ -std=c++17 -I../include test_utils.cc -o test_utils -pthread && ./test_utils
```

## Build Requirements

- C++17
- pthread
