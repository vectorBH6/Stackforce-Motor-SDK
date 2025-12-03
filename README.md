# SF Motor Control

SF Motor Control 是一个基于 C++ 的电机控制项目，用于通过 CAN 总线与 SF 系列电机控制器通信，实现对电机的精确控制。

## 项目概述

该项目提供了通过 CAN 总线控制 SF 系列电机的功能，支持电机使能、失能、零位设置以及 MIT（Motor Identity Transform）控制模式等功能。MIT 控制模式允许用户直接设定电机的位置、速度、Kp和Kd等参数。

项目同时提供了 C++ 和 Python 两种语言的实现

## 功能特点

- 支持 SF 系列电机的完整控制（使能/失能/零位设置）
- 实现 MIT 控制模式，可精确控制电机位置、速度和力矩
- 支持多电机控制（最多 4 个电机）
- 提供实时反馈，可以读取电机的实际位置、速度和扭矩
- 同时提供 C++ 和 Python 实现

## 硬件要求

- 支持 CAN 总线的主机系统（如 Jetson Nano、树莓派等）
- SF 系列电机及相应的驱动器
- CAN 总线收发器
- 正确连接的 CAN 总线网络

## 软件依赖

### C++ 版本
- C++17 兼容的编译器（如 GCC 7+）
- CMake 3.10+
- Linux 系统（需要 socketCAN 支持）

### Python 版本
- Python 3.6+

## 使用方法

在运行之前，请确保系统已经正确配置了 CAN 接口。例如，对于 `can0` 接口：

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 up
```

### C++ 版本

```bash
cd build
cmake ..
make
```

编译后的可执行文件将位于 `build/sfmotor_control`。运行程序：

```bash
./sfmotor_control
```

程序默认会控制 ID 为 0x01 的电机，在运行过程中可以通过键盘输入目标角度值，单位rad。同时接收电机角度，角速度的反馈数据。

### Python 版本

Python 脚本位于 `script/` 目录中，可以直接运行无需编译。

```bash
python main.py 
```

程序默认会控制 ID 为 0x01 的电机，在运行过程中可以通过键盘输入目标角度值，单位rad。同时接收电机角度，角速度的反馈数据。

## 代码结构

```
SFmotor_control/
├── include/                  # 头文件及实现文件
│   ├── CAN_comm.cpp          # CAN 通信实现
│   ├── CAN_comm.h            # CAN 通信接口定义
│   ├── CAN_twai.cpp          # TWAI 协议实现
│   └── CAN_twai.h            # TWAI 协议接口定义
├── script/                   # Python 控制脚本
│   ├── __init__.py
│   ├── main.py               # Python 示例主程序
│   └── sf_can_controller.py  # Python CAN 控制器实现
├── src/                      # C++ 主程序源码
│   ├── config.h              # 配置和数据结构定义
│   └── main.cpp              # 主控制程序
└── CMakeLists.txt            # CMake 构建配置
```

## 通信协议

项目实现了针对 SF 系列电机的专用 CAN 通信协议：

- 使用标准 CAN 帧（11 位标识符）
- 支持多种功能码（NMT、RPDO、TPDO 等）
- MIT 控制模式采用专有数据格式

### 主要功能码

| 功能码 | 值     | 用途             |
|--------|--------|------------------|
| NMT    | 0x000  | 网络管理         |
| RPDO1  | 0x200  | 实时过程数据输出 |
| TPDO1  | 0x180  | 实时过程数据输入 |

### 控制命令

1. **使能命令**：通过 NMT 功能码发送使能信号
2. **失能命令**：通过 NMT 功能码发送失能信号
3. **MIT 控制命令**：通过 RPDO1 发送位置、速度、Kp、Kd 和扭矩参数

## API 参考

### C++ API

主要函数包括：

- `CANInit()` - 初始化 CAN 接口
- `enable(uint8_t nodeID)` - 使能指定 ID 的电机
- `disable(uint8_t nodeID)` - 失能指定 ID 的电机
- `sendMITCommand(uint8_t nodeID, MIT command)` - 发送 MIT 控制命令
- `recCANMessage()` - 接收 CAN 消息

### Python API

主要类和方法：

- `MotorController` 类
  - `enable()` - 使能电机
  - `disable()` - 失能电机
  - `send_mit_command(pos, vel, kp, kd, tor)` - 发送 MIT 控制命令
  - `poll_rx()` - 轮询接收 CAN 消息

## 注意事项

1. 在运行程序前必须确保 CAN 接口已经正确配置并启动
2. 电机控制涉及大功率设备，请注意电气安全
3. 项目目前假设 CAN 接口名称为 `can0`，如有不同请修改源代码
4. 控制参数（如 Kp、Kd）需要根据具体应用场景进行调整

## 故障排除

常见问题及解决方案：

1. **无法打开 CAN 接口**
   - 检查是否正确配置并启动了 CAN 接口
   - 确认使用的接口名称是否匹配（默认为 can0）

2. **无法与电机通信**
   - 检查 CAN 总线物理连接
   - 确认电机 ID 设置是否正确
   - 验证 CAN 波特率设置是否匹配

3. **控制效果不佳**
   - 调整 PID 参数（Kp、Kd）
   - 检查电机和负载的机械连接
   - 确保供电电压稳定
