# legged-robot

[English](README.md)

本项目是双足机器人控制板固件工程，运行在 STM32H723VGTX（Cortex-M7）平台，采用 STM32 HAL + FreeRTOS 架构，实现传感器数据采集、姿态解算、运动控制、上位机通信与电源管理等核心功能。

## 1. 项目概览

- MCU：STM32H723VGTX
- RTOS：FreeRTOS（CMSIS-RTOS 适配）
- 通信：FDCAN、USB CDC、USART、Marvlink（上位机）
- 主要算法：EKF、Mahony、Kalman、PID、VMC
- 主要设备：BMI088 IMU、DM 系列电机等

工程目标产物：`CtrlBoard-H7_IMU.elf`（并生成 `.hex`、`.bin`）。

## 2. 软件架构

系统采用“底层驱动 + 中间件 + 任务层 + 算法层 + 控制层”分层设计：

- **底层驱动层**：GPIO、TIM、ADC、SPI、USART、FDCAN、DMA 等硬件外设
- **中间件层**：FreeRTOS 任务调度、USB Device CDC
- **任务层 (APP Layer)**：
  - `chassisL_task` / `chassisR_task`：处理 FDCAN 指令、控制左右腿电机，并负责高频组包发送全车 10 个电机状态至上位机。
  - `INS_task`：读取 BMI088 IMU，执行位姿估计算法，并通过 USB 下发传感器四元数/欧拉角。
  - `protocol_task`：通过环形缓冲区 (Ring Buffer) 结合状态机，彻底消除粘包/长包断帧问题，稳定解析 USB 虚拟串口收到的 MAVLink V2 下发报文。
  - `body_task` / `ps2_task` / `vbus_check`：处理机身决策逻辑、PS2手柄通讯、核心电源监测。
- **算法层**：包含 EKF、Mahony、Kalman 滤波以及 PID 等控制求解。
- **控制层**：运动控制规划、连杆 VMC 以及底盘运动模型的具体实现。

## 3. 上下位机通信协议 (MAVLink V2 Lite)

固件通过 USB 虚拟串口 (CDC) 与 ROS 2 上位机使用紧凑的二进制 MAVLink V2 Lite 协议进行全双工通信，具有极高带宽高频能力。
- **基础数据帧**: `10 Byte 帧头 (STX=0xFD)` + `动态长度 Payload` + `2 Byte X.25 CRC 校验`。
- **上位机发 -> 下位机收 (控制下发)**:
  - **MsgID 0x02**: 下发全部 10 个关节电机的目标位置（共 20 字节 Payload），高频闭环流。
  - **MsgID 0xFF**: 标定某一个/所有电机的当前位置为零点（共 3 字节 Payload），低频服务指令。
- **下位机发 -> 上位机收 (状态回传)**:
  - **MsgID 0x03**: 电机状态信息回传，打包 10 个电机的当前位置与速度（共 40 字节 Payload）。由包含左腿/右腿任务在循环内维持 100Hz 甚至 500Hz 发送频率。
  - **MsgID 0x04**: IMU 数据与解算信息回传，打包 float 类型的 4 元数、3 轴角速度和 3 轴线加速度（共 40 字节 Payload）。

## 4. 上位机 ROS 2 桥接节点 (`bridge_node.cpp`)

位于项目根目录下的 `bridge_node.cpp` 提供了一个高性能的 ROS 2 C++ 接口节点：
- **ROS 话题输入端 (控制引脚)**:
  - 控制话题：`/motor_cmds` (`std_msgs/msg/Float64MultiArray`)，发送包含 10维 位置（Float64）的数据。
  - 清零话题：`/set_zero_cmd` (`std_msgs/msg/Int32`)
  - 清零服务：`/set_zero_position` (`std_srvs/srv/Empty`)
- **ROS 话题发布端 (反馈引脚)**:
  - 机器状态：`/joint_states` (`sensor_msgs/msg/JointState`)，将收到的 16位 无符号整型映射为真实的位置速度并发布。
  - 传感器层：`/imu/data` (`sensor_msgs/msg/Imu`)。

## 5. 目录说明

- `Core/`：CubeMX 生成的主工程代码（启动文件、中断、外设初始化）
- `Startup/`：启动汇编文件
- `Drivers/`：CMSIS 与 STM32 HAL 驱动库
- `Middlewares/`：FreeRTOS 与 USB Device 中间件
- `USB_DEVICE/`：USB CDC 设备实现
- `User/APP/`：业务任务与系统任务
- `User/Bsp/`：板级支持包（DWT、CAN、PWM、串口等）
- `User/Devices/`：外设设备驱动（如 BMI088、电机）
- `User/Algorithm/`：滤波与控制算法实现
- `User/Controller/`：控制器逻辑
- `User/Lib/`：通用工具函数

## 4. 构建环境

建议安装：

- `arm-none-eabi-gcc` 工具链
- `cmake`（当前 `CMakeLists.txt` 要求 4.0+）
- `ninja` 或 `make`
- `openocd`（可选，用于下载调试）

## 5. 构建方法

### 5.1 命令行构建

在项目根目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

输出文件：

- `build/CtrlBoard-H7_IMU.elf`
- `build/CtrlBoard-H7_IMU.hex`
- `build/CtrlBoard-H7_IMU.bin`

### 5.2 CLion 构建

- 直接打开工程根目录
- 选择 CMake Profile（Debug/Release）
- 确认 Toolchain 指向 `arm-none-eabi-*`
- 点击 Build 生成固件

## 6. 下载与调试

仓库提供 OpenOCD 配置文件：

- `stm32h723.cfg`
- `stm32h723_dap.cfg`

示例：

```bash
openocd -f stm32h723.cfg
```

启动 OpenOCD 后，可通过 GDB 或 IDE 进行下载、断点调试和在线查看。

## 7. 开发注意事项

- 本工程包含大量 CubeMX 自动生成代码，重新生成后请重点检查 `USER CODE BEGIN/END` 区域是否完整保留。
- 修改系统时钟、定时器、CAN 波特率等底层配置后，建议同步回归检查控制周期与通信时序。
- `CMakeLists.txt` 由模板生成，文件头已标注自动生成，修改前请确认团队流程。

## 8. 常见问题

- 编译器找不到：确认 `arm-none-eabi-gcc` 已加入 PATH。
- CMake 配置失败：检查 CMake 版本与工具链路径。
- 烧录连接失败：确认调试器连接、供电、电缆和 OpenOCD 配置文件匹配。

## 9. License

本项目使用 **MIT License**。详情请参阅项目根目录下的 [LICENSE](LICENSE) 文件。
对于引入的第三方组件（如 FreeRTOS、STM32 HAL 库），请遵循其各自的许可证声明。