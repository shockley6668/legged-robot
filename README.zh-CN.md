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

- 底层驱动层：GPIO、TIM、ADC、SPI、USART、FDCAN、DMA 等硬件外设
- 中间件层：FreeRTOS、USB Device CDC
- 任务层：机体控制、左右腿控制、IMU 处理、协议收发、电压检测、手柄输入等
- 算法层：状态估计与滤波、控制算法
- 控制层：运动控制与执行器控制接口

## 3. 目录说明

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

如仓库未单独提供项目级 License，请按源码文件头声明及第三方组件各自许可证要求使用。