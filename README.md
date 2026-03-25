# legged-robot

[中文说明 (Chinese)](README.zh-CN.md)

Firmware for a **biped robot control board** based on STM32H723VGTX (Cortex-M7). The project uses STM32 HAL + FreeRTOS and covers sensor acquisition, state estimation, motion control, host communication, and power management.

## 1. Overview

- MCU: STM32H723VGTX
- RTOS: FreeRTOS (CMSIS-RTOS adaptation)
- Communication: FDCAN, USB CDC, USART, Marvlink (host PC)
- Main algorithms: EKF, Mahony, Kalman, PID, VMC
- Main devices: BMI088 IMU, DM motor series

Build target: `CtrlBoard-H7_IMU.elf` (with `.hex` and `.bin` outputs).

## 2. Software Architecture

Layered design:

- **Low-level drivers**: GPIO, TIM, ADC, SPI, USART, FDCAN, DMA
- **Middleware**: FreeRTOS, USB Device CDC
- **APP Layer (Task Layer)**: 
  - `chassisL_task` / `chassisR_task`: Controls left/right leg motors via FDCAN. Periodically packages all 10 motor states and sends them to the PC.
  - `INS_task`: Reads BMI088 IMU, runs attitude estimation algorithms, and streams quaternion/IMU data to the PC.
  - `protocol_task`: Implements a fast ring buffer and state machine to parse incoming USB CDC data (MAVLink V2 Lite) to prevent packet loss, routing commands to motor controllers.
  - `body_task` / `ps2_task`: Higher-level movement logic and PS2 remote controller handling.
- **Algorithm layer**: EKF, Mahony, Kalman, PID, VMC
- **Controller layer**: Joint torque calculations and kinematics

## 3. Communication Protocol (MAVLink V2 Lite)

The firmware communicates with the ROS 2 host PC via USB CDC (Virtual COM Port) using a compact, binary MAVLink V2 Lite protocol.
- **Frame Structure**: `10 bytes Header (STX=0xFD)` + `Payload` + `2 bytes X.25 CRC`.
- **Host -> MCU (Commands)**:
  - **MsgID 0x02**: Target joint positions for all 10 motors (20-byte payload, 32 bytes total). High-frequency stream.
  - **MsgID 0xFF**: Save motor zero-point command (3-byte payload). Low-frequency service call.
- **MCU -> Host (Feedback)**:
  - **MsgID 0x03**: Motor state feedback (Position & Velocity for 10 joints). 40-byte payload, 52 bytes total frame. Streamed at 100Hz~500Hz from the chassis task.
  - **MsgID 0x04**: IMU & Attitude feedback (Float32 Quaternion * 4, Gyro * 3, Accel * 3). 40-byte payload, 52 bytes total frame. Streamed from the INS task.

## 4. ROS 2 Host Node (`bridge_node.cpp`)

Located at the project root, `bridge_node.cpp` is a ROS 2 C++ node that acts as the communication bridge between the MCU's USB serial port and the ROS 2 environment.
- **Robust Serial Driver**: Configured for raw binary transmission. It strictly disables Linux terminal special character conversions (e.g., `0x0D` to `0x0A`) utilizing `<termios.h>` to guarantee payload/CRC integrity. Uses an advanced sliding-window parsing logic to easily handle high-frequency throughput continuously without CPU spikes.
- **Submits Commands (PC -> MCU)**:
  - `/motor_cmds` (`std_msgs/msg/Float64MultiArray`): Expected to contain 10 float values for target joint positions.
  - `/set_zero_cmd` (`std_msgs/msg/Int32`): Motor ID to set as zero point (`-1` for all).
  - Service `/set_zero_position` (`std_srvs/srv/Empty`): Triggers a global zero-point calibration.
- **Publishes Data (MCU -> PC)**:
  - `/joint_states` (`sensor_msgs/msg/JointState`): Real-time positions and velocities of the joints.
  - `/imu/data` (`sensor_msgs/msg/Imu`): Real-time orientation and linear/angular velocities.

## 5. Directory Layout

- `Core/`: CubeMX-generated core code (startup, interrupts, peripheral init)
- `Startup/`: startup assembly
- `Drivers/`: CMSIS and STM32 HAL drivers
- `Middlewares/`: FreeRTOS and USB middleware
- `USB_DEVICE/`: USB CDC device implementation
- `User/APP/`: application/system tasks
- `User/Bsp/`: board support package (DWT, CAN, PWM, UART, etc.)
- `User/Devices/`: device drivers (e.g., BMI088, motors)
- `User/Algorithm/`: estimation and control algorithms
- `User/Controller/`: controller logic
- `User/Lib/`: common utilities

## 6. Build Environment

Recommended tools:

- `arm-none-eabi-gcc`
- `cmake` (current `CMakeLists.txt` requires 4.0+)
- `ninja` or `make`
- `openocd` (optional, for flashing/debugging)

## 7. Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

Artifacts:

- `build/CtrlBoard-H7_IMU.elf`
- `build/CtrlBoard-H7_IMU.hex`
- `build/CtrlBoard-H7_IMU.bin`

## 8. Flash and Debug

Provided OpenOCD configs:

- `stm32h723.cfg`
- `stm32h723_dap.cfg`

Example:

```bash
openocd -f stm32h723.cfg
```

Then connect with GDB/IDE for flashing and debugging.

## 9. Notes

- This project contains CubeMX-generated code; always verify `USER CODE BEGIN/END` sections after regeneration.
- If clock/timer/CAN settings are changed, re-check control loop timing and communication timing.
- `CMakeLists.txt` is template-generated; follow your team flow before editing it.

## 10. License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for details.
For third-party components (such as FreeRTOS, STM32 HAL libraries), please follow their respective license terms.