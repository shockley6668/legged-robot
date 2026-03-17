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

- Low-level drivers: GPIO, TIM, ADC, SPI, USART, FDCAN, DMA
- Middleware: FreeRTOS, USB Device CDC
- Task layer: body/chassis control, IMU processing, protocol handling, voltage check, controller input
- Algorithm layer: estimation and filtering
- Controller layer: motion/control outputs

## 3. Directory Layout

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

## 4. Build Environment

Recommended tools:

- `arm-none-eabi-gcc`
- `cmake` (current `CMakeLists.txt` requires 4.0+)
- `ninja` or `make`
- `openocd` (optional, for flashing/debugging)

## 5. Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
```

Artifacts:

- `build/CtrlBoard-H7_IMU.elf`
- `build/CtrlBoard-H7_IMU.hex`
- `build/CtrlBoard-H7_IMU.bin`

## 6. Flash and Debug

Provided OpenOCD configs:

- `stm32h723.cfg`
- `stm32h723_dap.cfg`

Example:

```bash
openocd -f stm32h723.cfg
```

Then connect with GDB/IDE for flashing and debugging.

## 7. Notes

- This project contains CubeMX-generated code; always verify `USER CODE BEGIN/END` sections after regeneration.
- If clock/timer/CAN settings are changed, re-check control loop timing and communication timing.
- `CMakeLists.txt` is template-generated; follow your team flow before editing it.

## 8. License

If no project-level license file is provided, follow source file headers and each third-party component license.