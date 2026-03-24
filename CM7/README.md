# OMNI-BOT CM7 Firmware

This folder contains the STM32H755 CM7 firmware for motor control, sensing, command parsing, networking, and real-time robot execution.

## What this firmware does

- Runs the main real-time control loop on **CM7** (FreeRTOS-based).
- Reads wheel encoders and IMU data.
- Computes robot state and control outputs.
- Drives 3 motor channels (PWM + direction).
- Exchanges trajectory/command data with the Pi5 over Ethernet UDP.

## Folder overview

- `Core/Inc`, `Core/Src` — application code (control, command, sensors, networking)
- `LWIP/` — Ethernet/LwIP integration
- `Debug/` — generated build output (`OMNI-BOT_CM7.elf`, object files, makefiles)
- `build_flash_cm7.sh` — one-command build + flash via ST-LINK/OpenOCD
- `STM32H755ZITX_FLASH.ld`, `STM32H755ZITX_RAM.ld` — linker scripts

## Key modules

- `Core/Src/main.c`
  - HAL + clock + peripheral bring-up
  - UART RX interrupt rearm logic
  - timer/PWM/encoder initialization
  - FreeRTOS task start
- `Core/Src/controller.c` / `Core/Inc/controller.h`
  - motion/controller step
  - state estimator integration
- `Core/Src/cmd.c` / `Core/Inc/cmd.h`
  - serial command parser and motor command interfaces
  - PWM conversion and limits
- `Core/Src/udp_client.c` / `Core/Inc/udp_client.h`
  - UDP protocol to Pi5 (`192.168.1.100:9000`)
  - sends pose, receives trajectory/commands
- `Core/Src/IMU.c`, `Core/Src/sensor.c`, `Core/Src/battery_monitor.c`
  - IMU decode, sensor processing, battery monitoring

## Network contract (CM7 ↔ Pi5)

Defaults from `udp_client.h`:

- STM32 static IP: `192.168.1.10`
- Pi5 server IP: `192.168.1.100`
- UDP port: `9000`
- Shared header magic/version used for packet validation

If network settings change, update both CM7 and Pi server sides.

## Prerequisites

Install required tools on development machine:

- `arm-none-eabi-gcc` toolchain
- `make`
- `openocd`
- ST-LINK connected to target board

Quick checks:

```bash
arm-none-eabi-gcc --version
openocd --version
```

## Build and flash

### Recommended (one command)

From this folder:

```bash
./build_flash_cm7.sh
```

Script behavior:

1. Builds in `Debug/` (`make all -j$(nproc)`)
2. Programs `Debug/OMNI-BOT_CM7.elf` via OpenOCD
3. Verifies and resets the MCU

### Build only

```bash
cd Debug
make all -j$(nproc)
```

## Typical bring-up flow

1. Build/flash CM7 firmware.
2. Ensure Pi server is running and reachable at `192.168.1.100:9000`.
3. Power robot and verify encoder/IMU telemetry is updating.
4. Confirm trajectory messages are received from Pi (UDP path healthy).
5. Validate wheel response at low speed before full-motion testing.

## Command mode transitions (`traj` / `map`)

Current STM32 behavior:

- `traj 1`: requests trajectory generation/recording and **keeps manual mode enabled** (`traj_mode=0`) so the robot can still be driven while mapping.
- `map 0`: finishes mapping and switches to **trajectory-follow mode only** (`traj_mode=1`).
- `traj 0`: returns to **manual mode** (`traj_mode=0`).

This sequence is intended for "drive while mapping, then lock to trajectory" workflow.

## Troubleshooting

### Flash fails / cannot connect ST-LINK

- Check USB cable and target power.
- Ensure no other debugger session is holding the probe.
- Retry OpenOCD command from `build_flash_cm7.sh` manually to inspect detailed errors.

### Build errors in `Debug/`

- Clean and rebuild:

```bash
cd Debug
make clean
make all -j$(nproc)
```

- Verify toolchain binaries are in `PATH`.

### No trajectory coming from Pi

- Verify Pi server is running and bound to the expected port.
- Confirm CM7 and Pi IP settings match expected subnet.
- Check Ethernet link and switch/cable.

### Robot not moving as expected

- Verify command mode (`traj_mode` vs manual command path).
- Check encoder sign/direction conventions and wheel sign constants in `cmd.h`.
- Validate IMU yaw feed and estimator outputs before tuning controller gains.

## Safety notes

- Start with robot lifted or wheels clear of ground for first motion tests.
- Use conservative speed/acceleration limits during tuning.
- Keep an emergency stop path available during bring-up.

## Related workspace

- CM7 firmware here: `/home/nickolas/OMNI-BOT/CM7`
- ROS2 + Pi stack: `/home/nickolas/ros2_ws/src/omni_src`

Use CM7 + Pi/ROS2 together for full closed-loop operation.

## Operator guide

For day-to-day Pi5 remote flashing and Bluetooth control workflow, see:

- `RDP_PI5_FLASH_AND_BT_COMMAND_GUIDE.md`
