# OMNI CM7 Firmware

This directory contains the active STM32H755 CM7 runtime for the robot.

CM4 project files exist elsewhere in the repository, but CM4 is not the normal runtime path for the robot. The checked-in robot runtime is CM7.

## What CM7 does

The CM7 firmware handles the real-time side of the robot:

- 3-wheel kiwi / omni control
- state estimation
- motor PWM output
- encoder processing
- IMU integration
- UDP communication with the Raspberry Pi
- command shell handling
- battery voltage monitoring and RPM limiting

Normal boot behavior is manual mode:

- `traj_mode = 0` on boot
- cached trajectories are invalidated before entering `traj 1` and `traj 3`

## Major modules

These are the main checked-in CM7 source modules:

- `Core/Src/main.c`
  - system bringup
  - task creation
  - 100 Hz `remote()` control loop
  - 10 Hz pose heartbeat to the Pi

- `Core/Src/cmd.c`
  - USART1 command shell
  - `traj`, `traj2`, `map`, `wp`, `term`, `shutdown` commands

- `Core/Src/controller.c`
  - main motion controller

- `Core/Src/state_estimator.c`
  - robot state estimation used for outgoing pose and control

- `Core/Src/PWM.c`
  - motor output handling

- `Core/Src/udp_client.c`
  - lwIP UDP client
  - command send/ack handling
  - latest trajectory cache

- `Core/Src/IMU.c`
  - BNO RVC IMU handling

- `Core/Src/battery_monitor.c`
  - ADC-based battery measurement
  - scales max RPM by battery voltage


## Runtime interfaces

### USART1 command shell

- Command shell path: `USART1`
- Initialized by `CMD_Init(&huart1)` in `main.c`
- Used for remote/manual commands and mode changes

### USART2 IMU path

- IMU path: `USART2`
- Uses the `BNO_RVC_*` API in `IMU.c`

### Ethernet / UDP path

- lwIP UDP client in `udp_client.c`
- Pi defaults:
  - `192.168.1.100:9000`
- STM32 default IP:
  - `192.168.1.10`

### Motion I/O

- encoder timers feed the estimator and controller
- PWM motor outputs drive the 3-wheel platform

## Runtime timing

Checked-in timing in the CM7 runtime:

- control loop in `remote()` runs at `100 Hz`
- pose heartbeat to the Pi runs at `10 Hz`
- trajectory deadman timeout is `700 ms`

## Pi communication path

Normal data flow with the Pi is:

1. CM7 estimates pose.
2. CM7 sends `POSE` packets to the Pi.
3. Pi publishes pose into ROS 2.
4. Pi planner publishes `/planned_path` and `/planned_path_velocities`.
5. Pi UDP server packages those into `TRAJ` packets.
6. CM7 follows the incoming trajectory when `traj_mode = 1`.

## Shell commands

These are the main checked-in shell commands and meanings.

- `traj 0`
  - manual / standby mode
  - stop consuming cached trajectories immediately

- `traj 1`
  - autonomous localization mode
  - Pi localizes and streams trajectories back to the STM32
  - invalidates old trajectories before switching

- `traj 3`
  - autonomous mode using blank global map behavior on the Pi
  - local obstacle avoidance stays live on the Pi side
  - invalidates old trajectories before switching

- `traj2 2`
  - manual drive on STM32 while Pi runs localization
  - trajectory following disabled on STM32

- `map 1`
  - start mapping mode on the Pi
  - disables trajectory following on STM32

- `map 0`
  - finish mapping and return to autonomous localization mode

- `map 2`
  - switch Pi to live-map mode

- `map 3`
  - switch Pi to frozen/localization mode

- `wp t`
  - request centered waypoint test pattern generation on the Pi

- `term`
  - start Pi terminal passthrough

- `shutdown`
  - request Pi shutdown

## Build and flash

Use the checked-in helper script:

```bash
cd CM7
./build_flash_cm7.sh
```

That script builds the `Debug` target and flashes `Debug/OMNI-BOT_CM7.elf` through OpenOCD.

## Safety notes

- Start with the robot lifted or otherwise off the ground.
- Verify wheel sign conventions before commanding larger motion.
- Verify low-speed response first.
- Confirm that `traj 0` gives you predictable manual behavior before enabling `traj 1` or `traj 3`.

