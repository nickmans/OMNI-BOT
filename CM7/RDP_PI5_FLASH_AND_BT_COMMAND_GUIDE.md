# CM7 Operator Guide: Pi5 RDP Flash + Bluetooth Commands

This guide is for the workflow where:

- STM32 board is physically connected to the Pi5 via ST-LINK
- You remote into the Pi5 over RDP
- You flash firmware by running `build_flash_cm7.sh`
- You command the robot over Bluetooth device name: **OMNI-BOT-BLE**

---

## 1) Flash CM7 firmware from Pi5 over RDP

### A. Connect to Pi5 desktop

1. Start your RDP client from your laptop/PC.
2. Connect to the Pi5 desktop session.
3. Open a terminal on the Pi5.

### B. Go to CM7 firmware folder

```bash
cd /home/nickolas/OMNI-BOT/CM7
```

(Optional, if using git-based updates)

```bash
git pull
```

### C. Build + flash in one command

```bash
./build_flash_cm7.sh
```

What this script does:

1. `cd Debug`
2. `make all -j$(nproc)`
3. Programs `Debug/OMNI-BOT_CM7.elf` using OpenOCD + ST-LINK
4. Verifies and resets MCU

### D. Expected successful output cues

- Build completes without compile/link errors.
- OpenOCD prints successful `program`, `verify`, and `reset` completion.
- Command exits with status 0.

---

## 2) If flashing fails on Pi5

### ST-LINK not detected / USB permission issue

Check probe visibility:

```bash
lsusb | grep -i st
```

Check tool install:

```bash
openocd --version
arm-none-eabi-gcc --version
```

If permissions are wrong, add user groups and re-login:

```bash
sudo usermod -a -G dialout,plugdev $USER
```

### Build failure in `Debug/`

```bash
cd /home/nickolas/OMNI-BOT/CM7/Debug
make clean
make all -j$(nproc)
```

Then retry:

```bash
cd /home/nickolas/OMNI-BOT/CM7
./build_flash_cm7.sh
```

---

## 3) Bluetooth command operation (`OMNI-BOT-BLE`)

Firmware command parser runs on UART1 at **115200 8N1** and accepts one command per line.

### A. Phone/tablet app setup

1. Pair/connect to BLE module named **OMNI-BOT-BLE**.
2. Open a serial/BLE terminal app.
3. Set line ending to **LF** (`\n`) or **CRLF**.
4. Send commands as text lines.

### B. Command formats

Both are supported:

- Numeric ID: e.g. `3`
- Name + args: e.g. `traj 1`

(Each must end with newline.)

### C. Command reference

- `0` or `hello` → heartbeat test (toggles LED, replies `hello`)
- `1` or `reset` → MCU reset
- `2` or `help` → list commands
- `3` or `Stop` → stop body motion command
- `4` or `Slow` → set slow translational speed
- `5` or `Medium` → set medium translational speed
- `6 <deg>` or `dir <deg>` → set move direction in degrees
- `7 <yaw_rate>` or `w <yaw_rate>` → set yaw rate command
- `8 <0|1>` or `traj <0|1>`
  - `traj 1` start trajectory-follow mode (requests Pi START_TRAJ)
  - `traj 0` stop trajectory-follow mode
- `9 2` or `traj2 2` → request Pi ROS2 start/restart
- `10` or `shutdown` → request Pi5 shutdown
- `11 <wheel 1..3> <rpm>` or `wtest <wheel> <rpm>` → single-wheel RPM test mode
- `11 off` or `wtest off` → exit wheel test mode
- `12 <mode>` or `map <mode>` mapping control:
  - `map 1` start mapping
  - `map 0` finish mapping
  - `map 2` use live map
  - `map 3` use frozen map

### D. Typical operator command sequence

#### Bringup and trajectory mode

```text
help
traj2 2
traj 1
```

#### Stop trajectory mode

```text
traj 0
Stop
```

#### Wheel bench test

```text
wtest 1 30
wtest 2 30
wtest 3 30
wtest off
```

#### Mapping workflow

```text
map 1
map 0
map 2
```

---

## 4) Safety checklist for command operation

- Lift robot wheels off floor for first command test after flashing.
- Start with `Stop`, then low-speed commands.
- Use `traj 0` + `Stop` before disconnecting app.
- Keep manual E-stop/kill power path available.

---

## 5) One-screen quick routine (RDP + BT)

1. RDP into Pi5.
2. Run:
   ```bash
   cd /home/nickolas/OMNI-BOT/CM7 && ./build_flash_cm7.sh
   ```
3. Connect phone to **OMNI-BOT-BLE**.
4. Send:
   ```text
   help
   traj2 2
   traj 1
   ```
5. When done:
   ```text
   traj 0
   Stop
   ```
