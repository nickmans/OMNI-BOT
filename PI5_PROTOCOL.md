# Pi5 Ethernet Protocol Documentation

## Overview
This document describes the communication protocol between the STM32H7 robot controller and the Raspberry Pi 5 navigation computer over Ethernet TCP/IP.

## Connection Details
- **Server**: Raspberry Pi 5
- **Default IP**: 192.168.1.100
- **Default Port**: 9000
- **Protocol**: TCP
- **STM32 sends** pose data at 5 Hz (200ms period)
- **Pi5 sends** trajectory commands when available

## Message Format

All messages use **little-endian** byte order.

### Message Header (24 bytes)
```c
struct eth_msg_header {
    uint32_t magic;         // 0x4F4D4E49 ('OMNI')
    uint16_t version;       // 1
    uint16_t msg_type;      // See message types below
    uint32_t seq;           // Sequence number
    uint32_t t_ms;          // Timestamp in milliseconds
    uint32_t payload_len;   // Payload length in bytes
    uint32_t crc32;         // CRC32 (0 for no CRC)
};
```

### Message Types
- `1` - POSE message (STM32 → Pi5)
- `10` - TRAJECTORY message (Pi5 → STM32)
- `20` - COMMAND message (STM32 → Pi5)

## STM32 → Pi5: POSE Message

**Total Size**: 52 bytes (24 byte header + 28 byte payload)

**Payload Structure** (28 bytes):
```c
struct eth_pose_payload {
    uint32_t pose_t_ms;     // Pose timestamp in milliseconds
    float x;                // Position x in meters
    float y;                // Position y in meters
    float yaw;              // Orientation in radians [-pi, pi]
    float vx;               // Velocity x in m/s (world frame)
    float vy;               // Velocity y in m/s (world frame)
    float wz;               // Angular velocity in rad/s
};
```

**Sent**: Every 200ms (5 Hz) when connected

## Pi5 → STM32: TRAJECTORY Message

**Total Size**: 44 bytes (24 byte header + 20 byte payload)

**Payload Structure** (20 bytes):
```c
struct eth_traj_payload {
    float x_des;            // Desired position x in meters
    float y_des;            // Desired position y in meters
    float yaw_des;          // Desired orientation in radians
    float vx_world;         // Feedforward velocity x in m/s (world frame)
    float vy_world;         // Feedforward velocity y in m/s (world frame)
};
```

**Received**: When Pi5 has new trajectory setpoint

## STM32 → Pi5: COMMAND Message

**Total Size**: 28 bytes (24 byte header + 4 byte payload)

**Payload Structure** (4 bytes):
```c
struct eth_cmd_payload {
    uint32_t command;       // Command code
};
```

**Command Codes**:
- `1` - START_TRAJ: Start trajectory following (launch ROS2 node)
- `2` - STOP_TRAJ: Stop trajectory following

## Connection Behavior

### STM32 Behavior
1. Attempts connection every 10 seconds if not connected
2. Once connected, sends POSE messages at 5 Hz
3. Listens for TRAJECTORY messages from Pi5
4. On connection error, waits 10 seconds before retry
5. Continues robot operation even when disconnected (graceful degradation)

### Pi5 Server Expected Behavior
1. Listen on configured port (default 9000)
2. Accept incoming STM32 connection
3. Receive POSE messages continuously
4. Send TRAJECTORY messages when new setpoint available
5. Close connection gracefully when done

## Data Types and Alignment

All structures use:
- **Packed layout** (`__attribute__((packed))`)
- **Little-endian** byte order
- **4-byte alignment** for buffers to avoid hard faults

Float values are transmitted as IEEE 754 single-precision (32-bit).

## Example Message Flow

```
STM32                          Pi5
  |                             |
  |--- TCP Connect ------------->|
  |<-- TCP Accept ---------------|
  |                             |
  |--- POSE (seq=0) ------------>|
  |--- POSE (seq=1) ------------>|
  |<-- TRAJ --------------------|  (Pi5 has new trajectory)
  |--- POSE (seq=2) ------------>|
  |--- POSE (seq=3) ------------>|
  ...
```

## Implementation Notes

### Current Simplified Implementation
- Uses LWIP netconn API (not raw API)
- Blocking operations in dedicated thread
- 10-second reconnection attempts
- Simple polling approach
- Avoids complex callbacks and state machines

### Thread Safety
- Pose data protected by mutex
- Trajectory data protected by mutex
- Non-blocking mutex acquisition (timeout=0) in control loops

### Robustness Features
- Automatic reconnection
- Graceful degradation when Pi5 unavailable
- No blocking of robot control loops
- Memory-aligned buffers to prevent hard faults

## Testing

### Test on Pi5
```python
import socket
import struct

# Create server
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('192.168.1.100', 9000))
sock.listen(1)
print("Waiting for STM32...")

conn, addr = sock.accept()
print(f"Connected: {addr}")

while True:
    # Receive POSE message (52 bytes)
    data = conn.recv(52)
    if len(data) == 52:
        magic, ver, msg_type, seq, t_ms, payload_len, crc = struct.unpack('<IHHIIII', data[:24])
        pose_t, x, y, yaw, vx, vy, wz = struct.unpack('<Iffffff', data[24:52])
        print(f"POSE {seq}: x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
```

### Send Trajectory from Pi5
```python
# Build TRAJ message
header = struct.pack('<IHHIIII', 
    0x4F4D4E49,  # magic
    1,           # version
    10,          # TRAJ message type
    0,           # seq
    0,           # timestamp
    20,          # payload length
    0)           # crc
    
payload = struct.pack('<fffff',
    1.0,   # x_des
    2.0,   # y_des
    0.5,   # yaw_des
    0.1,   # vx_world
    0.1)   # vy_world

conn.send(header + payload)
```

## Troubleshooting

### Hard Fault Issues
- **Cause**: Unaligned memory access
- **Solution**: Use aligned buffers (`__attribute__((aligned(4)))`)
- **Solution**: Ensure all pointers are properly aligned

### Connection Failures
- **Check**: Pi5 server is running
- **Check**: IP addresses match
- **Check**: Network cable connected
- **Check**: Firewall settings on Pi5

### No Trajectory Updates
- **Check**: Pi5 is sending TRAJ messages
- **Check**: Message format matches specification
- **Check**: Magic number and version are correct

## Version History

- **Version 1** (Feb 2026): Initial protocol definition
- **Simplified Implementation** (Feb 2026): Removed complex callbacks, uses netconn API
