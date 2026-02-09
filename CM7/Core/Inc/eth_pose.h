/*
 * eth_pose.h
 *
 *  Created on: Feb 4, 2026
 *      Author: GitHub Copilot
 */

#ifndef INC_ETH_POSE_H_
#define INC_ETH_POSE_H_

#include <stdint.h>
#include <stdbool.h>

// Protocol constants
#define ETH_POSE_MAGIC      0x4F4D4E49  // 'OMNI'
#define ETH_POSE_VERSION    1
#define ETH_MSG_TYPE_POSE   1
#define ETH_MSG_TYPE_TRAJ   10
#define ETH_MSG_TYPE_CMD    20  // Control commands

// Message header (24 bytes) - NOT USED (manual packing instead)
// Kept for documentation only
typedef struct {
    uint32_t magic;         // 0x4F4D4E49 ('OMNI')
    uint16_t version;       // 1
    uint16_t msg_type;      // 1=POSE, 10=TRAJ
    uint32_t seq;           // Sequence number
    uint32_t t_ms;          // Timestamp in milliseconds
    uint32_t payload_len;   // Payload length in bytes
    uint32_t crc32;         // CRC32 (0 for no CRC)
} eth_msg_header_t;

// POSE payload (28 bytes) - NOT USED (manual packing instead)
// Kept for documentation only
typedef struct {
    uint32_t pose_t_ms;     // Pose timestamp in milliseconds
    float x;                // Position x in meters
    float y;                // Position y in meters
    float yaw;              // Orientation in radians [-pi, pi]
    float vx;               // Velocity x in m/s
    float vy;               // Velocity y in m/s
    float wz;               // Angular velocity in rad/s
} eth_pose_payload_t;

// TRAJ payload (20 bytes) - NOT USED (manual packing instead)
// Kept for documentation only
typedef struct {
    float x_des;            // Desired position x in meters
    float y_des;            // Desired position y in meters
    float yaw_des;          // Desired orientation in radians
    float vx_world;         // Feedforward velocity x in m/s (world frame)
    float vy_world;         // Feedforward velocity y in m/s (world frame)
} eth_traj_payload_t;

// CMD payload (4 bytes) - NOT USED (manual packing instead)
// Kept for documentation only
typedef struct {
    uint32_t command;       // Command code
} eth_cmd_payload_t;

// Command codes
#define CMD_START_TRAJ  1   // Start trajectory following (launch ROS2 node)
#define CMD_STOP_TRAJ   2   // Stop trajectory following

// Complete POSE message (52 bytes) - NOT USED (manual packing instead)
// Kept for documentation only
typedef struct {
    eth_msg_header_t header;
    eth_pose_payload_t payload;
} eth_pose_msg_t;

// Complete TRAJ message (44 bytes) - NOT USED (manual packing instead)
// Kept for documentation only
typedef struct {
    eth_msg_header_t header;
    eth_traj_payload_t payload;
} eth_traj_msg_t;

// Complete CMD message (28 bytes) - NOT USED (manual packing instead)  
// Kept for documentation only
typedef struct {
    eth_msg_header_t header;
    eth_cmd_payload_t payload;
} eth_cmd_msg_t;

// Trajectory setpoint (what controller needs)
typedef struct {
    float x_des;            // Desired position x in meters
    float y_des;            // Desired position y in meters
    float yaw_des;          // Desired orientation in radians
    float vx_world;         // Feedforward velocity x in m/s (world frame)
    float vy_world;         // Feedforward velocity y in m/s (world frame)
    uint32_t timestamp_ms;  // When received
    bool valid;             // Whether trajectory is valid/received
} trajectory_setpoint_t;

// Robot pose state
typedef struct {
    double x;               // Position x in meters
    double y;               // Position y in meters
    double yaw;             // Orientation in radians
    double vx;              // Velocity x in m/s (world frame)
    double vy;              // Velocity y in m/s (world frame)
    double wz;              // Angular velocity in rad/s
    uint32_t timestamp_ms;  // Timestamp in milliseconds
} robot_pose_t;

// Configuration
typedef struct {
    const char* server_ip;  // Pi server IP address
    uint16_t server_port;   // Pi server port
    uint32_t send_period_ms;// Send period in milliseconds (200ms = 5Hz)
} eth_pose_config_t;

// Initialize Ethernet POSE communication
void ETH_POSE_Init(const eth_pose_config_t* config);

// Start the Ethernet POSE thread
void ETH_POSE_StartThread(void);

// Update robot pose (thread-safe)
void ETH_POSE_UpdatePose(const robot_pose_t* pose);

// Enable/disable trajectory following mode (sends command to Pi)
void ETH_POSE_SetTrajectoryMode(bool enable);

// Get trajectory mode status
bool ETH_POSE_GetTrajectoryMode(void);

// Get current robot pose (thread-safe)
void ETH_POSE_GetPose(robot_pose_t* pose);

// Get latest trajectory setpoint (thread-safe)
bool ETH_POSE_GetTrajectory(trajectory_setpoint_t* traj);

// Get connection status
bool ETH_POSE_IsConnected(void);

#endif /* INC_ETH_POSE_H_ */
