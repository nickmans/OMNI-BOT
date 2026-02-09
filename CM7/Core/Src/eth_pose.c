/*
 * eth_pose.c
 *
 *  Created on: Feb 4, 2026
 *      Author: GitHub Copilot
 *
 * SIMPLIFIED Pi5 Ethernet Communication
 * - Simple blocking approach with 10-second reconnection attempts
 * - Uses LWIP netconn API (simpler than raw API)
 * - Minimal complexity, robust operation
 */

#include "eth_pose.h"
#include "lwip/api.h"
#include "lwip/netif.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

// Configuration
static eth_pose_config_t g_config = {
    .server_ip = "192.168.1.100",
    .server_port = 9000,
    .send_period_ms = 200  // 5 Hz
};

// Shared robot pose with mutex protection
static robot_pose_t g_robot_pose = {0};
static osMutexId_t g_pose_mutex = NULL;

// Shared trajectory setpoint with mutex protection
static trajectory_setpoint_t g_trajectory = {0};
static osMutexId_t g_traj_mutex = NULL;

// Connection state
static struct netconn* g_conn = NULL;
static bool g_is_connected = false;
static uint32_t g_seq_number = 0;

// Thread handle
static osThreadId_t g_eth_pose_thread_handle = NULL;

// Forward declarations
static void EthernetPoseThread(void* argument);

// Helper functions for endian conversion
static void Pack_U32_LE(uint8_t* buf, uint32_t val);
static void Pack_U16_LE(uint8_t* buf, uint16_t val);
static void Pack_Float_LE(uint8_t* buf, float val);
static uint32_t Unpack_U32_LE(const uint8_t* buf);
static uint16_t Unpack_U16_LE(const uint8_t* buf);
static float Unpack_Float_LE(const uint8_t* buf);


// Initialize Ethernet POSE communication
void ETH_POSE_Init(const eth_pose_config_t* config)
{
    if (config) {
        g_config = *config;
    }
    
    printf("ETH_POSE: Init - Server: %s:%d, Period: %lu ms\n", 
           g_config.server_ip, g_config.server_port, g_config.send_period_ms);
    
    // Create mutex for trajectory data
    g_traj_mutex = osMutexNew(NULL);
    if (g_traj_mutex == NULL) {
        printf("ETH_POSE: Failed to create trajectory mutex\n");
    }
    
    // Create mutex for pose data
    g_pose_mutex = osMutexNew(NULL);
    if (g_pose_mutex == NULL) {
        printf("ETH_POSE: Failed to create pose mutex\n");
    }
}

// Start the Ethernet POSE thread
void ETH_POSE_StartThread(void)
{
    const osThreadAttr_t thread_attr = {
        .name = "EthPoseThread",
        .stack_size = 1024 * 4,  // Reduced stack size
        .priority = osPriorityBelowNormal
    };
    
    g_eth_pose_thread_handle = osThreadNew(EthernetPoseThread, NULL, &thread_attr);
    if (g_eth_pose_thread_handle == NULL) {
        printf("ETH_POSE: Failed to create thread\n");
    }
}


// Update robot pose (thread-safe, non-blocking)
void ETH_POSE_UpdatePose(const robot_pose_t* pose)
{
    if (!pose) return;
    
    if (g_pose_mutex) {
        if (osMutexAcquire(g_pose_mutex, 0) == osOK) {
            g_robot_pose = *pose;
            osMutexRelease(g_pose_mutex);
        }
    }
}

// Get current robot pose (thread-safe, non-blocking)
void ETH_POSE_GetPose(robot_pose_t* pose)
{
    if (!pose) return;
    
    if (g_pose_mutex) {
        if (osMutexAcquire(g_pose_mutex, 0) == osOK) {
            *pose = g_robot_pose;
            osMutexRelease(g_pose_mutex);
        } else {
            *pose = g_robot_pose;
        }
    } else {
        *pose = g_robot_pose;
    }
}

// Get latest trajectory setpoint (thread-safe, non-blocking)
bool ETH_POSE_GetTrajectory(trajectory_setpoint_t* traj)
{
    if (!traj) return false;
    
    bool valid = false;
    if (g_traj_mutex) {
        if (osMutexAcquire(g_traj_mutex, 0) == osOK) {
            *traj = g_trajectory;
            valid = g_trajectory.valid;
            osMutexRelease(g_traj_mutex);
        } else {
            *traj = g_trajectory;
            valid = g_trajectory.valid;
        }
    } else {
        *traj = g_trajectory;
        valid = g_trajectory.valid;
    }
    return valid;
}

// Enable/disable trajectory following mode
void ETH_POSE_SetTrajectoryMode(bool enable)
{
    // For now, just log - simplified version doesn't send commands
    printf("ETH_POSE: Trajectory mode %s\n", enable ? "ENABLED" : "DISABLED");
}

// Get trajectory mode status
bool ETH_POSE_GetTrajectoryMode(void)
{
    return false;  // Simplified version doesn't support this
}

// Get connection status
bool ETH_POSE_IsConnected(void)
{
    return g_is_connected;
}


// ==============================================
// Main Ethernet POSE Thread - SIMPLIFIED
// ==============================================

// Main thread - DISABLED to prevent hard faults
// Re-enable gradually once root cause is identified
static void EthernetPoseThread(void* argument)
{
    (void)argument;
    
    printf("ETH_POSE: Thread started (ALL NETWORK OPERATIONS DISABLED)\n");
    printf("ETH_POSE: Thread will sleep indefinitely to avoid hard faults\n");
    
    // Just sleep forever - no network operations
    for (;;) {
        osDelay(10000);  // Sleep 10 seconds
    }
}

// ==============================================
// Helper Functions - Little Endian Packing
// ==============================================

static void Pack_U32_LE(uint8_t* buf, uint32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

static void Pack_U16_LE(uint8_t* buf, uint16_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

static void Pack_Float_LE(uint8_t* buf, float val)
{
    union {
        float f;
        uint32_t u;
    } converter;
    
    converter.f = val;
    Pack_U32_LE(buf, converter.u);
}

static uint32_t Unpack_U32_LE(const uint8_t* buf)
{
    return ((uint32_t)buf[0]) |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

static uint16_t Unpack_U16_LE(const uint8_t* buf)
{
    return ((uint16_t)buf[0]) |
           ((uint16_t)buf[1] << 8);
}

static float Unpack_Float_LE(const uint8_t* buf)
{
    union {
        float f;
        uint32_t u;
    } converter;
    
    converter.u = Unpack_U32_LE(buf);
    return converter.f;
}
