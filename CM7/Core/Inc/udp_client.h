#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <stdint.h>
#include <stdbool.h>

#define PI5_IP_ADDR    "192.168.1.100"
#define PI5_PORT       9000

#define STM32_IP_ADDR  "192.168.1.10"
#define NETMASK_ADDR   "255.255.255.0"
#define GATEWAY_ADDR   "192.168.1.1"

#define MAGIC   0x4F4D4E49u
#define VERSION 1u
#define HEADER_SIZE 24u

typedef enum {
    MSG_TYPE_POSE = 1,
    MSG_TYPE_TRAJ = 10,
    MSG_TYPE_CMD  = 20
} MessageType;

typedef enum {
    CMD_STOP_ROS2  = 0,
    CMD_START_TRAJ = 1,
    CMD_STOP_TRAJ  = 2
} CommandID;

void UDP_Client_Task(void *argument);
void UDP_Client_RequestCmd(CommandID cmd_id);
void UDP_Client_SendZeroPose(void);

bool UDP_Client_CopyLatestTraj(void *out_buf, uint32_t buf_len, uint16_t *out_knots, float *out_dt, uint16_t *out_flags);

#endif
