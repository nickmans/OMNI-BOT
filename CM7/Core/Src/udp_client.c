#include "udp_client.h"
#include "main.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <fcntl.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/etharp.h"
#include "lwip/netif.h"
#include "lwip/errno.h"

#include <string.h>

extern struct netif gnetif;

#ifndef MSG_DONTWAIT
#define MSG_DONTWAIT 0
#endif

#define UDP_TASK_DELAY_MS           5u
#define UDP_POSE_PERIOD_MS          1000u
#define UDP_SOCKET_RETRY_MIN_MS     100u
#define UDP_SOCKET_RETRY_MAX_MS     5000u
#define UDP_CMD_RETRY_COUNT         3u
#define UDP_CMD_RETRY_INTERVAL_MS   100u
#define UDP_RX_BUFFER_SIZE          1500u
#define UDP_MAX_TRAJ_KNOTS          64u

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t msg_type;
    uint32_t seq;
    uint32_t t_ms;
    uint32_t payload_len;
    uint32_t crc32;
} MessageHeader;

typedef struct __attribute__((packed))
{
    uint32_t pose_t_ms;
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float wz;
} PosePayload;

typedef struct __attribute__((packed))
{
    uint32_t reply_to_pose_seq;
    uint32_t traj_t0_ms;
    uint16_t n_knots;
    uint16_t flags;
    uint32_t reserved;
    float dt;
} TrajectoryHeader;

typedef struct __attribute__((packed))
{
    float x;
    float y;
    float yaw;
    float velocity;
} TrajectoryKnot;

typedef struct __attribute__((packed))
{
    uint16_t cmd_id;
    uint16_t arg_len;
} CommandPayload;

extern double x[3];
extern double vd[3];
extern double yawrate;

static int s_udp_socket = -1;
static struct sockaddr_in s_dst_addr;

static uint32_t s_pose_seq = 0;

static volatile bool s_cmd_pending = false;
static volatile uint32_t s_cmd_seq = 0;
static volatile uint16_t s_cmd_id = 0;
static volatile uint32_t s_cmd_last_tx_ms = 0;
static volatile uint32_t s_cmd_retry_left = 0;

static PosePayload s_pending_pose;
static volatile bool s_pose_pending = false;

static TrajectoryHeader s_last_traj_hdr;
static TrajectoryKnot s_last_traj_knots[UDP_MAX_TRAJ_KNOTS];
static volatile uint16_t s_last_traj_count = 0;
static volatile bool s_last_traj_valid = false;

static bool time_elapsed(uint32_t now, uint32_t start, uint32_t interval_ms)
{
    return (uint32_t)(now - start) >= interval_ms;
}

static void pack_message(uint16_t msg_type, uint32_t seq, const void *payload, uint32_t payload_len, uint8_t *out_buffer)
{
    MessageHeader *hdr = (MessageHeader *)out_buffer;
    hdr->magic = MAGIC;
    hdr->version = VERSION;
    hdr->msg_type = msg_type;
    hdr->seq = seq;
    hdr->t_ms = HAL_GetTick();
    hdr->payload_len = payload_len;
    hdr->crc32 = 0;

    if (payload_len > 0 && payload != NULL)
    {
        memcpy(out_buffer + HEADER_SIZE, payload, payload_len);
    }
}

static bool parse_message(const uint8_t *buffer, uint32_t len, MessageHeader *out_hdr, const uint8_t **out_payload)
{
    if (len < HEADER_SIZE)
    {
        return false;
    }

    const MessageHeader *hdr = (const MessageHeader *)buffer;
    if (hdr->magic != MAGIC || hdr->version != VERSION)
    {
        return false;
    }

    if ((HEADER_SIZE + hdr->payload_len) > len)
    {
        return false;
    }

    if (out_hdr)
    {
        *out_hdr = *hdr;
    }
    if (out_payload)
    {
        *out_payload = buffer + HEADER_SIZE;
    }

    return true;
}

static void handle_traj_message(const uint8_t *payload, uint32_t payload_len)
{
    if (payload_len < sizeof(TrajectoryHeader))
    {
        return;
    }

    const TrajectoryHeader *hdr = (const TrajectoryHeader *)payload;
    uint32_t expected_size = sizeof(TrajectoryHeader) + (uint32_t)hdr->n_knots * sizeof(TrajectoryKnot);
    if (payload_len < expected_size)
    {
        return;
    }

    uint16_t count = hdr->n_knots;
    if (count > UDP_MAX_TRAJ_KNOTS)
    {
        count = UDP_MAX_TRAJ_KNOTS;
    }

    taskENTER_CRITICAL();
    s_last_traj_hdr = *hdr;
    memcpy(s_last_traj_knots, payload + sizeof(TrajectoryHeader), count * sizeof(TrajectoryKnot));
    s_last_traj_count = count;
    s_last_traj_valid = true;
    taskEXIT_CRITICAL();
}

static void handle_cmd_ack(const MessageHeader *hdr, const uint8_t *payload, uint32_t payload_len)
{
    if (payload_len < sizeof(CommandPayload))
    {
        return;
    }

    const CommandPayload *cmd = (const CommandPayload *)payload;
    if (cmd->cmd_id != s_cmd_id)
    {
        return;
    }

    if (hdr->seq != s_cmd_seq)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_cmd_pending = false;
    s_cmd_retry_left = 0;
    taskEXIT_CRITICAL();
}

static void handle_received_message(const MessageHeader *hdr, const uint8_t *payload, uint32_t payload_len)
{
    if (hdr->msg_type == MSG_TYPE_TRAJ)
    {
        handle_traj_message(payload, payload_len);
    }
    else if (hdr->msg_type == MSG_TYPE_CMD)
    {
        handle_cmd_ack(hdr, payload, payload_len);
    }
}

static void udp_receive_nonblocking(void)
{
    if (s_udp_socket < 0)
    {
        return;
    }

    uint8_t temp_buf[UDP_RX_BUFFER_SIZE];

    for (;;)
    {
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int received = recvfrom(s_udp_socket, temp_buf, sizeof(temp_buf), MSG_DONTWAIT, (struct sockaddr *)&src, &slen);
        if (received <= 0)
        {
            if (received < 0)
            {
                int err = errno;
                if (err == EWOULDBLOCK || err == EAGAIN)
                {
                    return;
                }
            }
            return;
        }

        uint32_t offset = 0;
        while (offset + HEADER_SIZE <= (uint32_t)received)
        {
            MessageHeader hdr;
            const uint8_t *payload = NULL;
            if (!parse_message(temp_buf + offset, (uint32_t)received - offset, &hdr, &payload))
            {
                break;
            }

            uint32_t total = HEADER_SIZE + hdr.payload_len;
            handle_received_message(&hdr, payload, hdr.payload_len);
            offset += total;
        }
    }
}

static void udp_send_nonblocking(const uint8_t *data, uint32_t len)
{
    if (s_udp_socket < 0)
    {
        return;
    }

    static uint32_t last_err_log_ms = 0;

    int sent = sendto(s_udp_socket, data, len, MSG_DONTWAIT, (struct sockaddr *)&s_dst_addr, sizeof(s_dst_addr));
    if (sent < 0)
    {
        int err = errno;
        if (err == EWOULDBLOCK || err == EAGAIN)
        {
            return;
        }

        uint32_t now = HAL_GetTick();
        if (time_elapsed(now, last_err_log_ms, 2000u))
        {
            printf("udp: sendto failed err=%d len=%lu\r\n", err, (unsigned long)len);
            last_err_log_ms = now;
        }
    }
}

static void send_pose_payload(const PosePayload *pose)
{
    if (!pose)
    {
        return;
    }

    if (s_udp_socket < 0 || !netif_is_link_up(&gnetif))
    {
        printf("udp: not sending pose, socket=%d link=%d\r\n", s_udp_socket, netif_is_link_up(&gnetif));
        return;
    }

    uint8_t buffer[HEADER_SIZE + sizeof(PosePayload)];
    pack_message(MSG_TYPE_POSE, s_pose_seq++, pose, sizeof(*pose), buffer);

    udp_send_nonblocking(buffer, sizeof(buffer));
}

static void queue_pose(const PosePayload *pose)
{
    if (!pose)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_pending_pose = *pose;
    s_pose_pending = true;
    taskEXIT_CRITICAL();
}

void UDP_Client_SendZeroPose(void)
{
    PosePayload pose;
    pose.pose_t_ms = HAL_GetTick();
    pose.x = 0.0f;
    pose.y = 0.0f;
    pose.yaw = 0.0f;
    pose.vx = 0.0f;
    pose.vy = 0.0f;
    pose.wz = 0.0f;

    queue_pose(&pose);
}

static void send_cmd_to_pi5(uint16_t cmd_id, uint32_t seq)
{
    CommandPayload payload;
    payload.cmd_id = cmd_id;
    payload.arg_len = 0;

    uint8_t buffer[HEADER_SIZE + sizeof(CommandPayload)];
    pack_message(MSG_TYPE_CMD, seq, &payload, sizeof(payload), buffer);

    udp_send_nonblocking(buffer, sizeof(buffer));
}

static int udp_bind_socket(int sock)
{
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = 0;  // Let system assign port for sending
    local_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    {
        printf("udp: bind failed, errno=%d\r\n", errno);
        return -1;
    }

    printf("udp: socket bound\r\n");
    return 0;
}

static int udp_setup_nonblocking_socket(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        printf("udp: socket creation failed, errno=%d\r\n", errno);
        return -1;
    }

    int flags = lwip_fcntl(sock, F_GETFL, 0);
    (void)lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    int reuse = 1;
    (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000;
    (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    (void)setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    if (udp_bind_socket(sock) < 0)
    {
        lwip_close(sock);
        return -1;
    }

    return sock;
}

static void udp_prepare_dest_addr(void)
{
    memset(&s_dst_addr, 0, sizeof(s_dst_addr));
    s_dst_addr.sin_family = AF_INET;
    s_dst_addr.sin_port = htons(PI5_PORT);
    s_dst_addr.sin_addr.s_addr = inet_addr(PI5_IP_ADDR);
}

void UDP_Client_RequestCmd(CommandID cmd_id)
{
    taskENTER_CRITICAL();
    s_cmd_id = (uint16_t)cmd_id;
    s_cmd_seq++;
    s_cmd_pending = true;
    s_cmd_retry_left = UDP_CMD_RETRY_COUNT;
    s_cmd_last_tx_ms = 0;
    taskEXIT_CRITICAL();
}

bool UDP_Client_CopyLatestTraj(void *out_buf, uint32_t buf_len, uint16_t *out_knots, float *out_dt, uint16_t *out_flags)
{
    if (!out_buf || buf_len == 0)
    {
        return false;
    }

    if (!s_last_traj_valid)
    {
        return false;
    }

    taskENTER_CRITICAL();
    uint16_t count = s_last_traj_count;
    if (count > UDP_MAX_TRAJ_KNOTS)
    {
        count = UDP_MAX_TRAJ_KNOTS;
    }

    uint32_t bytes = (uint32_t)count * sizeof(TrajectoryKnot);
    if (bytes > buf_len)
    {
        bytes = buf_len - (buf_len % sizeof(TrajectoryKnot));
        count = (uint16_t)(bytes / sizeof(TrajectoryKnot));
    }

    memcpy(out_buf, s_last_traj_knots, bytes);
    if (out_knots)
    {
        *out_knots = count;
    }
    if (out_dt)
    {
        *out_dt = s_last_traj_hdr.dt;
    }
    if (out_flags)
    {
        *out_flags = s_last_traj_hdr.flags;
    }
    taskEXIT_CRITICAL();

    return true;
}

void UDP_Client_Task(void *argument)
{
    (void)argument;

    uint32_t next_socket_retry = 0;
    uint32_t retry_ms = UDP_SOCKET_RETRY_MIN_MS;
    bool last_link_up = false;

    udp_prepare_dest_addr();

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        if (s_udp_socket < 0)
        {
            if (time_elapsed(now, next_socket_retry, 0))
            {
                int sock = udp_setup_nonblocking_socket();
                if (sock >= 0)
                {
                    s_udp_socket = sock;
                    retry_ms = UDP_SOCKET_RETRY_MIN_MS;
                    printf("udp: socket created and bound, link %s, ip %u.%u.%u.%u\r\n",
                           netif_is_link_up(&gnetif) ? "up" : "down",
                           ip4_addr1(&gnetif.ip_addr), ip4_addr2(&gnetif.ip_addr), ip4_addr3(&gnetif.ip_addr), ip4_addr4(&gnetif.ip_addr));
                }
                else
                {
                    printf("udp: socket setup failed, errno=%d\r\n", errno);
                    if (retry_ms < UDP_SOCKET_RETRY_MAX_MS)
                    {
                        retry_ms *= 2u;
                        if (retry_ms > UDP_SOCKET_RETRY_MAX_MS)
                        {
                            retry_ms = UDP_SOCKET_RETRY_MAX_MS;
                        }
                    }
                    next_socket_retry = now + retry_ms;
                }
            }

            osDelay(UDP_TASK_DELAY_MS);
            continue;
        }

        /* Log link state only on change and send a single gratuitous ARP on link-up. */
        bool link_up = netif_is_link_up(&gnetif);
        if (link_up != last_link_up)
        {
            printf("netif link %s ip %u.%u.%u.%u gw %u.%u.%u.%u\r\n",
                   link_up ? "up" : "down",
                   ip4_addr1(&gnetif.ip_addr), ip4_addr2(&gnetif.ip_addr), ip4_addr3(&gnetif.ip_addr), ip4_addr4(&gnetif.ip_addr),
                   ip4_addr1(&gnetif.gw), ip4_addr2(&gnetif.gw), ip4_addr3(&gnetif.gw), ip4_addr4(&gnetif.gw));

            if (link_up)
            {
                err_t arp_err = etharp_gratuitous(&gnetif);
                printf("netif gratuitous ARP sent (err=%d)\r\n", (int)arp_err);
            }

            last_link_up = link_up;
        }

        if (s_cmd_pending)
        {
            if (s_cmd_retry_left > 0 && time_elapsed(now, s_cmd_last_tx_ms, UDP_CMD_RETRY_INTERVAL_MS))
            {
                send_cmd_to_pi5(s_cmd_id, s_cmd_seq);
                s_cmd_last_tx_ms = now;
                s_cmd_retry_left--;
                if (s_cmd_retry_left == 0)
                {
                    s_cmd_pending = false;
                }
            }
        }

        /* Only this task touches the UDP socket; drain any queued pose. */
        if (link_up && s_pose_pending)
        {
            PosePayload pose;
            taskENTER_CRITICAL();
            pose = s_pending_pose;
            s_pose_pending = false;
            taskEXIT_CRITICAL();

            send_pose_payload(&pose);
        }

        udp_receive_nonblocking();
        osDelay(UDP_TASK_DELAY_MS);
    }
}
