/*
 * How to use (typical):
 *  1) In main.c after MX_USART1_UART_Init():
 *       CMD_Init(&huart1);
 *       CMD_StartTask();   // creates the parser task
 *
 *  2) Ensure HAL_UART_RxCpltCallback() in your project calls into this module
 *     (this file implements the callback; if you already have one elsewhere,
 *      forward the byte to CMD_OnUartRxByteFromISR()).
 *
 *  3) Send from phone: "0\n" => runs command ID 0
 *
 * This file is self-contained, but you’ll usually create a cmd.h with the public APIs at the bottom.
 */

#include "main.h"
#include "stm32h7xx_hal.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include "eth_pose.h"
#include "cmd.h"

extern TIM_HandleTypeDef htim2;
/* ----------------------------- Config --------------------------------- */

#ifndef CMD_ENABLE_NAME_COMMANDS
#define CMD_ENABLE_NAME_COMMANDS      1   // 1 = allow "hello", "reset", "help" in addition to numeric-only
#endif

#ifndef CMD_MAX_LINE
#define CMD_MAX_LINE                 64u  // max chars per received line (excluding null terminator)
#endif

#ifndef CMD_RX_RING_SIZE
#define CMD_RX_RING_SIZE            256u  // must be power-of-two for fastest wrap (recommended)
#endif

#ifndef CMD_TASK_STACK_WORDS
#define CMD_TASK_STACK_WORDS        512u  // 2048 bytes - needed for CMD_Printf 128-byte buffer + call stack
#endif

#ifndef CMD_TASK_PRIO
#define CMD_TASK_PRIO               (tskIDLE_PRIORITY + 2)
#endif

#ifndef CMD_UART_TX_TIMEOUT_MS
#define CMD_UART_TX_TIMEOUT_MS       20u
#endif

#ifndef CMD_ECHO_RX
#define CMD_ECHO_RX                  0    // 1 = echo received lines back
#endif

#ifndef CMD_PRINT_UNKNOWN
#define CMD_PRINT_UNKNOWN            1    // 1 = print error on unknown command
#endif

#ifndef CMD_ECHO_RAW_TO_PC
#define CMD_ECHO_RAW_TO_PC  0   // 1 = echo every RX byte to PC via putchar/printf
#endif


/* ------------------------- Internal types ----------------------------- */

typedef void (*cmd_handler_t)(const char *args);

typedef struct
{
    uint16_t        id;
    const char     *name;   // optional if CMD_ENABLE_NAME_COMMANDS=0
    const char     *help;
    cmd_handler_t   fn;
} cmd_entry_t;

/* ---------------------- Module state / globals ------------------------ */

static UART_HandleTypeDef *s_huart = NULL;

static TaskHandle_t  s_cmdTaskHandle = NULL;
static SemaphoreHandle_t s_txMutex = NULL;

static uint8_t  s_rb[CMD_RX_RING_SIZE];
static volatile uint16_t s_rbHead = 0;
static volatile uint16_t s_rbTail = 0;

/* -------------------------- Forward decls ----------------------------- */

static void     CMD_Task(void *argument);
static void     CMD_ProcessLine(char *line);
static char    *CMD_Trim(char *s);
static bool     CMD_IsPureNumber(const char *s);
static const cmd_entry_t *CMD_FindById(uint32_t id);
static const cmd_entry_t *CMD_FindByName(const char *name);

static void     CMD_Printf(const char *fmt, ...);

/* ----------------------- Example command handlers ---------------------- */

static void cmd_hello(const char *args);
static void cmd_slow(const char *args);
static void cmd_med(const char *args);
static void cmd_stop(const char *args);
static void cmd_reset(const char *args);
static void cmd_help(const char *args);
static void cmd_dir(const char *args);
static void cmd_rotate(const char *args);
static void cmd_traj(const char *args);

/* -------------------------- Command table ----------------------------- */
/*
 * Edit this table to suit your project.
 * ID-based usage: send "0\n" to run cmd_hello, "1\n" to run cmd_reset, etc.
 */
static const cmd_entry_t s_cmdTable[] =
{
    { 0, "hello", "Print hello",              cmd_hello },
    { 1, "reset", "NVIC_SystemReset()",       cmd_reset },
    { 2, "help",  "List available commands",  cmd_help  },
	{ 3, "Stop",  "Stop Wheel",  cmd_stop  },
	{ 4, "Slow",  "Move Wheel Slowly",  cmd_slow  },
	{ 5, "Medium",  "Move Wheel Mediumly",  cmd_med  },
	{ 6, "dir",  "dir + #",  cmd_dir  },
	{ 7, "w",  "w + #",  cmd_rotate  },
	{ 8, "traj",  "traj 1=start 0=stop",  cmd_traj  },

};
static const size_t s_cmdTableCount = sizeof(s_cmdTable) / sizeof(s_cmdTable[0]);

/* ------------------------- Ring buffer helpers ------------------------ */

static inline uint16_t rb_next(uint16_t idx)
{
    return (uint16_t)((idx + 1u) & (CMD_RX_RING_SIZE - 1u));
}

static inline bool rb_is_full(void)
{
    return (rb_next(s_rbHead) == s_rbTail);
}

static inline bool rb_is_empty(void)
{
    return (s_rbHead == s_rbTail);
}

static bool rb_push(uint8_t b)
{
    uint16_t next = rb_next(s_rbHead);
    if (next == s_rbTail)
    {
        // full
        return false;
    }
    s_rb[s_rbHead] = b;
    s_rbHead = next;
    return true;
}

static bool rb_pop(uint8_t *out)
{
    if (rb_is_empty()) return false;
    *out = s_rb[s_rbTail];
    s_rbTail = rb_next(s_rbTail);
    return true;
}

/* ----------------------------- Public API ----------------------------- */
/*
 * Recommended cmd.h prototypes:
 *
 *   void CMD_Init(UART_HandleTypeDef *huart);
 *   bool CMD_StartTask(void);
 *   void CMD_OnUartRxByteFromISR(uint8_t b);
 *
 * If you already have HAL_UART_RxCpltCallback elsewhere, call CMD_OnUartRxByteFromISR()
 * from your callback and restart HAL_UART_Receive_IT() similarly.
 */

void CMD_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    if (s_txMutex == NULL)
    {
        s_txMutex = xSemaphoreCreateMutex();
    }
}

bool CMD_StartTask(void)
{
    if (s_cmdTaskHandle != NULL) return true;
    if (s_huart == NULL) return false;

    BaseType_t ok = xTaskCreate(
        CMD_Task,
        "CMD",
        CMD_TASK_STACK_WORDS,
        NULL,
        CMD_TASK_PRIO,
        &s_cmdTaskHandle
    );

    return (ok == pdPASS);
}

/*
 * Optional: if you want to feed bytes from another UART callback location.
 */
void CMD_OnUartRxByteFromISR(uint8_t b)
{
    BaseType_t hpw = pdFALSE;
    (void)rb_push(b);               // drop on overflow
    if (s_cmdTaskHandle)
    {
        vTaskNotifyGiveFromISR(s_cmdTaskHandle, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

/* ---------------------- HAL callback (optional) ----------------------- */
/*
 * If you want this module to own HAL_UART_RxCpltCallback(), set
 *   #define CMD_PROVIDE_HAL_UART_CALLBACK 1
 * in cmd.h (or compiler defines) AND do not define HAL_UART_RxCpltCallback elsewhere.
 *
 * Default is 0 because most projects have a single callback in main.c that
 * dispatches bytes to multiple modules (CMD + IMU, etc.).
 */
#ifndef CMD_PROVIDE_HAL_UART_CALLBACK
#define CMD_PROVIDE_HAL_UART_CALLBACK 0
#endif

#if CMD_PROVIDE_HAL_UART_CALLBACK
static uint8_t s_rxByte = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL) return;
    if (huart == s_huart)
    {
        CMD_OnUartRxByteFromISR(s_rxByte);
        (void)HAL_UART_Receive_IT(s_huart, &s_rxByte, 1);
    }
}
#endif
/* ------------------------------- Task --------------------------------- */

static void CMD_Task(void *argument)
{
    (void)argument;

    char line[CMD_MAX_LINE + 1u];
    uint32_t lineLen = 0;

    for (;;)
    {
        // Sleep until we get at least one byte
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Drain ring buffer
        uint8_t b;
        while (rb_pop(&b))
        {
        #if CMD_ECHO_RAW_TO_PC
            // Echo raw UART1 stream to the PC console (printf retarget).
            // This runs in the task context (safe), not in the ISR.
            (void)putchar((int)b);     // or: printf("%c", b);
        #endif

            if (b == '\r')
            {
                // keep ignoring CR for line parsing (but it already got echoed above)
                continue;
            }

            if (b == '\n')
            {
                line[lineLen] = '\0';

#if CMD_ECHO_RX
                CMD_Printf("> %s\r\n", line);
#endif

                CMD_ProcessLine(line);
                lineLen = 0;
                continue;
            }

            // normal char
            if (lineLen < CMD_MAX_LINE)
            {
                line[lineLen++] = (char)b;
            }
            else
            {
                // overflow -> reset line (or you could keep last CMD_MAX_LINE chars)
                lineLen = 0;
            }
        }
    }
}

/* --------------------------- Parsing logic ---------------------------- */

static void CMD_ProcessLine(char *line)
{
    char *s = CMD_Trim(line);
    if (s[0] == '\0') return;

    // Numeric-only => ID lookup
    if (CMD_IsPureNumber(s))
    {
        uint32_t id = (uint32_t)strtoul(s, NULL, 10);
        const cmd_entry_t *e = CMD_FindById(id);
        if (e && e->fn)
        {
            e->fn(NULL);
        }
        else
        {
#if CMD_PRINT_UNKNOWN
            CMD_Printf("ERR unknown id %lu\r\n", (unsigned long)id);
#endif
        }
        return;
    }

#if CMD_ENABLE_NAME_COMMANDS
    // Name-based: token = command, remainder = args
    char *cmd = s;
    while (*s && !isspace((unsigned char)*s)) s++;
    if (*s) { *s++ = '\0'; }
    char *args = CMD_Trim(s);

    const cmd_entry_t *e = CMD_FindByName(cmd);
    if (e && e->fn)
    {
        e->fn(args[0] ? args : NULL);
    }
    else
    {
#if CMD_PRINT_UNKNOWN
        CMD_Printf("ERR unknown cmd '%s'\r\n", cmd);
#endif
    }
#else
#if CMD_PRINT_UNKNOWN
    CMD_Send("ERR expected numeric id\r\n");
#endif
#endif
}

static char *CMD_Trim(char *s)
{
    while (isspace((unsigned char)*s)) s++;

    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) end--;
    *end = '\0';
    return s;
}

static bool CMD_IsPureNumber(const char *s)
{
    // allow leading/trailing spaces already trimmed; allow optional + sign
    if (*s == '+') s++;
    if (*s == '\0') return false;

    while (*s)
    {
        if (!isdigit((unsigned char)*s)) return false;
        s++;
    }
    return true;
}

static const cmd_entry_t *CMD_FindById(uint32_t id)
{
    for (size_t i = 0; i < s_cmdTableCount; i++)
    {
        if ((uint32_t)s_cmdTable[i].id == id) return &s_cmdTable[i];
    }
    return NULL;
}

static const cmd_entry_t *CMD_FindByName(const char *name)
{
    for (size_t i = 0; i < s_cmdTableCount; i++)
    {
        const char *n = s_cmdTable[i].name;
        if (n && (strcmp(n, name) == 0)) return &s_cmdTable[i];
    }
    return NULL;
}

/* -------------------------- TX helper funcs --------------------------- */

void CMD_Send(const char *s)
{
    if (!s_huart || !s) return;

    if (s_txMutex) (void)xSemaphoreTake(s_txMutex, portMAX_DELAY);
    (void)HAL_UART_Transmit(s_huart, (uint8_t*)s, (uint16_t)strlen(s), 100);
    if (s_txMutex) (void)xSemaphoreGive(s_txMutex);
}

static void CMD_Printf(const char *fmt, ...)
{
    if (!fmt) return;

    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    CMD_Send(buf);
}

double direction = 0;

volatile double speed[3] = {0};
volatile double vxd = 0;
volatile double vyd = 0;
volatile double yawrated = 0;
double vdes = 0;
static void cmd_rotate(const char *args)
{
    if (!args) return;

    char *end = NULL;
    float wd = strtof(args, &end);

    if (end == args) return;

    yawrated = wd;
    
    // Automatically stop trajectory mode when manual control is used
    if (ETH_POSE_GetTrajectoryMode()) {
        ETH_POSE_SetTrajectoryMode(false);
    }
}
static void cmd_dir(const char *args)
{
    if (!args) return;

    char *end = NULL;
    float deg = strtof(args, &end);

    if (end == args) return;

    direction = deg * pion180 + 0.261799;   // radians + 15deg offset
    vxd = vdes * cos(direction);
    vyd = vdes * sin(direction);
    
    // Automatically stop trajectory mode when manual control is used
    if (ETH_POSE_GetTrajectoryMode()) {
        ETH_POSE_SetTrajectoryMode(false);
    }
}

static void cmd_slow(const char *args)
{
    (void)args;
    CMD_Send("Slow\r\n");
    vdes = 0.3;
    vxd = vdes * cos(direction);
    vyd = vdes * sin(direction);
    
    // Automatically stop trajectory mode when manual control is used
    if (ETH_POSE_GetTrajectoryMode()) {
        ETH_POSE_SetTrajectoryMode(false);
    }
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SLOW);
}
static void cmd_med(const char *args)
{
    (void)args;
    CMD_Send("Medium\r\n");
    vdes = 0.6;
    vxd = vdes * cos(direction);
    vyd = vdes * sin(direction);
    
    // Automatically stop trajectory mode when manual control is used
    if (ETH_POSE_GetTrajectoryMode()) {
        ETH_POSE_SetTrajectoryMode(false);
    }
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MED);
}

void cmd_stop(const char *args)
{
    (void)args;
    CMD_Send("Stop\r\n");
    vxd = 0;
    vyd = 0;
    
    // Automatically stop trajectory mode when manual control is used
    if (ETH_POSE_GetTrajectoryMode()) {
        ETH_POSE_SetTrajectoryMode(false);
    }
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, NEUTRAL);
}


static void cmd_hello(const char *args)
{
    (void)args;
    BSP_LED_Toggle(LED_RED);
    CMD_Send("hello\r\n");
}

static void cmd_reset(const char *args)
{
    (void)args;

    // Optional: tell the phone first
    CMD_Send("resetting...\r\n");

    // Give UART a moment to flush (FreeRTOS-safe delay)
    vTaskDelay(pdMS_TO_TICKS(50));

    __DSB();
    __ISB();
    NVIC_SystemReset();
}

static void cmd_help(const char *args)
{
    (void)args;
    CMD_Send("Commands:\r\n");
    for (size_t i = 0; i < s_cmdTableCount; i++)
    {
        const cmd_entry_t *e = &s_cmdTable[i];
#if CMD_ENABLE_NAME_COMMANDS
        CMD_Printf("  %u  %-8s  %s\r\n", (unsigned)e->id, (e->name ? e->name : ""), (e->help ? e->help : ""));
#else
        CMD_Printf("  %u  %s\r\n", (unsigned)e->id, (e->help ? e->help : ""));
#endif
    }
}

static void cmd_traj(const char *args)
{
    // Parse argument: 1=start trajectory mode, 0=stop trajectory mode
    int mode = 0;
    if (args && *args) {
        mode = atoi(args);
    }
    
    if (mode == 1) {
        ETH_POSE_SetTrajectoryMode(true);
        CMD_Send("Trajectory mode STARTED\r\n");
    } else {
        ETH_POSE_SetTrajectoryMode(false);
        CMD_Send("Trajectory mode STOPPED\r\n");
    }
}
