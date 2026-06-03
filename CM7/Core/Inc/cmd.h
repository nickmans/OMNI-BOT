/*
 * cmd.h
 *
 *  Created on: 6 Jan 2026
 *      Author: Nick
 */
#include <stdbool.h>
#include <stdint.h>
void CMD_Init(UART_HandleTypeDef *huart);
bool CMD_StartTask(void);
void CMD_Send(const char *s);
void CMD_SendRaw(const uint8_t *data, uint16_t len);
void CMD_TerminalPassthroughRemoteClosed(void);

// MD20A motor driver output range
// PWM: 0-100% duty cycle (TIM2 ARR=7371)
#define PWM_PERIOD 7371
#define PWM_MIN 0
#define PWM_MAX PWM_PERIOD
#define PWM_RANGE PWM_PERIOD  // Full range for duty cycle

// Saturn 5303 planetary gearmotor constants.
// Datasheet encoder resolution at output shaft is 3895.9 counts/rev.
// If timer decoding reports 4x this value, set QUAD_FACTOR to 4.0.
#define SATURN5303_ENCODER_PPR_OUTPUT       3895.9
#define SATURN5303_ENCODER_QUAD_FACTOR      1.0
#define SATURN5303_GEAR_RATIO               139.14
#define SATURN5303_ENCODER_CPR_MOTOR_SHAFT  (SATURN5303_ENCODER_PPR_OUTPUT / SATURN5303_GEAR_RATIO)
#define SATURN5303_OUTPUT_CPR               (SATURN5303_ENCODER_PPR_OUTPUT * SATURN5303_ENCODER_QUAD_FACTOR)
#define SATURN5303_NO_LOAD_RPM              146.0

// Legacy aliases kept for backward compatibility; intentionally mapped to Saturn.
#define POLULU37D50_NO_LOAD_RPM_24V         SATURN5303_NO_LOAD_RPM
#define POLULU37D50_NO_LOAD_RPM             POLULU37D50_NO_LOAD_RPM_24V
#define POLULU37D50_OUTPUT_CPR              SATURN5303_OUTPUT_CPR
#define POLULU37D50_GEAR_RATIO              SATURN5303_GEAR_RATIO

// Wheel sign convention (model space):
// +1.0 means electrical forward matches positive model wheel rotation.
// -1.0 means wheel is mechanically/electrically inverted relative to model.
#define WHEEL1_SIGN                     (+1.0)
#define WHEEL2_SIGN                     (+1.0)
#define WHEEL3_SIGN                     (+1.0)

// Wheel-test command polarity:
// +1.0 => positive `wtest` RPM uses normal internal sign
// -1.0 => positive `wtest` RPM flips internal sign (useful when bench test
//         positive command currently produces negative measured RPM)
#define WHEEL_TEST_CMD_SIGN             (+1.0)

// Encoder polarity from raw timer counts -> model wheel frame.
// Set to -1.0 when a wheel's physical clockwise spin reports negative counts.
#define ENC_WHEEL1_SIGN                 (-1.0)
#define ENC_WHEEL2_SIGN                 (-1.0)
#define ENC_WHEEL3_SIGN                 (-1.0)

extern volatile double speed[3];
extern volatile double vxd;
extern volatile double vyd;
extern volatile double yawrated;
extern volatile uint8_t wheel_test_mode;
extern volatile int8_t wheel_test_index;
extern volatile double wheel_test_target_rpm;
extern volatile uint8_t pwm_test_mode;
extern volatile double pwm_test_ratio[3];
extern volatile uint8_t yaw_kick_enabled;

// 0 = remote/manual velocity control, 1 = follow incoming Pi5 trajectory
extern volatile uint8_t traj_mode;

void PWM(double rpm[3], double dt);
void PWM_SetMaxRpmLimit(double max_rpm);
double PWM_GetMaxRpmLimit(void);
/* Call from HAL_UART_RxCpltCallback (ISR context) */
void CMD_OnUartRxByteFromISR(uint8_t b);
