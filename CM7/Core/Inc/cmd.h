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

// MD20A motor driver with Saturn 5303 motors
// PWM: 0-100% duty cycle (TIM2 ARR=7371)
#define PWM_PERIOD 7371
#define PWM_MIN 0
#define PWM_MAX PWM_PERIOD
#define PWM_RANGE PWM_PERIOD  // Full range for duty cycle

// Saturn 5303 planetary gearmotor (output shaft) configuration
// Datasheet no-load speeds: 70 RPM @12V, 120 RPM @20V, 130 RPM @24V
#define SATURN5303_NO_LOAD_RPM_12V      70.0
#define SATURN5303_NO_LOAD_RPM_20V      120.0
#define SATURN5303_NO_LOAD_RPM_24V      130.0

// Legacy nominal operating point kept for compatibility with existing tuning
#define SATURN5303_NO_LOAD_RPM          SATURN5303_NO_LOAD_RPM_20V

// Encoder resolution (datasheet): 3895.9 PPR at output shaft
// If your timer count is 4x this value, set QUAD_FACTOR to 4.0.
#define SATURN5303_ENCODER_PPR_OUTPUT   3895.9
#define SATURN5303_ENCODER_QUAD_FACTOR  1.0
#define SATURN5303_OUTPUT_CPR           (SATURN5303_ENCODER_PPR_OUTPUT * SATURN5303_ENCODER_QUAD_FACTOR)

// Gear ratio from datasheet formula: (1 + 46/11)^3
#define SATURN5303_GEAR_RATIO           ((1.0 + (46.0/11.0)) * (1.0 + (46.0/11.0)) * (1.0 + (46.0/11.0)))

// Wheel sign convention (model space):
// +1.0 means electrical forward matches positive model wheel rotation.
// -1.0 means wheel is mechanically/electrically inverted relative to model.
#define WHEEL1_SIGN                     (-1.0)
#define WHEEL2_SIGN                     (-1.0)
#define WHEEL3_SIGN                     (-1.0)

// Wheel-test command polarity:
// +1.0 => positive `wtest` RPM uses normal internal sign
// -1.0 => positive `wtest` RPM flips internal sign (useful when bench test
//         positive command currently produces negative measured RPM)
#define WHEEL_TEST_CMD_SIGN             (+1.0)

extern volatile double speed[3];
extern volatile double vxd;
extern volatile double vyd;
extern volatile double yawrated;
extern volatile uint8_t wheel_test_mode;
extern volatile int8_t wheel_test_index;
extern volatile double wheel_test_target_rpm;

// 0 = remote/manual velocity control, 1 = follow incoming Pi5 trajectory
extern volatile uint8_t traj_mode;

void PWM(double rpm[3], double dt);
void PWM_SetMaxRpmLimit(double max_rpm);
double PWM_GetMaxRpmLimit(void);
/* Call from HAL_UART_RxCpltCallback (ISR context) */
void CMD_OnUartRxByteFromISR(uint8_t b);
