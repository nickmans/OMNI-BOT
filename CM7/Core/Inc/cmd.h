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

// MD20A motor driver with Pololu 37D 24V 50:1 motors
// PWM: 0-100% duty cycle (TIM2 ARR=7371)
#define PWM_PERIOD 7371
#define PWM_MIN 0
#define PWM_MAX PWM_PERIOD
#define PWM_RANGE PWM_PERIOD  // Full range for duty cycle

// Pololu 37D 24V 50:1 output shaft configuration.
// Bench data: 0.95 PWM gave ~157 RPM with old CPR=3895.9. Corrected to CPR=3200,
// this is ~191.2 RPM at 95% duty, so use ~201 RPM nominal at 24V.
#define POLULU37D50_NO_LOAD_RPM_24V     201.0
#define POLULU37D50_NO_LOAD_RPM          POLULU37D50_NO_LOAD_RPM_24V

// Output shaft encoder CPR for these motors.
#define POLULU37D50_OUTPUT_CPR           3200.0
#define POLULU37D50_GEAR_RATIO           50.0

// Backward-compatible aliases used in existing source files.
#define SATURN5303_NO_LOAD_RPM           POLULU37D50_NO_LOAD_RPM
#define SATURN5303_OUTPUT_CPR            POLULU37D50_OUTPUT_CPR

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

// 0 = remote/manual velocity control, 1 = follow incoming Pi5 trajectory
extern volatile uint8_t traj_mode;

void PWM(double rpm[3], double dt);
void PWM_SetMaxRpmLimit(double max_rpm);
double PWM_GetMaxRpmLimit(void);
/* Call from HAL_UART_RxCpltCallback (ISR context) */
void CMD_OnUartRxByteFromISR(uint8_t b);
