/*
 * cmd.h
 *
 *  Created on: 6 Jan 2026
 *      Author: Nick
 */
#include <stdbool.h>
void CMD_Init(UART_HandleTypeDef *huart);
bool CMD_StartTask(void);
void CMD_Send(const char *s);

// MD20A motor driver with Saturn 5303 motors
// PWM: 0-100% duty cycle (TIM2 ARR=7371)
#define PWM_PERIOD 7371
#define PWM_MIN 0
#define PWM_MAX PWM_PERIOD
#define PWM_RANGE PWM_PERIOD  // Full range for duty cycle

extern volatile double speed[3];
extern volatile double vxd;
extern volatile double vyd;
extern volatile double yawrated;

void PWM(double rpm[3], double dt);
/* Call from HAL_UART_RxCpltCallback (ISR context) */
void CMD_OnUartRxByteFromISR(uint8_t b);
