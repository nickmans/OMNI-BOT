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

#define NEUTRAL 2633
#define SLOW    (NEUTRAL + 200)
#define MED		(NEUTRAL + 450)
#define MED1		(NEUTRAL + 550)
#define MED2		(NEUTRAL + 650)
#define MED3		(NEUTRAL + 750)
#define VICTOR_RANGE 878
#define MAXf    (NEUTRAL + VICTOR_RANGE)				// max is +878
#define MAXb 	(NEUTRAL - VICTOR_RANGE)

extern volatile double speed[3];
extern volatile double vxd;
extern volatile double vyd;
extern volatile double yawrated;

void PWM(double rpm[3], double dt);
/* Call from HAL_UART_RxCpltCallback (ISR context) */
void CMD_OnUartRxByteFromISR(uint8_t b);
