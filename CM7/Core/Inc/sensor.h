#pragma once

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float ax_mps2, ay_mps2, az_mps2;   // Linear accel (m/s^2)
  float yaw_rate_rps;               // Gyro Z (rad/s)
  float heading_deg;                // 0..360 (deg), declination-adjusted
  uint32_t last_linacc_ms;
  uint32_t last_gyro_ms;
  uint32_t last_rv_ms;
} IMU_Data_t;

/**
 * Call this from HAL_GPIO_EXTI_Callback when the IMU INT pin triggers.
 */
void IMU_OnExti(uint16_t gpio_pin);

/**
 * Init + enable reports.
 * - hi2c: your I2C2 handle
 * - addr7: usually 0x4B on SparkFun (sometimes 0x4A)
 * - report_hz: e.g. 100
 */
HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint32_t report_hz);

/**
 * Drain pending packets when INT is asserted.
 * Call often from main loop / task. It only does work when INT fired.
 */
void IMU_Service(void);

/**
 * Get the latest cached values (non-blocking).
 */
IMU_Data_t IMU_Get(void);

/**
 * Blocking “on request” reads: waits until a *new* sample arrives (timeout_ms).
 * These still rely on INT and call IMU_Service internally.
 */
HAL_StatusTypeDef IMU_ReadLinearAccel(float *ax, float *ay, float *az, uint32_t timeout_ms);
HAL_StatusTypeDef IMU_ReadYawRate(float *yaw_rate_rps, uint32_t timeout_ms);
HAL_StatusTypeDef IMU_ReadTrueHeading(float *heading_deg, uint32_t timeout_ms);

/** Optional: magnetic declination adjust (degrees, +east). */
void IMU_SetDeclinationDeg(float decl_deg);
