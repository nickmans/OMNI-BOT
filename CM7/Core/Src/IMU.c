/*
 * IMU.c
 *
 *  Created on: 13 Jan 2026
 *      Author: Nick
 */

#include "IMU.h"
#include "stm32h7xx_hal.h"
#include <math.h>


static volatile bno_rvc_sample_t g_latest;
static volatile uint32_t g_seq = 0;            // sequence for atomic-ish reads

// ---- parser state ----
typedef enum { WAIT_AA1, WAIT_AA2, COLLECT } st_t;
static st_t st = WAIT_AA1;
static uint8_t pkt[19];
static uint8_t k = 0;

static inline int16_t le_i16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline float wrap180f(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void BNO_RVC_Init(void)
{
  // Reset published sample + parser state.
  // Note: UART RX is armed in main.c (HAL_UART_Receive_IT).
  g_latest.valid = 0;
  g_seq = 0;
  st = WAIT_AA1;
  k = 0;
}



// Call this from HAL_UART_RxCpltCallback when USART2 receives 1 byte
void BNO_RVC_OnByte(uint8_t b)
{
  switch (st) {
    case WAIT_AA1:
      if (b == 0xAA) { pkt[0] = b; st = WAIT_AA2; }
      break;

    case WAIT_AA2:
      if (b == 0xAA) { pkt[1] = b; k = 2; st = COLLECT; }
      else st = WAIT_AA1;
      break;

    case COLLECT:
      pkt[k++] = b;
      if (k >= sizeof(pkt)) {
        // verify checksum: sum bytes [2..17] == pkt[18]
        uint8_t sum = 0;
        for (int i = 2; i <= 17; i++) sum = (uint8_t)(sum + pkt[i]);

        if (sum == pkt[18]) {
          // decode
          bno_rvc_sample_t s = {0};
          s.index     = pkt[2];
          s.yaw_cdeg  = le_i16(&pkt[3]);
          s.pitch_cdeg= le_i16(&pkt[5]);
          s.roll_cdeg = le_i16(&pkt[7]);
          s.ax_mg     = le_i16(&pkt[9]);
          s.ay_mg     = le_i16(&pkt[11]);
          s.az_mg     = le_i16(&pkt[13]);

          s.yaw_deg = 0.01f * (float)s.yaw_cdeg;
          s.pitch_deg = 0.01f * (float)s.pitch_cdeg;
          s.roll_deg = 0.01f * (float)s.roll_cdeg;

          const float mg_to_mps2 = 9.80665e-3f;
          s.ax_mps2 = mg_to_mps2 * (float)s.ax_mg;
          s.ay_mps2 = mg_to_mps2 * (float)s.ay_mg;
          s.az_mps2 = mg_to_mps2 * (float)s.az_mg;

          // ---- yaw-rate from successive yaw samples ----
          static uint8_t  have_prev = 0;
          static uint8_t  prev_idx = 0;
          static float    prev_yaw = 0;

          if (have_prev) {
            uint8_t di = (uint8_t)(s.index - prev_idx);  // wraps naturally
            if (di == 0) di = 1;                         // avoid /0 if weird
            float dt = 0.01f * (float)di;                // 100 Hz base period
            float dyaw_deg = wrap180f(s.yaw_deg - prev_yaw);
            float yawrate_deg_s = dyaw_deg / dt;
            s.yawrate_rad_s = yawrate_deg_s * (float)(M_PI / 180.0);
          } else {
            s.yawrate_rad_s = 0.0f;
            have_prev = 1;
          }

          prev_idx = s.index;
          prev_yaw = s.yaw_deg;

          // publish latest (sequence-protected)
          g_seq++;
          g_latest = s;
          g_latest.valid = 1;
          g_seq++;
        }

        // resync for next packet
        st = WAIT_AA1;
      }
      break;
  }
}

bool BNO_RVC_GetLatest(bno_rvc_sample_t *out)
{
  if (!out) return false;

  // simple seq-based consistency check (avoid tearing)
  uint32_t s1, s2;
  do {
    s1 = g_seq;
    *out = g_latest;
    s2 = g_seq;
  } while ((s1 != s2) || (s1 & 1u));

  return (out->valid != 0);
}

// Calculate linear acceleration (gravity compensated) and update main variables
void BNO_RVC_UpdateMain(double *yaw_out, double *yawrate_out, 
                        double *ax_out, double *ay_out, double *az_out)
{
  bno_rvc_sample_t imu;
  if (!BNO_RVC_GetLatest(&imu)) return;
  
  // Convert angles to radians
  const double deg2rad = M_PI / 180.0;
  const double pitch_rad = (double)imu.pitch_deg * deg2rad;
  const double roll_rad = (double)imu.roll_deg * deg2rad;
  
  // Gravity constant (m/s^2)
  const double g = 9.80665;
  
  // Swap pitch/roll in calculation - sensor reports them opposite to body convention
  const double cp = cos(roll_rad);   // Use roll for pitch calc
  const double sp = sin(roll_rad);
  const double cr = cos(pitch_rad);  // Use pitch for roll calc
  const double sr = sin(pitch_rad);
  
  // Accelerometer measures -g rotated into body frame
  const double gx = -g * sp;
  const double gy = g * sr * cp;
  const double gz = g * cr * cp;
  
  // Linear acceleration = measured - gravity component
  const double ax_linear = (double)imu.ax_mps2 - gx;
  const double ay_linear = (double)imu.ay_mps2 - gy;
  const double az_linear = (double)imu.az_mps2 - gz;
  
  // Write to output variables
  if (yaw_out) *yaw_out = (double)imu.yaw_deg;
  if (yawrate_out) *yawrate_out = (double)imu.yawrate_rad_s;
  if (ax_out) *ax_out = ax_linear;
  if (ay_out) *ay_out = ay_linear;
  if (az_out) *az_out = az_linear;
}

// Debug function to get raw orientation and acceleration
void BNO_RVC_GetDebug(double *pitch_out, double *roll_out,
                      double *ax_raw, double *ay_raw, double *az_raw)
{
  bno_rvc_sample_t imu;
  if (!BNO_RVC_GetLatest(&imu)) return;
  
  if (pitch_out) *pitch_out = (double)imu.pitch_deg;
  if (roll_out) *roll_out = (double)imu.roll_deg;
  if (ax_raw) *ax_raw = (double)imu.ax_mps2;
  if (ay_raw) *ay_raw = (double)imu.ay_mps2;
  if (az_raw) *az_raw = (double)imu.az_mps2;
}
