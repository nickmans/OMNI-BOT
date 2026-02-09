#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint8_t  valid;
  uint8_t  index;
  int16_t  yaw_cdeg, pitch_cdeg, roll_cdeg;
  int16_t  ax_mg, ay_mg, az_mg;

  float    yaw_deg, pitch_deg, roll_deg;
  float    yawrate_rad_s;
  float    ax_mps2, ay_mps2, az_mps2;
} bno_rvc_sample_t;

void BNO_RVC_Init(void);
void BNO_RVC_OnByte(uint8_t b);
bool BNO_RVC_GetLatest(bno_rvc_sample_t *out);
void BNO_RVC_UpdateMain(double *yaw_out, double *yawrate_out, 
                        double *ax_out, double *ay_out, double *az_out);
void BNO_RVC_GetDebug(double *pitch_out, double *roll_out,
                      double *ax_raw, double *ay_raw, double *az_raw);
