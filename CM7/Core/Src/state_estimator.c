/* state_estimator.c
 *
 * Dead-reckoning state estimator for a 3-wheel 120° omni ("kiwi") base.
 *
 * HARD REQUIREMENT (per user):
 *   - Yaw ALWAYS comes from the IMU (BNO086) and is treated as ground truth.
 *   - We do NOT integrate yaw from wheel speeds.
 *
 * Inputs:
 *   - Encoder-derived wheel angular speeds (rad/s): w1,w2,w3
 *   - IMU yaw (rad)
 *   - dt (s)
 *
 * Output:
 *   - Pose [x, y, yaw] in meters/radians
 *
 * Assumptions / Dependencies:
 *   - main.h provides:
 *       extern double rw;   // wheel radius (m)
 *       extern double L;    // distance from robot center to wheel contact point (m)
 *
 * Optional:
 *   - If your IMU yaw has an arbitrary offset at each boot, you can "zero" it at startup:
 *       StateEstimator_ZeroImuYaw(current_imu_yaw_rad);
 *     After that, estimator yaw = imu_yaw - zero (wrapped to [-pi, pi]).
 *
 * Sign conventions:
 *   - If a wheel direction is flipped relative to the kinematic model, define per-wheel signs
 *     in main.h (or compiler flags) and set to -1.0 as needed:
 *       #define STATE_EST_W1_SIGN (-1.0)
 *       #define STATE_EST_W2_SIGN ( 1.0)
 *       #define STATE_EST_W3_SIGN ( 1.0)
 */

#include "main.h"
#include <math.h>
#include <stdint.h>

/* Optional per-wheel sign corrections (define in main.h if needed). */
#ifndef STATE_EST_W1_SIGN
#define STATE_EST_W1_SIGN (1.0)
#endif
#ifndef STATE_EST_W2_SIGN
#define STATE_EST_W2_SIGN (1.0)
#endif
#ifndef STATE_EST_W3_SIGN
#define STATE_EST_W3_SIGN (1.0)
#endif

/* Internal state (double everywhere) */
static double se_x_m   = 0.0;
static double se_y_m   = 0.0;
static double se_yaw_r = 0.0;

static double se_imu_yaw_zero_r = 0.0;  /* subtract from IMU yaw (rad) */
static uint8_t se_inited = 0u;

/* Wrap angle to [-pi, pi]. */
static inline double wrap_pi(double a)
{
  const double two_pi = 2.0 * M_PI;
  a = fmod(a + M_PI, two_pi);
  if (a < 0.0) { a += two_pi; }
  return a - M_PI;
}

/* Public API --------------------------------------------------------------- */

void StateEstimator_Reset(double x0_m, double y0_m, double yaw0_rad)
{
  se_x_m   = x0_m;
  se_y_m   = y0_m;
  se_yaw_r = wrap_pi(yaw0_rad);
  se_inited = 1u;
}

/* Use this if you want the IMU yaw origin to be 0 at startup.
 * Call once after IMU yaw is valid/stable.
 */
void StateEstimator_ZeroImuYaw(double imu_yaw_rad)
{
  se_imu_yaw_zero_r = imu_yaw_rad;
}

/* Get current pose without updating. */
void StateEstimator_GetPose(double pose_out[3])
{
  if (!pose_out) { return; }
  pose_out[0] = se_x_m;
  pose_out[1] = se_y_m;
  pose_out[2] = se_yaw_r;
}

/* Update estimator (yaw from IMU only).
 *
 * Inputs:
 *   w_rad_s[3]   : wheel speeds [w1 w2 w3] in rad/s (encoder-derived).
 *   dt_s         : timestep in seconds.
 *   imu_yaw_rad  : IMU yaw in radians (ground truth).
 *
 * Output:
 *   pose_out[3]  : [x y yaw] after update (meters, meters, radians).
 */
void StateEstimator_Update(const double w_rad_s[3],
                           double dt_s,
                           double imu_yaw_rad,
                           double pose_out[3])
{
  if (!se_inited)
  {
    /* Default init if caller forgot. */
    StateEstimator_Reset(0.0, 0.0, 0.0);
  }

  if (!w_rad_s || (dt_s <= 0.0) || !isfinite(dt_s))
  {
    if (pose_out) { StateEstimator_GetPose(pose_out); }
    return;
  }

  /* Robot geometry from main.h (extern). */
  const double r = rw;
  const double Leff = L;

  /* Guard against invalid geometry. */
  if (!isfinite(r) || !isfinite(Leff) || (Leff == 0.0))
  {
    if (pose_out) { StateEstimator_GetPose(pose_out); }
    return;
  }

  /* Wheel speeds (rad/s) with optional sign correction. */
  const double w1 = STATE_EST_W1_SIGN * w_rad_s[0];
  const double w2 = STATE_EST_W2_SIGN * w_rad_s[1];
  const double w3 = STATE_EST_W3_SIGN * w_rad_s[2];

  /* Body-frame velocities (matches your MATLAB kinematics). */
  const double k_s3 = 0.57735026918962576451; /* sqrt(3)/3 */
  const double vx_body = r * (k_s3 * (w2 - w3));
  const double vy_body = r * ((2.0/3.0) * w1 - (1.0/3.0) * (w2 + w3));

  /* Yaw is ground truth from IMU (optionally zeroed). */
  const double yaw = wrap_pi(imu_yaw_rad - se_imu_yaw_zero_r);
  se_yaw_r = yaw;

  /* Rotate body -> world using IMU yaw. */
  const double c = cos(yaw);
  const double s = sin(yaw);
  const double vx_world =  vx_body * c - vy_body * s;
  const double vy_world =  vx_body * s + vy_body * c;

  /* Integrate position (Euler). */
  se_x_m += dt_s * vx_world;
  se_y_m += dt_s * vy_world;

  if (pose_out)
  {
    pose_out[0] = se_x_m;
    pose_out[1] = se_y_m;
    pose_out[2] = se_yaw_r;
  }
}
