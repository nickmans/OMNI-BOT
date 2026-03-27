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
 * Sign conventions:
 *   - If a wheel direction is flipped relative to the kinematic model, define per-wheel signs
 *     in main.h (or compiler flags) and set to -1.0 as needed:
 *       #define STATE_EST_W1_SIGN (-1.0)
 *       #define STATE_EST_W2_SIGN ( 1.0)
 *       #define STATE_EST_W3_SIGN ( 1.0)
 */

#include "main.h"
#include "cmd.h"
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

/* Estimator tuning (override in main.h if desired). */
#ifndef STATE_EST_ACC_LPF_TAU_S
#define STATE_EST_ACC_LPF_TAU_S (0.15)
#endif
#ifndef STATE_EST_VEL_LEAK_TAU_S
#define STATE_EST_VEL_LEAK_TAU_S (1.80)
#endif
#ifndef STATE_EST_SLIP_LOW_MPS2
#define STATE_EST_SLIP_LOW_MPS2 (0.55)
#endif
#ifndef STATE_EST_SLIP_HIGH_MPS2
#define STATE_EST_SLIP_HIGH_MPS2 (3.80)
#endif
#ifndef STATE_EST_LOW_SPEED_MPS
#define STATE_EST_LOW_SPEED_MPS (0.20)
#endif
#ifndef STATE_EST_LOW_SPEED_BOOST_MAX
#define STATE_EST_LOW_SPEED_BOOST_MAX (0.12)
#endif
#ifndef STATE_EST_ACCEL_CLIP_MPS2
#define STATE_EST_ACCEL_CLIP_MPS2 (3.5)
#endif
#ifndef STATE_EST_ACCEL_DEADBAND_MPS2
#define STATE_EST_ACCEL_DEADBAND_MPS2 (0.08)
#endif
#ifndef STATE_EST_STATIONARY_WHEEL_RADPS
#define STATE_EST_STATIONARY_WHEEL_RADPS (0.30)
#endif
#ifndef STATE_EST_WHEEL_DEADBAND_RADPS
#define STATE_EST_WHEEL_DEADBAND_RADPS (0.35)
#endif
#ifndef STATE_EST_CMD_DEADBAND_LOW_RADPS
#define STATE_EST_CMD_DEADBAND_LOW_RADPS (0.20)
#endif
#ifndef STATE_EST_CMD_DEADBAND_HIGH_RADPS
#define STATE_EST_CMD_DEADBAND_HIGH_RADPS (1.20)
#endif
#ifndef STATE_EST_STATIONARY_ENC_SPEED_MPS
#define STATE_EST_STATIONARY_ENC_SPEED_MPS (0.05)
#endif
#ifndef STATE_EST_STATIONARY_ACC_MPS2
#define STATE_EST_STATIONARY_ACC_MPS2 (0.20)
#endif
#ifndef STATE_EST_BIAS_ADAPT
#define STATE_EST_BIAS_ADAPT (0.01)
#endif
#ifndef STATE_EST_BIAS_STATIONARY_HOLD_S
#define STATE_EST_BIAS_STATIONARY_HOLD_S (0.30)
#endif
#ifndef STATE_EST_HARD_ZERO_VEL_MPS
#define STATE_EST_HARD_ZERO_VEL_MPS (0.02)
#endif
#ifndef STATE_EST_MIN_INTEGRATE_SPEED_MPS
#define STATE_EST_MIN_INTEGRATE_SPEED_MPS (0.01)
#endif
#ifndef STATE_EST_ZUPT_BLEND
#define STATE_EST_ZUPT_BLEND (0.18)
#endif
#ifndef STATE_EST_ENC_ACCEL_LPF_TAU_S
#define STATE_EST_ENC_ACCEL_LPF_TAU_S (0.16)
#endif
#ifndef STATE_EST_SLIP_SMOOTH_TAU_S
#define STATE_EST_SLIP_SMOOTH_TAU_S (0.30)
#endif
#ifndef STATE_EST_HOLO_ACCEL_CHECK_MIN_SPEED_MPS
#define STATE_EST_HOLO_ACCEL_CHECK_MIN_SPEED_MPS (0.15)
#endif
#ifndef STATE_EST_HOLO_ACCEL_MISMATCH_LOW_MPS2
#define STATE_EST_HOLO_ACCEL_MISMATCH_LOW_MPS2 (0.50)
#endif
#ifndef STATE_EST_HOLO_ACCEL_MISMATCH_HIGH_MPS2
#define STATE_EST_HOLO_ACCEL_MISMATCH_HIGH_MPS2 (2.50)
#endif
#ifndef STATE_EST_HOLO_SLIP_BOOST_MAX
#define STATE_EST_HOLO_SLIP_BOOST_MAX (0.10)
#endif
#ifndef STATE_EST_ENCODER_WEIGHT_FLOOR
#define STATE_EST_ENCODER_WEIGHT_FLOOR (0.55)
#endif

/* Optional per-wheel sign corrections (define in main.h if needed). */
#ifndef STATE_EST_W1_SIGN
#ifdef WHEEL1_SIGN
#define STATE_EST_W1_SIGN (WHEEL1_SIGN)
#else
#define STATE_EST_W1_SIGN (1.0)
#endif
#endif
#ifndef STATE_EST_W2_SIGN
#ifdef WHEEL2_SIGN
#define STATE_EST_W2_SIGN (WHEEL2_SIGN)
#else
#define STATE_EST_W2_SIGN (1.0)
#endif
#endif
#ifndef STATE_EST_W3_SIGN
#ifdef WHEEL3_SIGN
#define STATE_EST_W3_SIGN (WHEEL3_SIGN)
#else
#define STATE_EST_W3_SIGN (1.0)
#endif
#endif

/* Internal state (double everywhere) */
static double se_x_m   = 0.0;
static double se_y_m   = 0.0;
static double se_yaw_r = 0.0;
static double se_vx_body_mps = 0.0;
static double se_vy_body_mps = 0.0;
static double se_wz_body_rps = 0.0;
static double se_vx_world_mps = 0.0;
static double se_vy_world_mps = 0.0;
static double se_vx_imu_body_mps = 0.0;
static double se_vy_imu_body_mps = 0.0;
static double se_ax_filt_body_mps2 = 0.0;
static double se_ay_filt_body_mps2 = 0.0;
static double se_ax_bias_body_mps2 = 0.0;
static double se_ay_bias_body_mps2 = 0.0;
static double se_vx_enc_prev_mps = 0.0;
static double se_vy_enc_prev_mps = 0.0;
static double se_ax_enc_filt_mps2 = 0.0;
static double se_ay_enc_filt_mps2 = 0.0;
static double se_slip_ratio_filt = 0.0;
static double se_slip_innovation_mps2 = 0.0;
static double se_w_enc_last = 1.0;
static double se_w_imu_last = 0.0;
static double se_vx_enc_body_last = 0.0;
static double se_vy_enc_body_last = 0.0;
static double se_stationary_time_s = 0.0;
static double se_cmd_abs_max_radps = 0.0;
static double se_wheel_deadband_radps = 0.0;
static double se_v_enc_mag_mps = 0.0;
static uint8_t se_stationary_now = 0u;
static uint8_t se_stationary_for_bias = 0u;

static uint8_t se_inited = 0u;

/* Wrap angle to [-pi, pi]. */
static inline double wrap_pi(double a)
{
  const double two_pi = 2.0 * M_PI;
  a = fmod(a + M_PI, two_pi);
  if (a < 0.0) { a += two_pi; }
  return a - M_PI;
}

static inline double clampd(double x, double lo, double hi)
{
  if (x < lo) { return lo; }
  if (x > hi) { return hi; }
  return x;
}

static inline double absd(double x)
{
  return (x >= 0.0) ? x : -x;
}

static inline double deadbandd(double x, double band)
{
  return (absd(x) < band) ? 0.0 : x;
}

/* Public API --------------------------------------------------------------- */

void StateEstimator_Reset(double x0_m, double y0_m, double yaw0_rad)
{
  se_x_m   = x0_m;
  se_y_m   = y0_m;
  se_yaw_r = wrap_pi(yaw0_rad);
  se_vx_body_mps = 0.0;
  se_vy_body_mps = 0.0;
  se_wz_body_rps = 0.0;
  se_vx_world_mps = 0.0;
  se_vy_world_mps = 0.0;
  se_vx_imu_body_mps = 0.0;
  se_vy_imu_body_mps = 0.0;
  se_ax_filt_body_mps2 = 0.0;
  se_ay_filt_body_mps2 = 0.0;
  se_ax_bias_body_mps2 = 0.0;
  se_ay_bias_body_mps2 = 0.0;
  se_vx_enc_prev_mps = 0.0;
  se_vy_enc_prev_mps = 0.0;
  se_ax_enc_filt_mps2 = 0.0;
  se_ay_enc_filt_mps2 = 0.0;
  se_slip_ratio_filt = 0.0;
  se_slip_innovation_mps2 = 0.0;
  se_w_enc_last = 1.0;
  se_w_imu_last = 0.0;
  se_vx_enc_body_last = 0.0;
  se_vy_enc_body_last = 0.0;
  se_stationary_time_s = 0.0;
  se_cmd_abs_max_radps = 0.0;
  se_wheel_deadband_radps = 0.0;
  se_v_enc_mag_mps = 0.0;
  se_stationary_now = 0u;
  se_stationary_for_bias = 0u;
  se_inited = 1u;
}

/* Get current pose without updating. */
void StateEstimator_GetPose(double pose_out[3])
{
  if (!pose_out) { return; }
  pose_out[0] = se_x_m;
  pose_out[1] = se_y_m;
  pose_out[2] = se_yaw_r;
}

void StateEstimator_GetBodyVelocity(double vel_body_out[3])
{
  if (!vel_body_out) { return; }
  vel_body_out[0] = se_vx_body_mps;
  vel_body_out[1] = se_vy_body_mps;
  vel_body_out[2] = se_wz_body_rps;
}

void StateEstimator_GetSlipDebug(double debug_out[6])
{
  if (!debug_out) { return; }
  debug_out[0] = se_slip_ratio_filt;
  debug_out[1] = se_w_enc_last;
  debug_out[2] = se_w_imu_last;
  debug_out[3] = se_slip_innovation_mps2;
  debug_out[4] = se_ax_bias_body_mps2;
  debug_out[5] = se_ay_bias_body_mps2;
}

void StateEstimator_GetTuningDebug(double debug_out[6])
{
  if (!debug_out) { return; }
  debug_out[0] = se_cmd_abs_max_radps;
  debug_out[1] = se_wheel_deadband_radps;
  debug_out[2] = se_v_enc_mag_mps;
  debug_out[3] = (double)se_stationary_now;
  debug_out[4] = (double)se_stationary_for_bias;
  debug_out[5] = se_stationary_time_s;
}

void StateEstimator_GetFuseDebug(double debug_out[6])
{
  if (!debug_out) { return; }
  debug_out[0] = se_w_enc_last;
  debug_out[1] = se_w_imu_last;
  debug_out[2] = se_vx_enc_body_last;
  debug_out[3] = se_vy_enc_body_last;
  debug_out[4] = se_vx_body_mps;
  debug_out[5] = se_vy_body_mps;
}

/* Update estimator (yaw from IMU only).
 *
 * Inputs:
 *   w_rad_s[3]   : wheel speeds [w1 w2 w3] in rad/s (encoder-derived).
 *   dt_s         : timestep in seconds.
 *   imu_yaw_rad  : IMU yaw in radians (ground truth).
 *   imu_ax_body_mps2 : IMU linear accel X in body frame (m/s^2).
 *   imu_ay_body_mps2 : IMU linear accel Y in body frame (m/s^2).
 *
 * Output:
 *   pose_out[3]  : [x y yaw] after update (meters, meters, radians).
 */
void StateEstimator_Update(const double w_rad_s[3],
                           double dt_s,
                           double imu_yaw_rad,
                           double imu_ax_body_mps2,
                           double imu_ay_body_mps2,
                           double pose_out[3])
{
  uint8_t stationary_now = 0u;
  uint8_t stationary_for_bias = 0u;

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

  /* Prevent unstable one-step jumps on delayed loop cycles. */
  dt_s = clampd(dt_s, 1e-4, 0.05);

  const double yaw_for_rate = wrap_pi(imu_yaw_rad);

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
  const double w1_raw = STATE_EST_W1_SIGN * w_rad_s[0];
  const double w2_raw = STATE_EST_W2_SIGN * w_rad_s[1];
  const double w3_raw = STATE_EST_W3_SIGN * w_rad_s[2];

  const double cmd_abs_max = fmax(absd((double)speed[0]),
                                  fmax(absd((double)speed[1]), absd((double)speed[2])));
  const double cmd_low = STATE_EST_CMD_DEADBAND_LOW_RADPS;
  const double cmd_high = fmax(STATE_EST_CMD_DEADBAND_HIGH_RADPS, cmd_low + 1e-6);
  const double cmd_scale = clampd((cmd_abs_max - cmd_low) / (cmd_high - cmd_low), 0.0, 1.0);
  const double wheel_deadband_radps = STATE_EST_WHEEL_DEADBAND_RADPS * (1.0 - cmd_scale);
  se_cmd_abs_max_radps = cmd_abs_max;
  se_wheel_deadband_radps = wheel_deadband_radps;

  /* Suppress small wheel slack/backlash motion in odometry kinematics. */
  const double w1 = deadbandd(w1_raw, wheel_deadband_radps);
  const double w2 = deadbandd(w2_raw, wheel_deadband_radps);
  const double w3 = deadbandd(w3_raw, wheel_deadband_radps);

  /* Body-frame velocities from encoders (matches your MATLAB kinematics). */
  const double k_s3 = 0.57735026918962576451; /* sqrt(3)/3 */
  const double vx_enc_body = r * (k_s3 * (w2 - w3));
  const double vy_enc_body = r * ((2.0/3.0) * w1 - (1.0/3.0) * (w2 + w3));
  const double wz_body = r * (w1 + w2 + w3) / (3.0 * Leff);
  se_vx_enc_body_last = vx_enc_body;
  se_vy_enc_body_last = vy_enc_body;

  /* --- Slip-aware fusion: accel-integrated velocity + encoder velocity ---
   * The IMU branch is leak-integrated to prevent drift, and we adaptively
   * reduce encoder trust when encoder-inferred acceleration disagrees with IMU.
   */
  {
    const double tau_acc_lpf_s = fmax(STATE_EST_ACC_LPF_TAU_S, 1e-3);
    const double tau_enc_acc_lpf_s = fmax(STATE_EST_ENC_ACCEL_LPF_TAU_S, 1e-3);
    const double tau_vel_leak_s = fmax(STATE_EST_VEL_LEAK_TAU_S, 1e-3);
    const double slip_span = fmax(STATE_EST_SLIP_HIGH_MPS2 - STATE_EST_SLIP_LOW_MPS2, 1e-6);
    const double low_speed_ref = fmax(STATE_EST_LOW_SPEED_MPS, 1e-6);

    double ax_meas = isfinite(imu_ax_body_mps2) ? imu_ax_body_mps2 : 0.0;
    double ay_meas = isfinite(imu_ay_body_mps2) ? imu_ay_body_mps2 : 0.0;

    const double w_abs_max = fmax(absd(w1_raw), fmax(absd(w2_raw), absd(w3_raw)));
    const double acc_mag_raw = sqrt(ax_meas * ax_meas + ay_meas * ay_meas);
    const double v_enc_mag = sqrt(vx_enc_body * vx_enc_body + vy_enc_body * vy_enc_body);
    se_v_enc_mag_mps = v_enc_mag;
    const uint8_t wheel_quiet = (w_abs_max < STATE_EST_STATIONARY_WHEEL_RADPS) ? 1u : 0u;
    const uint8_t enc_speed_quiet = (v_enc_mag < STATE_EST_STATIONARY_ENC_SPEED_MPS) ? 1u : 0u;
    const uint8_t accel_quiet = (acc_mag_raw < STATE_EST_STATIONARY_ACC_MPS2) ? 1u : 0u;
    const uint8_t stationary = (accel_quiet && (wheel_quiet || enc_speed_quiet)) ? 1u : 0u;
    stationary_now = stationary;
    se_stationary_now = stationary_now;
    if (stationary_now)
    {
      se_stationary_time_s += dt_s;
    }
    else
    {
      se_stationary_time_s = 0.0;
    }

    stationary_for_bias =
        (se_stationary_time_s >= STATE_EST_BIAS_STATIONARY_HOLD_S) ? 1u : 0u;
    se_stationary_for_bias = stationary_for_bias;

    /* Adapt accel bias only when likely stationary. */
    if (stationary_for_bias)
    {
      se_ax_bias_body_mps2 += STATE_EST_BIAS_ADAPT * (ax_meas - se_ax_bias_body_mps2);
      se_ay_bias_body_mps2 += STATE_EST_BIAS_ADAPT * (ay_meas - se_ay_bias_body_mps2);
    }

    ax_meas -= se_ax_bias_body_mps2;
    ay_meas -= se_ay_bias_body_mps2;

    /* Robustify accel: clip spikes and deadband tiny vibration. */
    ax_meas = clampd(ax_meas, -STATE_EST_ACCEL_CLIP_MPS2, STATE_EST_ACCEL_CLIP_MPS2);
    ay_meas = clampd(ay_meas, -STATE_EST_ACCEL_CLIP_MPS2, STATE_EST_ACCEL_CLIP_MPS2);
    if (absd(ax_meas) < STATE_EST_ACCEL_DEADBAND_MPS2) { ax_meas = 0.0; }
    if (absd(ay_meas) < STATE_EST_ACCEL_DEADBAND_MPS2) { ay_meas = 0.0; }

    const double alpha_acc = dt_s / (tau_acc_lpf_s + dt_s);
    se_ax_filt_body_mps2 += alpha_acc * (ax_meas - se_ax_filt_body_mps2);
    se_ay_filt_body_mps2 += alpha_acc * (ay_meas - se_ay_filt_body_mps2);

    /* Leak-integrated IMU velocity (body frame). */
    const double leak = exp(-dt_s / tau_vel_leak_s);
    se_vx_imu_body_mps = leak * (se_vx_imu_body_mps + se_ax_filt_body_mps2 * dt_s);
    se_vy_imu_body_mps = leak * (se_vy_imu_body_mps + se_ay_filt_body_mps2 * dt_s);

    /* Stationary zero-velocity update to suppress drift. */
    if (stationary_now)
    {
      const double z = clampd(STATE_EST_ZUPT_BLEND, 0.0, 1.0);
      se_vx_imu_body_mps *= (1.0 - z);
      se_vy_imu_body_mps *= (1.0 - z);

      if (stationary_for_bias)
      {
        if (absd(se_vx_imu_body_mps) < STATE_EST_HARD_ZERO_VEL_MPS) { se_vx_imu_body_mps = 0.0; }
        if (absd(se_vy_imu_body_mps) < STATE_EST_HARD_ZERO_VEL_MPS) { se_vy_imu_body_mps = 0.0; }
      }
    }

    /* Encoder-implied acceleration for slip innovation. */
    double ax_enc = (vx_enc_body - se_vx_enc_prev_mps) / dt_s;
    double ay_enc = (vy_enc_body - se_vy_enc_prev_mps) / dt_s;
    se_vx_enc_prev_mps = vx_enc_body;
    se_vy_enc_prev_mps = vy_enc_body;

    ax_enc = clampd(ax_enc, -STATE_EST_ACCEL_CLIP_MPS2, STATE_EST_ACCEL_CLIP_MPS2);
    ay_enc = clampd(ay_enc, -STATE_EST_ACCEL_CLIP_MPS2, STATE_EST_ACCEL_CLIP_MPS2);

    {
      const double alpha_enc_acc = dt_s / (tau_enc_acc_lpf_s + dt_s);
      se_ax_enc_filt_mps2 += alpha_enc_acc * (ax_enc - se_ax_enc_filt_mps2);
      se_ay_enc_filt_mps2 += alpha_enc_acc * (ay_enc - se_ay_enc_filt_mps2);
    }

    const double dax = se_ax_filt_body_mps2 - se_ax_enc_filt_mps2;
    const double day = se_ay_filt_body_mps2 - se_ay_enc_filt_mps2;
    const double slip_innovation = sqrt(dax * dax + day * day);
    se_slip_innovation_mps2 = slip_innovation;

    double slip_ratio = 0.0;
    if (slip_innovation >= STATE_EST_SLIP_HIGH_MPS2) {
      slip_ratio = 1.0;
    } else if (slip_innovation > STATE_EST_SLIP_LOW_MPS2) {
      slip_ratio = (slip_innovation - STATE_EST_SLIP_LOW_MPS2) / slip_span;
    }

    /* Holonomic acceleration-consistency scrub detector:
     * Compare IMU acceleration components along/normal to encoder velocity
     * direction against encoder-derived acceleration components. This remains
     * valid when translation and yaw are decoupled.
     */
    if (v_enc_mag > STATE_EST_HOLO_ACCEL_CHECK_MIN_SPEED_MPS)
    {
      const double tx = vx_enc_body / v_enc_mag;
      const double ty = vy_enc_body / v_enc_mag;
      const double nx = -ty;
      const double ny = tx;

      const double a_imu_t = (se_ax_filt_body_mps2 * tx) + (se_ay_filt_body_mps2 * ty);
      const double a_imu_n = (se_ax_filt_body_mps2 * nx) + (se_ay_filt_body_mps2 * ny);
      const double a_enc_t = (se_ax_enc_filt_mps2 * tx) + (se_ay_enc_filt_mps2 * ty);
      const double a_enc_n = (se_ax_enc_filt_mps2 * nx) + (se_ay_enc_filt_mps2 * ny);

      const double mismatch_t = absd(a_imu_t - a_enc_t);
      const double mismatch_n = absd(a_imu_n - a_enc_n);
      const double mismatch = sqrt(mismatch_t * mismatch_t + mismatch_n * mismatch_n);

      if (mismatch > STATE_EST_HOLO_ACCEL_MISMATCH_LOW_MPS2)
      {
        const double span = fmax(STATE_EST_HOLO_ACCEL_MISMATCH_HIGH_MPS2 - STATE_EST_HOLO_ACCEL_MISMATCH_LOW_MPS2, 1e-6);
        double holo_boost = (mismatch - STATE_EST_HOLO_ACCEL_MISMATCH_LOW_MPS2) / span;
        holo_boost = clampd(holo_boost, 0.0, 1.0);
        slip_ratio += STATE_EST_HOLO_SLIP_BOOST_MAX * holo_boost;
      }
    }

    /* Slight extra skepticism at very low wheel speed, where scrub is common.
     * Do not apply this while stationary, otherwise idle slip never reaches 0.
     */
    if (!stationary)
    {
      const double v_enc_mag = sqrt(vx_enc_body * vx_enc_body + vy_enc_body * vy_enc_body);
      const double low_speed_boost = (v_enc_mag < low_speed_ref)
                       ? (1.0 - (v_enc_mag / low_speed_ref))
                                     : 0.0;
      slip_ratio += STATE_EST_LOW_SPEED_BOOST_MAX * low_speed_boost;
    }
    else
    {
      slip_ratio = 0.0;
    }
    slip_ratio = clampd(slip_ratio, 0.0, 1.0);

    /* Smooth confidence to avoid rapid toggling. */
    {
      const double tau_slip = fmax(STATE_EST_SLIP_SMOOTH_TAU_S, 1e-3);
      const double alpha_slip = dt_s / (tau_slip + dt_s);
      se_slip_ratio_filt += alpha_slip * (slip_ratio - se_slip_ratio_filt);
      se_slip_ratio_filt = clampd(se_slip_ratio_filt, 0.0, 1.0);
    }

    /* Keep encoder dominant when healthy, but allow IMU branch to pull during slip. */
    {
      const double w_floor = clampd(STATE_EST_ENCODER_WEIGHT_FLOOR, 0.0, 1.0);
      const double w_enc = w_floor + (1.0 - w_floor) * (1.0 - se_slip_ratio_filt);
      const double w_imu = 1.0 - w_enc;
      se_w_enc_last = w_enc;
      se_w_imu_last = w_imu;
      se_vx_body_mps = w_enc * vx_enc_body + w_imu * se_vx_imu_body_mps;
      se_vy_body_mps = w_enc * vy_enc_body + w_imu * se_vy_imu_body_mps;
    }

    if (stationary_now)
    {
      const double z = clampd(STATE_EST_ZUPT_BLEND, 0.0, 1.0);
      se_vx_body_mps *= (1.0 - z);
      se_vy_body_mps *= (1.0 - z);

      if (stationary_for_bias)
      {
        if (absd(se_vx_body_mps) < STATE_EST_HARD_ZERO_VEL_MPS) { se_vx_body_mps = 0.0; }
        if (absd(se_vy_body_mps) < STATE_EST_HARD_ZERO_VEL_MPS) { se_vy_body_mps = 0.0; }
      }
    }
  }

  /* Yaw is ground truth from IMU. */
  const double yaw = yaw_for_rate;
  se_yaw_r = yaw;

  /* Rotate body -> world using IMU yaw. */
  const double c = cos(yaw);
  const double s = sin(yaw);
  const double vx_world =  se_vx_body_mps * c - se_vy_body_mps * s;
  const double vy_world =  se_vx_body_mps * s + se_vy_body_mps * c;

  se_wz_body_rps = wz_body;
  se_vx_world_mps = vx_world;
  se_vy_world_mps = vy_world;

  /* Integrate position (Euler), but suppress tiny residual creep at stop. */
  {
    const double v_world_mag = sqrt(vx_world * vx_world + vy_world * vy_world);
    if (!(stationary_now && (v_world_mag < STATE_EST_MIN_INTEGRATE_SPEED_MPS)))
    {
      se_x_m += dt_s * vx_world;
      se_y_m += dt_s * vy_world;
    }
  }

  if (pose_out)
  {
    pose_out[0] = se_x_m;
    pose_out[1] = se_y_m;
    pose_out[2] = se_yaw_r;
  }
}
