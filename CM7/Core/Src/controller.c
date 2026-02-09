#include "controller.h"
#include "main.h"
#include <math.h>
#include "cmd.h"

// Definitions for globals declared in controller.h
double k_p = 0.5;          // position P gain in body frame (small correction only)
double aumax_body = 0.5;   // max translational accel in body frame (m/s^2)
double umax = 8;        // max wheel speed (rad/s)
double aumax = 0.3/0.09; // max wheel angular accel (rad/s^2)
double jerkmax = 50*0.05; // max wheel angular jerk (rad/s^3)
double wmax = 1;         // max yaw rate (rad/s)
double dwmax = 1;        // max yaw accel (rad/s^2)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if defined(__GNUC__) || defined(__clang__)
#define ALWAYS_INLINE static inline __attribute__((always_inline))
#else
#define ALWAYS_INLINE static inline
#endif

/* Precompute commonly used constants for faster math */
static const double TWO_PI = 2.0 * M_PI;
static const double SQRT3_2 = 0.86602540378443864676; /* sqrt(3)/2 */

// ---------- helpers ----------
ALWAYS_INLINE double clampd(double x, double lo, double hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

ALWAYS_INLINE double signum(double x) {
    return (x > 0.0) - (x < 0.0);
}

ALWAYS_INLINE double wrapPi(double a) {
    /* Fast in-place range reduction to (-pi, pi] - assumes 'a' is not huge. */
    while (a > M_PI)  a -= TWO_PI;
    while (a <= -M_PI) a += TWO_PI;
    return a;
}

ALWAYS_INLINE double norm2_2(const double v[2]) {
    /* Use sqrt(x*x + y*y) which is usually faster than hypot on embedded targets */
    const double x = v[0];
    const double y = v[1];
    return sqrt(x*x + y*y);
}

ALWAYS_INLINE double maxabs3(const double v[3]) {
    const double a0 = fabs(v[0]);
    const double a1 = fabs(v[1]);
    const double a2 = fabs(v[2]);
    double m = (a0 > a1) ? a0 : a1;
    return (m > a2) ? m : a2;
}

// Omni 3-wheel (0,120,240) inverse kinematics consistent with your forward equations:
// vx = r*(sqrt(3)/3)*(w2 - w3)
// vy = r*(2/3*w1 - 1/3*(w2 + w3))
// w  = r*(w1 + w2 + w3)/(3L)
ALWAYS_INLINE void inverse_kinematics(double vx_body, double vy_body, double omega,
                               double u_out[3])
{
    const double r_inv = 1.0 / rw;
    const double Lc = L; /* use global L */

    /* compute once, avoid repeated divides */
    u_out[0] = r_inv * ( vy_body + Lc * omega );
    u_out[1] = r_inv * ( -0.5 * vy_body + (SQRT3_2) * vx_body + Lc * omega );
    u_out[2] = r_inv * ( -0.5 * vy_body - (SQRT3_2) * vx_body + Lc * omega );
}

static double du_prev[3] = {0};       // previous wheel delta (for jerk limiting)
static double yawrate_prev = 0.0;     // previous omega (yaw rate)
static double u_prev[3] = {0};        // previous wheel command u

void Controller_Step(const double           x[3],
                     const double           xd[5],
					 const double			vd[3],
					 int selector,
					 double dt)
{
    if (!x || !xd) return;
    //if (dt <= 0.0) dt = 1e-3;

    // -------------------------
    // current state
    // -------------------------
    const double pos[2] = { x[0], x[1] };
    const double yaw    = x[2];

    // desired state
    const double pos_d[2] = { xd[0], xd[1] };
    const double yaw_d    = xd[2];
    const double vx_world = xd[3];  // Feedforward velocity from trajectory
    const double vy_world = xd[4];
    
    double v_des_body[2] = {0};
    if (selector)
    {
		// ================================
		// 1) FEEDFORWARD + ERROR CORRECTION
		// ================================
		const double c = cos(yaw);
		const double s = sin(yaw);

		// Feedforward: convert trajectory velocity to body frame
		// R' = [ c  s; -s  c]
		double v_ff_body[2];
		v_ff_body[0] =  c * vx_world + s * vy_world;
		v_ff_body[1] = -s * vx_world + c * vy_world;

		// Small position error correction
		const double d_world[2] = { pos_d[0] - pos[0], pos_d[1] - pos[1] };
		
		// d_body = R' * d_world
		double d_body[2];
		d_body[0] =  c * d_world[0] + s * d_world[1];
		d_body[1] = -s * d_world[0] + c * d_world[1];

		// Use small P gain for position error correction only
		double v_corr_body[2];
		v_corr_body[0] = k_p * d_body[0];
		v_corr_body[1] = k_p * d_body[1];

		// Combined velocity (feedforward dominates)
		v_des_body[0] = v_ff_body[0] + v_corr_body[0];
		v_des_body[1] = v_ff_body[1] + v_corr_body[1];

		// Limit correction to avoid fighting trajectory (max 0.1 m/s)
		const double v_corr_mag = norm2_2(v_corr_body);
		if (v_corr_mag > 0.1) {
			const double scale = 0.1 / v_corr_mag;
			v_corr_body[0] *= scale;
			v_corr_body[1] *= scale;
			v_des_body[0] = v_ff_body[0] + v_corr_body[0];
			v_des_body[1] = v_ff_body[1] + v_corr_body[1];
		}
    }
	else if (!selector)
	{
        const double c = cos(yaw);
        const double s = sin(yaw);

        // body = R^T * world
        v_des_body[0] =  c*vd[0] + s*vd[1];
        v_des_body[1] = -s*vd[0] + c*vd[1];
	}
    
    // No translational accel limiting - using velocity directly
    const double vx_body = v_des_body[0];
    const double vy_body = v_des_body[1];

    // ================================
    // 3) YAW CONTROL
    // ================================
    double omega = 0;
    if (selector)
    {
		const double e = wrapPi(yaw_d - yaw);

		const double yaw_dead = (1.0 * M_PI / 180.0);  // 1 deg
		const double yaw_lin  = (6.0 * M_PI / 180.0);  // 6 deg

		const double ae = fabs(e);
		if (ae < yaw_dead) {
			omega = 0.0;
		} else if (ae < yaw_lin) {
			const double k_small = wmax / yaw_lin;
			omega = k_small * e;
		} else {
			omega = wmax * signum(e);
		}

		omega = clampd(omega, -wmax, wmax);
    }
    else if (!selector)
    {
        omega = clampd(vd[2], -wmax, wmax);
    }
    // yaw accel limit
    double domega = omega - yawrate_prev;
    const double domega_max = dwmax * dt;
    domega = clampd(domega, -domega_max, domega_max);

    omega = yawrate_prev + domega;
    yawrate_prev = omega;

    // ================================
    // 4) INVERSE KINEMATICS + SATURATION SPLIT
    // ================================
    double u_rot[3], u_trans[3];
    inverse_kinematics(0.0,     0.0,     omega, u_rot);
    inverse_kinematics(vx_body, vy_body, 0.0,   u_trans);

    // Find max s in [0,1] such that |u_rot + s*u_trans| <= umax for all wheels
    double s_lo = 0.0;
    double s_hi = 1.0;

    for (int i = 0; i < 3; i++) {
        const double a = u_trans[i];
        const double b = u_rot[i];

        if (fabs(a) < 1e-12) {
            if (fabs(b) > umax) {
                s_hi = -1.0; // impossible even at s=0
            }
            continue;
        }

        // -umax <= b + s*a <= umax
        const double s1 = (-umax - b) / a;
        const double s2 = ( umax - b) / a;

        const double smin = (s1 < s2) ? s1 : s2;
        const double smax = (s1 > s2) ? s1 : s2;

        if (smin > s_lo) s_lo = smin;
        if (smax < s_hi) s_hi = smax;
    }

    double s_scale;
    if (s_hi < s_lo) {
        s_scale = 0.0;                // no room for translation -> keep yaw
    } else {
        s_scale = clampd(s_hi, 0.0, 1.0); // largest feasible
    }

    double u_des[3] = {
        u_rot[0] + s_scale * u_trans[0],
        u_rot[1] + s_scale * u_trans[1],
        u_rot[2] + s_scale * u_trans[2]
    };

    // ================================
    // 6) WHEEL ACCEL + JERK LIMITING
    // ================================
    double du_des[3] = {
        u_des[0] - u_prev[0],
        u_des[1] - u_prev[1],
        u_des[2] - u_prev[2]
    };

    // Accel limit (inf-norm like MATLAB)
    const double du_max = aumax * dt;
    const double du_inf = maxabs3(du_des);

    double du_acc[3];
    if (du_inf > du_max && du_inf > 0.0) {
        const double scale = du_max / du_inf;
        du_acc[0] = du_des[0] * scale;
        du_acc[1] = du_des[1] * scale;
        du_acc[2] = du_des[2] * scale;
    } else {
        du_acc[0] = du_des[0];
        du_acc[1] = du_des[1];
        du_acc[2] = du_des[2];
    }

    // Jerk limit on du (inf-norm)
    double ddu[3] = {
        du_acc[0] - du_prev[0],
        du_acc[1] - du_prev[1],
        du_acc[2] - du_prev[2]
    };

    const double ddu_max = jerkmax * dt;
    const double ddu_inf = maxabs3(ddu);

    if (ddu_inf > ddu_max && ddu_inf > 0.0) {
        const double scale = ddu_max / ddu_inf;
        ddu[0] *= scale;
        ddu[1] *= scale;
        ddu[2] *= scale;
    }

    double du[3] = {
        du_prev[0] + ddu[0],
        du_prev[1] + ddu[1],
        du_prev[2] + ddu[2]
    };

    double u_cmd[3] = {
        u_prev[0] + du[0],
        u_prev[1] + du[1],
        u_prev[2] + du[2]
    };

    // outputs
    speed[0] = u_cmd[0];
    speed[1] = u_cmd[1];
    speed[2] = u_cmd[2];

    // update persistent state
    du_prev[0] = du[0];
    du_prev[1] = du[1];
    du_prev[2] = du[2];

    u_prev[0]  = u_cmd[0];
    u_prev[1]  = u_cmd[1];
    u_prev[2]  = u_cmd[2];
}
