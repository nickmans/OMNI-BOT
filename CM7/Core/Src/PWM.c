#include "main.h"
#include <stdbool.h>
#include <math.h>
#include "cmd.h"

extern TIM_HandleTypeDef htim2;

static const double DB_RPM       = 5.0;

static const double Kp = 3, Ki = 10;
static const double RPM_FS[3] = { 110, 108, 105 };

static double integ[3] = {0};
static double sp_rpm[3] = {0};

static const uint32_t CH[3] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 };
static const double sixtyon2pi = 9.54929658551;

#define ZERO_GUARD_RPM  1.0

static inline int sign_with_deadband(double x, double deadband)
{
    if (x >  deadband) return +1;
    if (x < -deadband) return -1;
    return 0; // inside deadband
}

void PWM(double rpm[3], double dt)
{
    enum { N_MOTORS = 3 };

    // Per-wheel direction latch (+1 / -1)
    static int dir_state[N_MOTORS] = { +1, +1, +1 };
    static const double MOTOR_POL[3] = { +1.0, -1.0, +1.0 }; // motor 2 reversed

    // Store previous *signed* command delta from NEUTRAL (in timer counts)
    static double cmd_prev_delta[N_MOTORS] = {0};

    const double FLIP_THRESH_RPM   = 10.0;                 // must be near-stop to reverse
    const double FLIP_THRESH_CMD   = (double)VICTOR_RANGE * 0.05; // must be near-neutral to reverse

    // ---------------------------------
    // 1) Direction selection / safe flip
    // ---------------------------------
    for (int i = 0; i < N_MOTORS; i++)
    {
        sp_rpm[i] = MOTOR_POL[i] * speed[i] * sixtyon2pi;

        const int req_dir = sign_with_deadband(sp_rpm[i], DB_RPM); // -1/0/+1
        int new_dir = dir_state[i];

        if (req_dir == 0)
        {
            new_dir = dir_state[i]; // keep last in deadband
        }
        else if (req_dir != dir_state[i])
        {
            const double wheel_rpm_mag = fabs(rpm[i]);
            const double prev_cmd_mag  = fabs(cmd_prev_delta[i]);

            if (wheel_rpm_mag < FLIP_THRESH_RPM && prev_cmd_mag < FLIP_THRESH_CMD)
            {
                new_dir = req_dir;
            }
        }

        dir_state[i] = new_dir;
        // NOTE: no set_motor_dir() anymore for Victor SP
    }

    // ---------------------------------
    // 2) PI + FF (magnitude), output around NEUTRAL
    // ---------------------------------
    for (int i = 0; i < N_MOTORS; i++)
    {
        const int chosen_dir = dir_state[i];

        // Determine what sign the setpoint wants (if outside deadband)
        const double sp = sp_rpm[i];
        const int want_dir = (fabs(sp) > DB_RPM) ? (sp >= 0.0 ? +1 : -1) : chosen_dir;

        // If the setpoint wants opposite sign but we haven't flipped yet -> coast at neutral
        double sp_mag = fabs(sp);
        if (want_dir != chosen_dir)
        {
            sp_mag = 0.0; // safest: command neutral until flip allowed
            // alternatively: sp_mag *= LOSER_SCALE;
        }

        if (sp_mag < ZERO_GUARD_RPM)
        {
            integ[i] = 0.0;
            cmd_prev_delta[i] = 0.0;
            __HAL_TIM_SET_COMPARE(&htim2, CH[i], NEUTRAL);
            continue;
        }

        const double y_meas = fabs(rpm[i]);   // magnitude feedback

        // PI on magnitude
        const double err  = sp_mag - y_meas;
        const double u_ff = (sp_mag / RPM_FS[i]) * (double)VICTOR_RANGE;
        const double u_fb = Kp * err + Ki * integ[i];

        double mag_cmd = u_ff + u_fb; // magnitude in "counts" (0..VICTOR_RANGE)

        // clamp magnitude
        if (mag_cmd < 0.0) mag_cmd = 0.0;
        if (mag_cmd > (double)VICTOR_RANGE) mag_cmd = (double)VICTOR_RANGE;

        // anti-windup: only integrate if not driving deeper into saturation
        const bool sat_lo = (mag_cmd <= 0.0);
        const bool sat_hi = (mag_cmd >= (double)VICTOR_RANGE);
        const bool drives_deeper = (sat_lo && err < 0.0) || (sat_hi && err > 0.0);

        if (!drives_deeper)
        {
            integ[i] += err * dt;
        }

        // cap integrator
        const double INTEG_CAP = (0.20 * (double)VICTOR_RANGE) / fmax(1e-6, Ki);
        if (integ[i] >  INTEG_CAP) integ[i] =  INTEG_CAP;
        if (integ[i] < -INTEG_CAP) integ[i] = -INTEG_CAP;

        // Build signed delta around NEUTRAL
		double target_delta = (double)chosen_dir * mag_cmd;

		// --- Slew limiting (DOWN + UP), magnitude-based ---
		// "counts per control tick" (same style as your old code)
		const double SLEW_DOWN = (double)VICTOR_RANGE * 0.15;  // toward neutral
		const double SLEW_UP   = (double)VICTOR_RANGE * 0.15;  // away from neutral (tune)

		const double prev_delta = cmd_prev_delta[i];
		const double prev_mag   = fabs(prev_delta);
		const double targ_mag   = fabs(target_delta);

		// If target wants opposite sign while we still have nonzero output,
		// force a decay-to-zero first (no sign flipping via slew).
		const bool sign_mismatch = (prev_mag > 0.0) && (prev_delta * target_delta < 0.0);

		if (sign_mismatch)
		{
		   // Decay magnitude toward 0, keep previous sign
		   double new_mag = prev_mag - SLEW_DOWN;
		   if (new_mag < 0.0) new_mag = 0.0;
		   target_delta = copysign(new_mag, prev_delta);
		}
		else if (targ_mag < prev_mag)
		{
		   // Slew down (limit how fast magnitude can decrease)
		   const double dm = prev_mag - targ_mag;
		   if (dm > SLEW_DOWN)
		   {
			   const double new_mag = prev_mag - SLEW_DOWN;
			   target_delta = copysign(new_mag, prev_delta); // keep sign while decaying
		   }
		}
		else if (targ_mag > prev_mag)
		{
		   // Slew up (limit how fast magnitude can increase)
		   const double dm = targ_mag - prev_mag;
		   if (dm > SLEW_UP)
		   {
			   const double new_mag = prev_mag + SLEW_UP;
			   target_delta = copysign(new_mag, target_delta); // use target sign
		   }
		}

		// Save for flip gating next loop
		cmd_prev_delta[i] = target_delta;

        // Convert to absolute compare value and clamp
        double cmd = (double)NEUTRAL + target_delta;
        if (cmd > (double)MAXf) cmd = (double)MAXf;
        if (cmd < (double)MAXb) cmd = (double)MAXb;

        __HAL_TIM_SET_COMPARE(&htim2, CH[i], (uint32_t)cmd);
    }
}
