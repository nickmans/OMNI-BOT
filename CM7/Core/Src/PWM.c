#include "main.h"
#include <stdbool.h>
#include <math.h>
#include "cmd.h"

extern TIM_HandleTypeDef htim2;

static const double DB_RPM       = 5.0;

// MD20A with Saturn 5303 planetary gearmotor specs
static const double Kp = 2.5, Ki = 8.0;
static const double RPM_FS[3] = { 230, 230, 230 };  // Saturn 5303 free speed ~230 RPM

static double integ[3] = {0};
static double sp_rpm[3] = {0};

static const uint32_t CH[3] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3 };
static const double sixtyon2pi = 9.54929658551;

#define ZERO_GUARD_RPM  1.0

// Helper function to set motor direction via GPIO
static inline void set_motor_dir(int motor_idx, int direction)
{
    GPIO_TypeDef* ports[3] = { DIR1_GPIO_Port, DIR2_GPIO_Port, DIR3_GPIO_Port };
    uint16_t pins[3] = { DIR1_Pin, DIR2_Pin, DIR3_Pin };
    
    // direction: +1 = forward (HIGH), -1 = reverse (LOW)
    HAL_GPIO_WritePin(ports[motor_idx], pins[motor_idx], 
                      (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

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

    // Store previous *signed* duty cycle command (0 to PWM_RANGE)
    static double cmd_prev[N_MOTORS] = {0};

    const double FLIP_THRESH_RPM   = 10.0;                 // must be near-stop to reverse
    const double FLIP_THRESH_CMD   = (double)PWM_RANGE * 0.05; // must be near-zero to reverse

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
            const double prev_cmd_mag  = cmd_prev[i];

            if (wheel_rpm_mag < FLIP_THRESH_RPM && prev_cmd_mag < FLIP_THRESH_CMD)
            {
                new_dir = req_dir;
            }
        }

        dir_state[i] = new_dir;
        set_motor_dir(i, new_dir);  // Set GPIO direction pin
    }

    // ---------------------------------
    // 2) PI + FF (magnitude), 0-100% duty cycle
    // ---------------------------------
    for (int i = 0; i < N_MOTORS; i++)
    {
        const int chosen_dir = dir_state[i];

        // Determine what sign the setpoint wants (if outside deadband)
        const double sp = sp_rpm[i];
        const int want_dir = (fabs(sp) > DB_RPM) ? (sp >= 0.0 ? +1 : -1) : chosen_dir;

        // If the setpoint wants opposite sign but we haven't flipped yet -> coast at zero
        double sp_mag = fabs(sp);
        if (want_dir != chosen_dir)
        {
            sp_mag = 0.0; // command zero until flip allowed
        }

        if (sp_mag < ZERO_GUARD_RPM)
        {
            integ[i] = 0.0;
            cmd_prev[i] = 0.0;
            __HAL_TIM_SET_COMPARE(&htim2, CH[i], 0);
            continue;
        }

        const double y_meas = fabs(rpm[i]);   // magnitude feedback

        // PI on magnitude
        const double err  = sp_mag - y_meas;
        const double u_ff = (sp_mag / RPM_FS[i]) * (double)PWM_RANGE;
        const double u_fb = Kp * err + Ki * integ[i];

        double mag_cmd = u_ff + u_fb; // magnitude in duty cycle counts (0..PWM_RANGE)

        // clamp magnitude
        if (mag_cmd < 0.0) mag_cmd = 0.0;
        if (mag_cmd > (double)PWM_RANGE) mag_cmd = (double)PWM_RANGE;

        // anti-windup: only integrate if not driving deeper into saturation
        const bool sat_lo = (mag_cmd <= 0.0);
        const bool sat_hi = (mag_cmd >= (double)PWM_RANGE);
        const bool drives_deeper = (sat_lo && err < 0.0) || (sat_hi && err > 0.0);

        if (!drives_deeper)
        {
            integ[i] += err * dt;
        }

        // cap integrator
        const double INTEG_CAP = (0.20 * (double)PWM_RANGE) / fmax(1e-6, Ki);
        if (integ[i] >  INTEG_CAP) integ[i] =  INTEG_CAP;
        if (integ[i] < -INTEG_CAP) integ[i] = -INTEG_CAP;

        // --- Slew limiting (DOWN + UP), magnitude-based ---
        const double SLEW_DOWN = (double)PWM_RANGE * 0.15;  // toward zero
        const double SLEW_UP   = (double)PWM_RANGE * 0.15;  // away from zero

        const double prev_mag = cmd_prev[i];
        const double targ_mag = mag_cmd;

        double final_cmd = targ_mag;

        if (targ_mag < prev_mag)
        {
            // Slew down (limit how fast duty cycle can decrease)
            const double dm = prev_mag - targ_mag;
            if (dm > SLEW_DOWN)
            {
                final_cmd = prev_mag - SLEW_DOWN;
            }
        }
        else if (targ_mag > prev_mag)
        {
            // Slew up (limit how fast duty cycle can increase)
            const double dm = targ_mag - prev_mag;
            if (dm > SLEW_UP)
            {
                final_cmd = prev_mag + SLEW_UP;
            }
        }

        // Save for next loop
        cmd_prev[i] = final_cmd;

        // Clamp to valid range and set PWM
        if (final_cmd > (double)PWM_MAX) final_cmd = (double)PWM_MAX;
        if (final_cmd < (double)PWM_MIN) final_cmd = (double)PWM_MIN;
        static int counter = 0;
        /*if (counter++%10==0)
        {
        	printf("%.3f ",final_cmd);
        }*/
        __HAL_TIM_SET_COMPARE(&htim2, CH[i], (uint32_t)final_cmd);
    }
    //printf("\n");
}
