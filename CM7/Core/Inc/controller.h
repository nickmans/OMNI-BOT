/*
 * controller.h
 *
 *  Created on: 17 Dec 2025
 *      Author: Nick Standing
 */

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <stdint.h>

// Gains / limits (same meaning as MATLAB sys.*)
extern double k_p;          // position P gain in body frame
extern double aumax_body;   // max translational accel in body frame (m/s^2)
extern double umax;         // max wheel speed (rad/s)
extern double aumax;        // max wheel angular accel (rad/s^2)
extern double jerkmax;      // max wheel angular jerk (rad/s^3)
extern double wmax;         // max yaw rate (rad/s)
extern double dwmax;        // max yaw accel (rad/s^2)

// Step controller
// x  = [x, y, yaw]
// xd = [x_des, y_des, yaw_des, vx_world, vy_world] (5 elements now)
void Controller_Step( const double           x[3],
                     const double           xd[5],
					 const double			vd[3],
					 int selector,
					 double dt);

void StateEstimator_Update(const double w_rad_s[3],
                           double dt_s,
                           double imu_yaw_rad,
                           double pose_out[3]);

void StateEstimator_Reset(double x0_m, double y0_m, double yaw0_rad);
void StateEstimator_ZeroImuYaw(double imu_yaw_rad);
void StateEstimator_GetPose(double pose_out[3]);


#endif
