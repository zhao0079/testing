/*
 * control_quadrotor_trajectory.h
 *
 *  Created on: 02.03.2011
 *      Author: mackayl
 */

#ifndef CONTROL_QUADROTOR_TRAJECTORY_H_
#define CONTROL_QUADROTOR_TRAJECTORY_H_

#include "pid.h"
/**
 * @file
 *   @brief Definition of the class control_quadrotor_trajectory.
 *
 *   @author Laurens Mackay
 *
 */


PID_t velocity_controller;
PID_t lateral_controller;
PID_t altitude_controller;

void control_quadrotor_trajectory_init(void);

void control_quadrotor_trajectory(void);

#endif /* CONTROL_QUADROTOR_TRAJECTORY_H_ */
