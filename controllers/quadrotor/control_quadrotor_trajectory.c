/*
 * control_quadrotor_trajectory.c
 *
 *  Created on: 02.03.2011
 *      Author: mackayl
 */
#include "control_quadrotor_trajectory.h"

#define CONTROL_PID_TRAJECTORY_INTERVAL	0.020

void control_quadrotor_trajectory_init(void){

	pid_init(&velocity_controller, 10, 0, 0,
			0, PID_MODE_DERIVATIV_SET, 150);//150
	pid_init(&lateral_controller, 10, 0, 0,
			0, PID_MODE_DERIVATIV_SET, 151);//151
	pid_init(&altitude_controller, 10, 0, 0,
			0, PID_MODE_DERIVATIV_SET, 152);

}

void control_quadrotor_trajectory(void){

}
