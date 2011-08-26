/*
 * optflow_speed_kalman.c
 *
 *  Created on: 23.08.2010
 *      Author: Petri Tanskanen
 *      		Gim Hee Lee
 *      		Laurens Mackay
 */
#include "optflow_speed_kalman.h"
#include "kalman.h"

#include "debug.h"
#include "sensors.h"
#include "math.h"
#include "transformation.h"

//#define VELOCITY_HOLD 0.999f
//#define ACCELERATION_HOLD 0.99f
#define VELOCITY_HOLD 0.99f
#define ACCELERATION_HOLD 1.0f

#define VEL_KF_TIME_STEP_X (1.0f / 50.0f)
#define VEL_KF_TIME_STEP_Y (1.0f / 50.0f)

#define PI  3.1415926535897932384626433832795029

//kalman_t optflow_speed_kalman_x;
//kalman_t optflow_speed_kalman_y;

float ax, bx, cx;
float ay, by, cy;
float vx, vy, pvx, pvy;
float Qx, Rx;
float Qy, Ry;
float scale;
int flag = 1;

void optflow_speed_kalman_init(void)
{
	// kalman filter parameters
	ax = global_data.param[PARAM_KAL_VEL_AX];
	bx = global_data.param[PARAM_KAL_VEL_BX];
	cx = 1.0;

	ay = global_data.param[PARAM_KAL_VEL_AY];
	by = global_data.param[PARAM_KAL_VEL_BY];
	cy = 1.0;

	// assumes initial state is 0
	vx = 0.0;
	vy = 0.0;

	// assumes initial error covariance is 0
	pvx = 0.0;
	pvy = 0.0;

	// noise parameters (hard-coded)
	Qx = 0.005;
	Rx = 0.2;

	Qy = 0.005;
	Ry = 0.2;

	// set the optical flow scale (assuming linear relation)
	//   scale = -0.0714;
	//   scale = 0.04;
	scale = 0.0008 / VEL_KF_TIME_STEP_X;
}

void optflow_speed_kalman(void)
{

	static float viconPre = 0.0;
	float_vect3 debug;
	//    float vx_ = 0.0;


	//Transform accelerometer used in all directions
	float_vect3 acc_nav;
	body2navi(&global_data.accel_si, &global_data.attitude, &acc_nav);
	static float gyro_x_offset = 0, gyro_y_offset = 0;
	float lp = 0.001f;
	gyro_x_offset = (1 - lp) * gyro_x_offset + lp * global_data.gyros_si.x;
	gyro_y_offset = (1 - lp) * gyro_y_offset + lp * global_data.gyros_si.y;

	static float z_position = 0;
	float z_lp = 0.1;
	z_position = (1 - z_lp) * z_position + z_lp * global_data.sonar_distance*cos(global_data.attitude.x)*cos(global_data.attitude.y);

	global_data.position.z = -z_position;

	// transform optical flow into global frame
	float_vect3 flow, flowQuad, flowWorld;//, flowQuadUncorr, flowWorldUncorr;
	flow.x = global_data.optflow.x;
	flow.y = global_data.optflow.y;
	flow.z = 0.0;

	turn_xy_plane(&flow, PI, &flowQuad);
	flowQuad.x = flowQuad.x * scale - (global_data.gyros_si.y - gyro_y_offset);
	flowQuad.y = flowQuad.y * scale + (global_data.gyros_si.x - gyro_x_offset);

	body2navi(&flowQuad, &global_data.attitude, &flowWorld);

	//	turn_xy_plane(&flow, PI, &flowQuadUncorr);
	//	body2navi(&flowQuadUncorr, &global_data.attitude, &flowWorldUncorr);

	//distance from flow sensor to ground
	//float flow_distance = -global_data.vicon_data.z;
	float flow_distance = z_position;

	// initializes x and y to global position
	if (global_data.param[PARAM_VICON_MODE] == 3)
	{
		global_data.position.x = global_data.vicon_data.x;
		global_data.position.y = global_data.vicon_data.y;
	}
	else if (global_data.param[PARAM_VICON_MODE] == 4)
	{
		global_data.position.x = 0;
		global_data.position.y = 0;
		global_data.position.z = 0;
	}

	//  global_data.position.z = global_data.vicon_data.z;

	//---------------------------------------------------
	// Vx Kalman Filter
	// prediction

	float vx_ = ax * vx;
	if (global_data.state.fly == FLY_FLYING)
	{
		vx += bx * global_data.attitude.y;
	}
	float pvx_ = ax * pvx + Qx;

	// do an update only if optical flow is good
	if (global_data.optflow.z > 10.0)
	{
		// kalman gain
		float Kx = pvx_ * ax / (ax * pvx_ * ax + Rx);

		// update step
		//float xflow = global_data.optflow.x*global_data.position.z*scale;
		float xflow = flow_distance * flowWorld.x;
		vx = vx_ + Kx * (xflow - cx * vx_);
		pvx = (1.0 - Kx * cx) * pvx_;
	}
	// otherwise take only the prediction
	else
	{
		vx = vx_;
		pvx = pvx_;
	}

	// assign readings from Kalman Filter
	global_data.velocity.x = vx;
	global_data.position.x += vx * VEL_KF_TIME_STEP_X;

	//---------------------------------------------------
	// Vy Kalman Filter
	// prediction
	float vy_ = ay * vy;
	if (global_data.state.fly == FLY_FLYING)
	{
		vy_ += by * global_data.attitude.x;
	}
	float pvy_ = ay * pvy + Qy;

	// do an update only if optical flow is good
	if (global_data.optflow.z > 10.0)
	{
		// kalman gain
		float Ky = pvy_ * ay / (ay * pvy_ * ay + Ry);

		// update step
		//float yflow = global_data.optflow.y*global_data.position.z*scale;
		float yflow = flow_distance * flowWorld.y;
		vy = vy_ + Ky * (yflow - cy * vy_);
		pvy = (1.0 - Ky * cy) * pvy_;
	}
	// otherwise take only the prediction
	else
	{
		vy = vy_;
		pvy = pvy_;
	}

	// assign readings from Kalman Filter
	global_data.velocity.y = vy;
	global_data.position.y += vy * VEL_KF_TIME_STEP_Y;

	float xvel = (global_data.vicon_data.x - viconPre) / VEL_KF_TIME_STEP_X;
	viconPre = global_data.vicon_data.x;
	debug.x = (global_data.gyros_si.x - gyro_x_offset);
	debug.y = (global_data.gyros_si.y - gyro_y_offset);
	debug.z = z_position;
	debug_vect("KALMAN", debug);
}
