/*
 * optflow_speed_kalman.c
 *
 *  Created on: 23.08.2010
 *      Author: Petri Tanskanen
 *      		Gim Hee Lee
 *      		Laurens Mackay
 */
#include "optflow_speed_kalman.h"
#include "global_data.h"
#include "kalman.h"
#include "altitude_speed.h"

#include "debug.h"
#include "sensors.h"
#include "math.h"
#include "transformation.h"
//#include "outdoor_position_kalman.h"

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


kalman_t outdoor_position_kalman_z;

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



	//Altitude Kalmanfilter
	//initalize matrices
#define TIME_STEP_Z (1.0f / 50.0f)

	static m_elem kal_z_a[4 * 4] =
	{ 1.0f, TIME_STEP_Z, TIME_STEP_Z * TIME_STEP_Z / 2.0f, 0,
	 0, 1.0f, TIME_STEP_Z, 0 ,
	 0, 0, 1.0f, 0 ,
	 0, 0, 0, 1.0f  };

	static m_elem kal_z_c[2*4] =
	{
	 1.0f, 0, 0, 0 ,
	 0, 0, 1.0f, 1.0f };

//	static m_elem kal_z_gain[4 * 2] =
//	{
//	0.0148553889079401, 3.73444963864759e-08,
//	0.00555539506146299, 1.49106022715582e-05,
//	0.000421844252811475, 0.997017766710577,
//	-0.000421844052617397, 9.97097528182815e-08
//	};
	static m_elem kal_z_gain[4 * 2] =
	{
	0.0211952061386090,	0.00432160006966059,
	0.0117007358555860,	0.00543786671309973,
	0.00319895063370895,	0.00357092204456261,
	-9.66582160118677e-07,	1.19075602845418e-06
};

//	static m_elem kal_z_gain_start[4*2] =
//	{
//	 0.060188321659420, 3.566208652525075e-16 ,
//	 0.008855645697701, 1.495920063190432e-13 ,
//	 6.514669086807784e-04, 9.997000796699675e-08 ,
//	 -6.514669086807778e-04, 0.999700079925069  };
	static m_elem kal_z_gain_start[4*2] =
	{
	 1.0f+0*0.060188321659420, 3.566208652525075e-16 ,
	 0*0.008855645697701, 0*1.495920063190432e-13 ,
	 0*6.514669086807784e-04, 0*9.997000796699675e-08 ,
	 0*-6.514669086807778e-04, 0.999700079925069  };

	static m_elem kal_z_x_apriori[4*1] =
	{
	 -430 ,
	 0 ,
	 0 ,
	 -9.81  };

	static m_elem kal_z_x_aposteriori[4*1] =
	{
	 -430 ,
	 0 ,
	 0 ,
	 -9.81 };

	kalman_init(&outdoor_position_kalman_z, 4, 2, kal_z_a, kal_z_c,
			kal_z_gain_start, kal_z_gain, kal_z_x_apriori, kal_z_x_aposteriori,
			100);

}

void optflow_speed_kalman(void)
{

	static float viconPre = 0.0;
	float_vect3 debug;
	//    float vx_ = 0.0;


	//Transform accelerometer used in all directions
	float_vect3 acc_nav;
	body2navi(&global_data.accel_si, &global_data.attitude, &acc_nav);

//	//Calculate gyro offsets. Workaround for old attitude filter.
//	static float gyro_x_offset = 0, gyro_y_offset = 0;
//	float lp = 0.001f;
//	gyro_x_offset = (1 - lp) * gyro_x_offset + lp * global_data.gyros_si.x;
//	gyro_y_offset = (1 - lp) * gyro_y_offset + lp * global_data.gyros_si.y;

	//Low-pass filter for sonar with all spikes. Makes filter following big steps.
	static float sonar_distance_spike = 0;
	float sonar_distance_spike_lp = 0.1; // ~ 1/time to get to new step
	sonar_distance_spike = (1 - sonar_distance_spike_lp) * sonar_distance_spike + sonar_distance_spike_lp * global_data.sonar_distance;

	//Low-pass filter for sonar without spikes
	//only update this low-pass if the signal is close to one of these two low-pass filters.
	static float sonar_distance = 0;
	float z_lp = 0.2; // real low-pass on spike rejected data.
	float spike_reject_threshold = 0.2f; // 0.4 m
	uint8_t sonar_distance_rejecting_spike=0;
	if ((fabs(sonar_distance_spike - global_data.sonar_distance) < spike_reject_threshold) ||
			(fabs(sonar_distance - global_data.sonar_distance) < spike_reject_threshold))
	{
		sonar_distance = (1 - z_lp) * sonar_distance + z_lp
				* global_data.sonar_distance * cos(global_data.attitude.x)
				* cos(global_data.attitude.y);
	}
	else
	{
		sonar_distance_rejecting_spike = 1;
	}

	global_data.sonar_distance_filtered = sonar_distance;


	//pressure altitude

	//Altitude Kalman Filter
	kalman_predict(&outdoor_position_kalman_z);

	m_elem z_measurement[2] =
	{ };
	m_elem z_mask[2] =
	{ 0, 1 };//we normaly only have acceleration an no pressure measurement

	//prepare measurement data
	//measurement #1 pressure => relative altitude
	//sensors_pressure_bmp085_read_out();

	if (global_data.state.pressure_ok)
	{
		z_measurement[0] = -calc_altitude_pressure(global_data.pressure_raw);

		z_mask[0] = 1;//we have a pressure measurement to update

		//debug output
		//						mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 50,
		//								outdoor_position_kalman_z.gainfactor);
	}


	//measurement #2 acceleration
	z_measurement[1] = acc_nav.z;

	//Put measurements into filter
	kalman_correct(&outdoor_position_kalman_z, z_measurement, z_mask);


	//use results
	float sonar_distance_use_treshold = 2.0f;//m
	static float ground_altitude = -0.0f;
	if (sonar_distance >= sonar_distance_use_treshold
			|| sonar_distance_rejecting_spike)
	{
		global_data.position.z
				= kalman_get_state(&outdoor_position_kalman_z, 0)
						- ground_altitude;
	}
	else
	{
		global_data.position.z = -sonar_distance;
		ground_altitude = kalman_get_state(&outdoor_position_kalman_z, 0)
				- (-sonar_distance);
	}
	//use velocity always
	global_data.velocity.z = kalman_get_state(&outdoor_position_kalman_z, 1);







	// transform optical flow into global frame
	float_vect3 flow, flowQuad, flowWorld;//, flowQuadUncorr, flowWorldUncorr;
	flow.x = global_data.optflow.x;
	flow.y = global_data.optflow.y;
	flow.z = 0.0;

	turn_xy_plane(&flow, PI, &flowQuad);
	flowQuad.x = flowQuad.x * scale - global_data.attitude_rate.y;
	flowQuad.y = flowQuad.y * scale + global_data.attitude_rate.x;
//	This was for biased gyro rates
//	flowQuad.x = flowQuad.x * scale - (global_data.gyros_si.y - gyro_y_offset);
//	flowQuad.y = flowQuad.y * scale + (global_data.gyros_si.x - gyro_x_offset);

	body2navi(&flowQuad, &global_data.attitude, &flowWorld);

	//	turn_xy_plane(&flow, PI, &flowQuadUncorr);
	//	body2navi(&flowQuadUncorr, &global_data.attitude, &flowWorldUncorr);

	//distance from flow sensor to ground
	//float flow_distance = -global_data.vicon_data.z;
	float flow_distance = sonar_distance;

	static float px = 0.0;
	static float py = 0.0;
	float QxLocal = 0.1, QyLocal = 0.1;
	float RxLocal = 0.1, RyLocal = 0.1;

	// initializes x and y to global position
	if (global_data.state.position_estimation_mode == POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_VICON)
	{
		global_data.position.x = global_data.vicon_data.x;
		global_data.position.y = global_data.vicon_data.y;
	}
	else if (global_data.state.position_estimation_mode == POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_GLOBAL_VISION && global_data.vision_data_global.new_data == 1)
	{
//		global_data.position.x = global_data.position.x*0.6f + 0.4f*global_data.vision_data_global.pos.x;
//		global_data.position.y = global_data.position.y*0.6f + 0.4f*global_data.vision_data_global.pos.y;

		//simple 1D Kalman filtering (x direction)
		float x_ = global_data.position.x;
		float px_ = px + QxLocal;

		float Kx = 0.4f;//px_/(px_ + RxLocal);

		float x = x_ + Kx*(global_data.vision_data_global.pos.x - x_);
		px = (1.0 - Kx)*px_;

		global_data.position.x = x;

		//simple 1D Kalman filtering (y direction)
		float y_ = global_data.position.y;
		float py_ = py + QyLocal;

		float Ky = 0.4f;//py_/(py_ + RyLocal);

		float y = y_ + Ky*(global_data.vision_data_global.pos.y - y_);
		py = (1.0 - Ky)*py_;

		global_data.position.y = y;

		global_data.vision_data_global.new_data = 0;
	}
	else if (global_data.state.position_estimation_mode == POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_NON_INTEGRATING)
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
		// Let speed decay to zero if no measurements are available
		vx = vx_*0.95;
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
		// Let speed decay to zero if no measurements are available
		vy = vy_*0.95;
		pvy = pvy_;
	}

	// assign readings from Kalman Filter
	global_data.velocity.y = vy;
	global_data.position.y += vy * VEL_KF_TIME_STEP_Y;

//	float xvel = (global_data.vicon_data.x - viconPre) / VEL_KF_TIME_STEP_X;
	viconPre = global_data.vicon_data.x;

	//debug.x = (global_data.gyros_si.x - gyro_x_offset);
	//debug.y = (global_data.attitude_rate.x);
//	debug.x =
//	= sonar_distance;
	debug.x =- calc_altitude_pressure(global_data.pressure_raw);
//	debug.y = sonar_distance_spike;
//	debug.z = global_data.sonar_distance_filtered;
	debug.y=kalman_get_state(&outdoor_position_kalman_z, 0);
	debug.z=ground_altitude;
	debug_vect("altitude", debug);
}
