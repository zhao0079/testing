/*
 * vision_position_kalman.c
 *
 *  Created on: 12.03.2010
 *      Author: Laurens Mackay
 */
#include "vicon_position_kalman.h"
#include "kalman.h"

#include "debug.h"
#include "sensors.h"
#include "math.h"
#include "altitude_speed.h"
#include "transformation.h"
#include "gps_transformations.h"

//#define ONLY_Z

//#define VELOCITY_HOLD 0.999f
//#define ACCELERATION_HOLD 0.99f
#define VELOCITY_HOLD 0.99f
#define ACCELERATION_HOLD 1.0f

kalman_t vicon_position_kalman_x;
kalman_t vicon_position_kalman_y;
kalman_t vicon_position_kalman_z;

void vicon_position_kalman_init(void)
{
#ifndef ONLY_Z
	//X Kalmanfilter
	//initalize matrices
#define TIME_STEP_X (1.0f / 200.0f)

	static m_elem kal_x_a[2 * 2] =
	{ 1.0f, TIME_STEP_X,
	  0.0f, VELOCITY_HOLD
	  };

	static m_elem kal_x_c[2 * 2] =
	{ 1.0f, 0.0f,
	  1.0f, 0.0f };

	static m_elem kal_x_gain[2 * 2]  =
	{
			0.550311626986869,	0.809201491990525,
			4.24117140900075,	5.04378836470466
	};

	static m_elem kal_x_gain_start[2 * 2] =
	{
			0.177673118212026,	0.363726574226735,
			0,	0
	};

	static m_elem kal_x_x_apriori[2 * 1] =
	{
	 0 ,
	 0 };

	static m_elem kal_x_x_aposteriori[2 * 1] =
	{
	 0 ,
	 0 };

	kalman_init(&vicon_position_kalman_x, 2, 2, kal_x_a, kal_x_c,
			kal_x_gain_start, kal_x_gain, kal_x_x_apriori, kal_x_x_aposteriori,
			1000);




	//Y Kalmanfilter
	//initalize matrices
#define TIME_STEP_Y (1.0f / 200.0f)

	static m_elem kal_y_a[2 * 2] =
	{ 1.0f, TIME_STEP_Y,
	  0.0f, VELOCITY_HOLD
	  };

	static m_elem kal_y_c[2 * 2] =
	{ 1.0f, 0.0f,
	  1.0f, 0.0f };

	static m_elem kal_y_gain[2 * 2]  =
	{
			0.550311626986869,	0.809201491990525,
			4.24117140900075,	5.04378836470466
	};

	static m_elem kal_y_gain_start[2 * 2] =
	{
			0.177673118212026,	0.363726574226735,
			0,	0
	};
	static m_elem kal_y_x_apriori[2 * 1] =
	{
	 0 ,
	 0 };

	static m_elem kal_y_x_aposteriori[2 * 1] =
	{
	 0 ,
	 0 };

	kalman_init(&vicon_position_kalman_y, 2, 2, kal_y_a, kal_y_c,
			kal_y_gain_start, kal_y_gain, kal_y_x_apriori, kal_y_x_aposteriori,
			1000);



#endif
	//Altitude Kalmanfilter
	//initalize matrices
#define TIME_STEP_Z (1.0f / 200.0f)

	static m_elem kal_z_a[2 * 2] =
	{ 1.0f, TIME_STEP_Z,
	  0.0f, VELOCITY_HOLD
	  };

	static m_elem kal_z_c[2 * 2] =
	{ 1.0f, 0.0f,
	  1.0f, 0.0f };

//	static m_elem kal_z_gain[2 * 2]  =
//	{
//			0.120089421679731,	4.35722593937886e-06,
//			0.213207108111745,	0.00113300623965082,
//			0.177947308239338,	0.772331751820938,
//			-0.177890740554481,	0.000772735355118873};
	static m_elem kal_z_gain[2 * 2] =
	{
			0.550311626986869,	0.809201491990525,
			4.24117140900075,	5.04378836470466
	};

//	{
//			0.177673118212026,	0.363726574226735,
//			1.13547678406557,	1.65089930506536
//	};

	static m_elem kal_z_gain_start[2 * 2] =
	{
			0.177673118212026,	0.363726574226735,
			0,	0
	};

	static m_elem kal_z_x_apriori[2 * 1] =
	{
	 0 ,
	 0 };

	static m_elem kal_z_x_aposteriori[2 * 1] =
	{
	 0 ,
	 0 };

	kalman_init(&vicon_position_kalman_z, 2, 2, kal_z_a, kal_z_c,
			kal_z_gain_start, kal_z_gain, kal_z_x_apriori, kal_z_x_aposteriori,
			1000);
}

void vicon_position_kalman(void)
{
	//Transform accelerometer used in all directions
	float_vect3 acc_nav;
	body2navi(&global_data.accel_si, &global_data.attitude, &acc_nav);

#ifndef ONLY_Z
	//X &Y Kalman Filter
	kalman_predict(&vicon_position_kalman_x);
	kalman_predict(&vicon_position_kalman_y);
#endif
	kalman_predict(&vicon_position_kalman_z);

	m_elem x_measurement[2] =
	{ };
	m_elem x_mask[2] =
	{ 0, 0 };//only acceleromenters normaly
	m_elem y_measurement[2] =
	{ };
	m_elem y_mask[2] =
	{ 0, 0 };//only acceleromenters normaly
	m_elem z_measurement[2] =
	{ };
	m_elem z_mask[2] =
	{ 0, 0 };//only acceleromenters normaly

	// Vicon fallback - if vision fails the filter will start using vicon position estimates instead
	float vision_taken = 0.f;
	if (global_data.vision_data.new_data || global_data.state.vicon_new_data)
	{
		if (global_data.state.position_estimation_mode == POSITION_ESTIMATION_MODE_VISION_VICON_BACKUP)
		{
			//measure difference:
			float difference = sqrtf((global_data.vision_data.pos.x	- global_data.vicon_data.x) * (global_data.vision_data.pos.x - global_data.vicon_data.x)
								   + (global_data.vision_data.pos.y	- global_data.vicon_data.y) * (global_data.vision_data.pos.y - global_data.vicon_data.y)
								   + (global_data.vision_data.pos.z	- global_data.vicon_data.z) * (global_data.vision_data.pos.z - global_data.vicon_data.z));

			//use only vision_data if difference to vicon is small or we don't have vicon_data at all
			if (global_data.state.vision_ok && (difference < global_data.param[PARAM_VICON_TAKEOVER_DISTANCE] || !global_data.state.vicon_ok))
			{
				if (global_data.vision_data.new_data)
				{
					//vision
					vision_taken = 1.f;
					x_measurement[1] = global_data.vision_data.pos.x;
					x_mask[1] = 1;
					y_measurement[1] = global_data.vision_data.pos.y;
					y_mask[1] = 1;
					z_measurement[1] = global_data.vision_data.pos.z;
					z_mask[1] = 1;
				}
			}
			else if (global_data.state.vicon_new_data)
			{ //vicon
				x_measurement[0] = global_data.vicon_data.x;
				x_mask[0] = 1;
				y_measurement[0] = global_data.vicon_data.y;
				y_mask[0] = 1;
				z_measurement[0] = global_data.vicon_data.z;
				z_mask[0] = 1;
			}
		}
		else if (global_data.state.position_estimation_mode == POSITION_ESTIMATION_MODE_VICON_ONLY)
		{
			if (global_data.state.vicon_new_data)
			{ //vicon
				x_measurement[0] = global_data.vicon_data.x;
				x_mask[0] = 1;
				y_measurement[0] = global_data.vicon_data.y;
				y_mask[0] = 1;
				z_measurement[0] = global_data.vicon_data.z;
				z_mask[0] = 1;
			}
		}

		// send trigger to filter output delay and status if vision data is used
		if (global_data.vision_data.new_data)
		{
			float vision_delay = (global_data.vision_data.comp_end - global_data.vision_data.time_captured)/1000.f;
			//debug_vect("IMU", vision_delay);
			mavlink_msg_debug_vect_send(global_data.param[PARAM_SEND_DEBUGCHAN], "IMU", global_data.vision_data.time_captured, vision_delay, vision_taken, 0.f);
		}

		//data has been used
		global_data.vision_data.new_data = 0;
		global_data.state.vicon_new_data = 0;
	}

	//Put measurements into filter

#ifndef ONLY_Z
	kalman_correct(&vicon_position_kalman_x, x_measurement, x_mask);
	kalman_correct(&vicon_position_kalman_y, y_measurement, y_mask);
#endif
	kalman_correct(&vicon_position_kalman_z, z_measurement, z_mask);

#ifndef ONLY_Z
	global_data.position.x = kalman_get_state(&vicon_position_kalman_x,0);
	global_data.velocity.x = kalman_get_state(&vicon_position_kalman_x,1);
	global_data.position.y = kalman_get_state(&vicon_position_kalman_y,0);
	global_data.velocity.y = kalman_get_state(&vicon_position_kalman_y,1);
#endif
	global_data.position.z = kalman_get_state(&vicon_position_kalman_z,0);
	global_data.velocity.z = kalman_get_state(&vicon_position_kalman_z,1);
}
