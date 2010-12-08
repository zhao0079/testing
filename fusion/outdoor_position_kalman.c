/*
 * outdoor_position_kalman.c
 *
 *  Created on: 02.12.2010
 *      Author: Laurens Mackay
 */
#include "outdoor_position_kalman.h"
#include "kalman.h"

#include "debug.h"
#include "sensors.h"
#include "math.h"
#include "altitude_speed.h"
#include "transformation.h"
#include "gps_transformations.h"

kalman_t outdoor_position_kalman_x;
kalman_t outdoor_position_kalman_y;
kalman_t outdoor_position_kalman_z;

static float altitude_local_origin = 0;

void outdoor_position_kalman_init(void)
{
	//X Kalmanfilter
	//initalize matrices
#define TIME_STEP_X (1.0f / 200.0f)

	static m_elem kal_x_a[4 * 4] =
	{ 1.0f, TIME_STEP_X, TIME_STEP_X * TIME_STEP_X / 2.0f, 0,
	 0, 1.0f, TIME_STEP_X, 0 ,
	 0, 0, 0.9f, 0 ,
	 0, 0, 0, 1.0f  };

	static m_elem kal_x_c[2*4] =
	{
	 1.0f, 0, 0, 0 ,
	 0, 0, 1.0f, 1.0f };
//
//	static m_elem kal_x_gain[4 * 2] =
//	{
//	0.0148553889079401, 3.73444963864759e-08,
//	0.00555539506146299, 1.49106022715582e-05,
//	0.000421844252811475, 0.997017766710577,
//	-0.000421844052617397, 9.97097528182815e-08 };

//	static m_elem kal_x_gain[4 * 2] =
//	{
//	0.374797938488850,	3.73850295872383e-08,
//	0.0853099645050014,	1.49106314199340e-05,
//	0.00111821605423848,	0.997017766714797,
//	-0.00111821464754384,	9.97055320882196e-08};

//	static m_elem kal_x_gain[4 * 2] =
//	{
//	0.652373111233803,	3.69547422415130e-08,
//	0.335139052708543,	1.48803888106151e-05,
//	0.0833402202608038,	0.996024802404946,
//	-0.0833402194802173,	0.000996025175585497};

//	static m_elem kal_x_gain[4 * 2] =
//	{
//	0.592750199387368,	4.43580350345266e-06,
//	0.261234180593370,	0.00113328056903801,
//	0.0520844935682290,	0.772332121775609,
//	-0.0520796109459413,	0.000772365439149219};

	static m_elem kal_x_gain[4 * 2] =
	{
	0.770934704461122,	4.40141801112333e-06,
	0.536732919373398,	0.00113318742260124,
	0.151314528891492,	0.772332026761219,
	-0.151273373292308,	0.0772460449859003};//0.000772460449859003




//	static m_elem kal_x_gain_start[4*2] =
//	{
//	 0.060188321659420, 3.566208652525075e-16 ,
//	 0.008855645697701, 1.495920063190432e-13 ,
//	 6.514669086807784e-04, 9.997000796699675e-08 ,
//	 -6.514669086807778e-04, 0.999700079925069  };
	static m_elem kal_x_gain_start[4*2] =
	{
	0.0291725594968339,	4.20818895401493e-16,
	0.00426373951034103,	1.50288378749457e-13,
	0.000311581023720996,	9.99700080297815e-08,
	-0.000311581023720996,	0.999700079925069};


	static m_elem kal_x_x_apriori[4*1] =
	{
	 0 ,
	 0 ,
	 0 ,
	 0  };

	static m_elem kal_x_x_aposteriori[4*1] =
	{
	 0 ,
	 0 ,
	 0 ,
	 0 };

	kalman_init(&outdoor_position_kalman_x, 4, 2, kal_x_a, kal_x_c,
			kal_x_gain_start, kal_x_gain, kal_x_x_apriori, kal_x_x_aposteriori,
			1000);




	//Y Kalmanfilter
	//initalize matrices
#define TIME_STEP_Y (1.0f / 200.0f)

	static m_elem kal_y_a[4 * 4] =
	{ 1.0f, TIME_STEP_Y, TIME_STEP_Y * TIME_STEP_Y / 2.0f, 0,
	 0, 1.0f, TIME_STEP_Y, 0 ,
	 0, 0, 0.9f, 0 ,
	 0, 0, 0, 1.0f  };

	static m_elem kal_y_c[2*4] =
	{
	 1.0f, 0, 0, 0 ,
	 0, 0, 1.0f, 1.0f };
//
//	static m_elem kal_y_gain[4 * 2] =
//	{
//	0.0148553889079401, 3.73444963864759e-08,
//	0.00555539506146299, 1.49106022715582e-05,
//	0.000421844252811475, 0.997017766710577,
//	-0.000421844052617397, 9.97097528182815e-08 };

//	static m_elem kal_y_gain[4 * 2] =
//	{
//	0.374797938488850,	3.73850295872383e-08,
//	0.0853099645050014,	1.49106314199340e-05,
//	0.00111821605423848,	0.997017766714797,
//	-0.00111821464754384,	9.97055320882196e-08};

//	static m_elem kal_y_gain[4 * 2] =
//	{
//	0.652373111233803,	3.69547422415130e-08,
//	0.335139052708543,	1.48803888106151e-05,
//	0.0833402202608038,	0.996024802404946,
//	-0.0833402194802173,	0.000996025175585497};

//	static m_elem kal_y_gain[4 * 2] =
//	{
//	0.592750199387368,	4.43580350345266e-06,
//	0.261234180593370,	0.00113328056903801,
//	0.0520844935682290,	0.772332121775609,
//	-0.0520796109459413,	0.000772365439149219};

	static m_elem kal_y_gain[4 * 2] =
	{
	0.770934704461122,	4.40141801112333e-06,
	0.536732919373398,	0.00113318742260124,
	0.151314528891492,	0.772332026761219,
	-0.151273373292308,	0.0772460449859003};//0.000772460449859003




//	static m_elem kal_y_gain_start[4*2] =
//	{
//	 0.060188321659420, 3.566208652525075e-16 ,
//	 0.008855645697701, 1.495920063190432e-13 ,
//	 6.514669086807784e-04, 9.997000796699675e-08 ,
//	 -6.514669086807778e-04, 0.999700079925069  };
	static m_elem kal_y_gain_start[4*2] =
	{
	0.0291725594968339,	4.20818895401493e-16,
	0.00426373951034103,	1.50288378749457e-13,
	0.000311581023720996,	9.99700080297815e-08,
	-0.000311581023720996,	0.999700079925069};


	static m_elem kal_y_x_apriori[4*1] =
	{
	 0 ,
	 0 ,
	 0 ,
	 0  };

	static m_elem kal_y_x_aposteriori[4*1] =
	{
	 0 ,
	 0 ,
	 0 ,
	 0 };

	kalman_init(&outdoor_position_kalman_y, 4, 2, kal_y_a, kal_y_c,
			kal_y_gain_start, kal_y_gain, kal_y_x_apriori, kal_y_x_aposteriori,
			1000);




	//Altitude Kalmanfilter
	//initalize matrices
#define TIME_STEP_Z (1.0f / 200.0f)

	static m_elem kal_z_a[4 * 4] =
	{ 1.0f, TIME_STEP_Z, TIME_STEP_Z * TIME_STEP_Z / 2.0f, 0,
	 0, 1.0f, TIME_STEP_Z, 0 ,
	 0, 0, 1.0f, 0 ,
	 0, 0, 0, 1.0f  };

	static m_elem kal_z_c[2*4] =
	{
	 1.0f, 0, 0, 0 ,
	 0, 0, 1.0f, 1.0f };

	static m_elem kal_z_gain[4 * 2] =
	{
	0.0148553889079401, 3.73444963864759e-08,
	0.00555539506146299, 1.49106022715582e-05,
	0.000421844252811475, 0.997017766710577,
	-0.000421844052617397, 9.97097528182815e-08 };

	static m_elem kal_z_gain_start[4*2] =
	{
	 0.060188321659420, 3.566208652525075e-16 ,
	 0.008855645697701, 1.495920063190432e-13 ,
	 6.514669086807784e-04, 9.997000796699675e-08 ,
	 -6.514669086807778e-04, 0.999700079925069  };

	static m_elem kal_z_x_apriori[4*1] =
	{
	 0 ,
	 0 ,
	 0 ,
	 -9.81  };

	static m_elem kal_z_x_aposteriori[4*1] =
	{
	 0 ,
	 0 ,
	 0 ,
	 -9.81 };

	kalman_init(&outdoor_position_kalman_z, 4, 2, kal_z_a, kal_z_c,
			kal_z_gain_start, kal_z_gain, kal_z_x_apriori, kal_z_x_aposteriori,
			1000);
}

void outdoor_position_kalman(void)
{
	//Transform accelerometer used in all directions
	float_vect3 acc_nav;
	body2navi(&global_data.accel_si, &global_data.attitude, &acc_nav);

	//X &Y Kalman Filter
	kalman_predict(&outdoor_position_kalman_x);
	kalman_predict(&outdoor_position_kalman_y);

	m_elem x_measurement[2] =
	{ };
	m_elem x_mask[2] =
	{ 0, 1 };//only acceleromenters normaly
	m_elem y_measurement[2] =
	{ };
	m_elem y_mask[2] =
	{ 0, 1 };//only acceleromenters normaly

//	static int i = 0;
//	if (i++ == 200)
//	{
//		i = 0;
//		x_measurement[0]=0;
//		x_mask[0]=1;//simulate GPS position 0  at 1 Hz
	//	}	static int i = 0;
	if (global_data.state.gps_ok && global_data.state.gps_new_data)
	{
		global_data.state.gps_new_data=0;
		float_vect3 gps_local;
		gps_get_local_position(&gps_local);
		x_measurement[0] = gps_local.x;
		x_mask[0] = 1;// GPS position at 1 Hz
		y_measurement[0] = gps_local.y;
		y_mask[0] = 1;// GPS position at 1 Hz
	}


	x_measurement[1] = acc_nav.x;
	y_measurement[1] = acc_nav.y;

	//Put measurements into filter
	kalman_correct(&outdoor_position_kalman_x, x_measurement, x_mask);
	kalman_correct(&outdoor_position_kalman_y, y_measurement, y_mask);

	//	static int i=2;
	//	if(i++==4){
	//		i=0;
	//debug

//	mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 50,
//			x_measurement[1]);
//	float_vect3 out_kal_x;
//	out_kal_x.x = kalman_get_state(&outdoor_position_kalman_x,0);
//	out_kal_x.y = kalman_get_state(&outdoor_position_kalman_x,1);
//	out_kal_x.z = kalman_get_state(&outdoor_position_kalman_x,3);
//	debug_vect("out_kal_x", out_kal_x);









	//Altitude Kalman Filter
	kalman_predict(&outdoor_position_kalman_z);

	m_elem z_measurement[2] =
	{ };
	m_elem z_mask[2] =
	{ 0, 1 };//we normaly only have acceleration an no pressure measurement

	//prepare measurement data
	//measurement #1 pressure => relative altitude
	static int nopressure = 1;

	nopressure++;
	if (nopressure == 2 || nopressure == 4)
	{
		sensors_pressure_bmp085_read_out();
	}
	if (nopressure == 4)
	{
		nopressure = 0;

		if (global_data.state.pressure_ok)
		{
			if (altitude_local_origin)
			{
				z_measurement[0] = -calc_altitude_pressure(
						global_data.pressure_raw) - altitude_local_origin;
			}
			else
			{
				altitude_set_local_origin();
			}

			z_mask[0] = 1;//we have a pressure measurement to update

			//debug output
//						mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 50,
//								outdoor_position_kalman_z.gainfactor);
		}
	}

	//measurement #2 acceleration
	z_measurement[1] = acc_nav.z;

	//Put measurements into filter
	kalman_correct(&outdoor_position_kalman_z, z_measurement, z_mask);

// save outputs
	global_data.position.x = kalman_get_state(&outdoor_position_kalman_x,0);
	global_data.position.y = kalman_get_state(&outdoor_position_kalman_y,0);
	global_data.position.z = kalman_get_state(&outdoor_position_kalman_z,0);

	global_data.velocity.x = kalman_get_state(&outdoor_position_kalman_x,1);
	global_data.velocity.y = kalman_get_state(&outdoor_position_kalman_y,1);
	global_data.velocity.z = kalman_get_state(&outdoor_position_kalman_z,1);



}
void altitude_set_local_origin(void){
	altitude_local_origin = -calc_altitude_pressure(
			global_data.pressure_raw);
}
