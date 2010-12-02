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

kalman_t outdoor_position_kalman_x;
kalman_t outdoor_position_kalman_y;
kalman_t outdoor_position_kalman_z;

void outdoor_position_kalman_init(void)
{
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
	kalman_predict(&outdoor_position_kalman_z);

	m_elem z_measurement[2] =
	{ };
	m_elem z_mask[2] =
	{ 0, 1 };//we normaly only have acceleration an no pressure measurement

	//prepare measurement data
	//measurement #1 pressure => relative altitude
	static int nopressure = 0;
	if (nopressure++ == 3)
	{
		nopressure = 0;
		sensors_pressure_bmp085_read_out();

		static float altitude_local_origin = 0;

		if (fabs(calc_altitude_pressure(global_data.pressure_raw)) < 2000)
		{
			if (altitude_local_origin)
			{
				z_measurement[0] = -calc_altitude_pressure(
						global_data.pressure_raw) - altitude_local_origin;
			}
			else
			{
				altitude_local_origin = -calc_altitude_pressure(
						global_data.pressure_raw);
			}

			z_mask[0] = 1;//we have a pressure measurement to update

			//debug output
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 50,
					z_measurement[0]);
		}
	}
	if (nopressure == 2)
	{
		//for temp measurement
		sensors_pressure_bmp085_read_out();
	}

	//measurement #2 acceleration
	float_vect3 acc_nav;
	body2navi(&global_data.accel_si, &global_data.attitude, &acc_nav);
	z_measurement[1] = acc_nav.z;

	//Put measurements into filter
	kalman_correct(&outdoor_position_kalman_z, z_measurement, z_mask);

	//	static int i=2;
	//	if(i++==4){
	//		i=0;
	//debug
	float_vect3 out_kal_z;
	out_kal_z.x = kalman_get_state(&outdoor_position_kalman_z,0);
	out_kal_z.y = kalman_get_state(&outdoor_position_kalman_z,1);
	out_kal_z.z = kalman_get_state(&outdoor_position_kalman_z,2);
	debug_vect("out_kal_z", out_kal_z);
	//}
}
