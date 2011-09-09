/*
 * mainloop.c
 *
 *  Created on: 08.10.2010
 *      Author: mackayl
 */

#include "conf.h"

#if PX_VEHICLE_TYPE == PX_AIRFRAME_QUADROTOR

#include "mainloop_quadrotor.h"
#include "common_mainloop_functions.h"
#include "mainloop_generic.h"

#include "inttypes.h"
#include "mcu_init.h"

// Include comm
#include "comm.h"
#include "mavlink.h"

// Include globals
#include "global_data.h"
#include "sensors.h"
#include "calibration.h"

#include "adc.h"
#include "led.h"
#include "ppm.h"
#include "pwm.h"
#include "sys_time.h"
#include "uart.h"
#include "dac.h"

#include "watchdog.h"
#include "control.h"

#include "compass.h"
#include "acceleration.h"
#include "gyros.h"
#include "motors.h"
#include "servos.h"
#include "radio_control.h"
#include "battery.h"
#include "shutter.h"
#include "infrared_distance.h"
#include "sonar_distance.h"
#include "gps.h"
#include "gps_transformations.h"

#include "optical_flow.h"

#include "altitude_speed.h"
#include "global_pos.h"
#include "communication.h"

#include "simple_altitude_moving_average.h"
#include "attitude_compl_euler.h"
#include "altitude_kalman.h"
#include "lookup_sin_cos.h"
#include "least_square.h"

#include "control_quadrotor_attitude.h"
#include "control_fixed_wing_attitude.h"
#include "control_quadrotor_position.h"
#include "control_quadrotor_start_land.h"
#include "remote_control.h"
#include "position_kalman3.h"
#include "vision_buffer.h"

#include "debug.h"
#include "transformation.h"
#include "eeprom.h"
#include "params.h"
#include "attitude_observer.h"
#include "matrix.h"
#include "float_checks.h"

#include "mmc_spi.h"
#include "dos.h"
#include "fat.h"

#include "attitude_tobi_laurens.h"
#include "outdoor_position_kalman.h"
#include "vision_position_kalman.h"
#include "vicon_position_kalman.h"
#include "optflow_speed_kalman.h"

// Executiontime debugging
float_vect3 time_debug;
static uint32_t count = 0;

// Static variables
// these variables are used during the whole
// code runtime in the mainloop
static const uint32_t min_mainloop_time = 5000; ///< The minimum wait interval between two mainloop software timer calls, = 1/max rate, initialized to 1 sec = 1000000 microseconds
//static uint32_t loop_max_time = 0;               ///< The maximum time in microseconds one mainloop took
static uint64_t last_mainloop_idle = 0; ///< Starvation Prevention


void main_loop_quadrotor(void)
{
	/**
	 * @brief Initialize the whole system
	 *
	 * All functions that need to be called before the first mainloop iteration
	 * should be placed here.
	 */
	main_init_generic();
	control_quadrotor_position_init();
	control_quadrotor_attitude_init();
	attitude_tobi_laurens_init();

	// FIXME XXX Make proper mode switching

//	outdoor_position_kalman_init();
	//vision_position_kalman_init();
	//vicon_position_kalman_init();
	optflow_speed_kalman_init();

	/**
	 * @brief This is the main loop
	 *
	 * It will be executed at maximum MCU speed (60 Mhz)
	 */
	// Executiontime debugging
	time_debug.x = 0;
	time_debug.y = 0;
	time_debug.z = 0;

	last_mainloop_idle = sys_time_clock_get_time_usec();
	debug_message_buffer("Starting main loop");

	led_off(LED_GREEN);
	led_off(LED_RED);
	while (1)
	{
		// Time Measurement
		uint64_t loop_start_time = sys_time_clock_get_time_usec();

		///////////////////////////////////////////////////////////////////////////
		/// CRITICAL 200 Hz functions
		///////////////////////////////////////////////////////////////////////////
		if (us_run_every(5000, COUNTER2, loop_start_time))
		{
			// Kalman Attitude filter, used on all systems
			gyro_read();
			sensors_read_acc();

			// Read out magnetometer at its default 50 Hz rate
			static uint8_t mag_count = 0;

			if (mag_count == 3)
			{
				sensors_read_mag();
				//attitude_observer_correct_magnet(global_data.magnet_corrected);
				mag_count = 0;
			}else if(mag_count==1){

				hmc5843_start_read();
				mag_count++;
			}
			else
			{
				mag_count++;
			}

			// Correction step of observer filter
			//attitude_observer_correct_accel(global_data.accel_raw);

			// Write in roll and pitch
			//static float_vect3 att; //if not static we have spikes in roll and pitch on bravo !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			//attitude_observer_get_angles(&att);
//			global_data.attitude.x = att.x;
//			global_data.attitude.y = att.y;
//			if (global_data.param[PARAM_ATT_KAL_IYAW])
//			{
//				global_data.attitude.z += 0.005 * global_data.gyros_si.z;
//			}
//			else
//			{
//				global_data.attitude.z = att.z;
//			}
			// Prediction step of observer
			//attitude_observer_predict(global_data.gyros_si);
			attitude_tobi_laurens();
			//sequential_kalmanfilter();

			//position_integrate(&global_data.attitude,&global_data.position,&global_data.velocity,&global_data.accel_si);

			// FIXME XXX Change to position estimation mode

			if (global_data.state.gps_mode >= 1)
			{
//				position_kalman_TL();
//				outdoor_position_kalman();
			}
			else
			{
//				vicon_position_kalman();
//				vision_position_kalman();
//				fuse_vision_altitude_200hz();
			}

			control_quadrotor_attitude();

			//debug counting number of executions
			count++;
		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// Camera Shutter - This takes 50 usecs!!!
		///////////////////////////////////////////////////////////////////////////
		// Set camera shutter with 2.5ms resolution
		else if (us_run_every(5000, COUNTER1, loop_start_time)) //was 2500 !!!
		{
			camera_shutter_handling(loop_start_time);

			// Measure time for debugging
			time_debug.x = max(time_debug.x, sys_time_clock_get_time_usec()
					- loop_start_time);

		}

		///////////////////////////////////////////////////////////////////////////
		/// CRITICAL FAST 50 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(20000, COUNTER3, loop_start_time))
		{
			// Read Analog-to-Digital converter
			adc_read();

			// Control the quadrotor position
			control_quadrotor_position();
			// Read remote control
			remote_control();

			control_camera_angle();

			//float_vect3 opt;
			static float_vect3 opt_int;
			uint8_t valid = optical_flow_get_dxy(80, &global_data.optflow.x, &global_data.optflow.y, &global_data.optflow.z);
			if (valid)
			{
				opt_int.x += global_data.optflow.x;
				opt_int.y += global_data.optflow.y;

			}
			global_data.sonar_distance = sonar_distance_get(ADC_5_CHANNEL);
			opt_int.z = valid;
			mavlink_msg_optical_flow_send(global_data.param[PARAM_SEND_DEBUGCHAN], loop_start_time + sys_time_clock_get_unix_offset(), 0, global_data.optflow.x, global_data.optflow.y, global_data.optflow.z, global_data.sonar_distance_filtered);
			//optical_flow_debug_vect_send();
			//debug_vect("opt_int", opt_int);
			optical_flow_start_read(80);



			optflow_speed_kalman();

			// Send the raw sensor/ADC values
			communication_send_raw_data(loop_start_time);
		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// CRITICAL FAST 20 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(50000, COUNTER4, loop_start_time))
		{
			//*** this happens in handle_controller_timeouts already!!!!! ***
			//			//update global_data.state
			//			if (global_data.param[PARAM_VICON_MODE] == 1)
			//			{
			//				//VICON_MODE 1 only accepts vicon position
			//				global_data.state.position_fix = global_data.state.vicon_ok;
			//			}
			//			else
			//			{
			//				//VICON_MODEs 0, 2, 3 accepts vision additionally, so check vision
			//				global_data.state.position_fix = global_data.state.vision_ok;
			//			}

			update_system_statemachine(loop_start_time);
			update_controller_setpoints();

			mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(
					global_data.param[PARAM_SEND_DEBUGCHAN],
					loop_start_time + sys_time_clock_get_unix_offset(),
					global_data.attitude_setpoint.x,
					global_data.attitude_setpoint.y,
					global_data.position_yaw_control_output,
					global_data.thrust_control_output);

			//STARTING AND LANDING
			quadrotor_start_land_handler(loop_start_time);
		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 100 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(5000, COUNTER6, loop_start_time))
		{

			if (global_data.param[PARAM_SEND_SLOT_DEBUG_6])
			{
				debug_vect("att_ctrl_o", global_data.attitude_control_output);
			}
		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// UNCRITICAL SLOW 5 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(200000, COUNTER8, loop_start_time))
		{
			// The onboard controllers go into failsafe mode once
			// position data is missing
			handle_controller_timeouts(loop_start_time);
			// Send buffered data such as debug text messages
			// Empty one message out of the buffer
			debug_message_send_one();

			// Toggle status led
			//led_toggle(LED_YELLOW);
			led_toggle(LED_RED); // just for green LED on alpha at the moment

			// Toggle active mode led
			if (global_data.state.mav_mode == MAV_MODE_MANUAL
					|| global_data.state.mav_mode == MAV_MODE_GUIDED
					|| global_data.state.mav_mode == MAV_MODE_AUTO
					|| global_data.state.mav_mode == MAV_MODE_TEST2)
			{
				led_on(LED_GREEN);
			}
			else
			{
				led_off(LED_GREEN);
			}

			handle_eeprom_write_request();
			handle_reset_request();

			update_controller_parameters();

			communication_send_controller_feedback();

			communication_send_remote_control();

			// Pressure sensor driver works, but not tested regarding stability
			//			sensors_pressure_bmp085_read_out();

			//
			//			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 50, calc_altitude_pressure(global_data.pressure_raw));
			//			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 51, global_data.pressure_raw);

			if (global_data.param[PARAM_POSITION_YAW_TRACKING] == 1)
			{
				mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN],
						90, global_data.param[PARAM_POSITION_SETPOINT_YAW]);
				mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN],
						91, global_data.yaw_pos_setpoint);
			}
			//			//testing gps
			//			//uint8_t gps_send_buf[]={0x01,0x01,0x00,0x00};
			//			uint8_t gps_send_buf[]={0x01,0x04,0x00,0x00};
			//
			//			uint8_t n=2;
			//			uint8_t CK_A = 0, CK_B = 0;
			//			for(uint8_t i=0;i<n;i++)
			//			{
			//			CK_A = CK_A + gps_send_buf[i];
			//			CK_B = CK_B + CK_A;
			//			}
			//
			//			//pol
			//			//ublox start
			//			uart1_transmit(0xb5);
			//			uart1_transmit(0x62);
			//			for(uint8_t i=0;i<n;i++){
			//				uart1_transmit(gps_send_buf[i]);
			//			}
			//			uart1_transmit(CK_A);
			//			uart1_transmit(CK_B);
			//
			//
			//			uint8_t gps_nmea_send_buf[] = "$PUBX,04*37\n";
			//
			//			uint8_t n_nmea = 12;
			//			for (uint8_t i = 0; i < n_nmea; i++)
			//			{
			//				uart1_transmit(gps_nmea_send_buf[i]);
			//			}
		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 1 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(1000000, COUNTER9, loop_start_time))
		{
			// Send system state, mode, battery voltage, etc.
			send_system_state();

			// FIXME XXX REMOVE
			// Send position setpoint offset
			debug_vect("pos offs", global_data.position_setpoint_offset);

			// Send current onboard time
			mavlink_msg_system_time_send(MAVLINK_COMM_1,
					sys_time_clock_get_unix_time());

			//update state from received parameters
			sync_state_parameters();

			//debug number of execution
			/*			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN],
			 101, count);*/
			count = 0;

			//			TESTING MATRIX MULTIPLICATION
			//			m_elem testA[4*4]={123478,2,12343,21345,
			//			1123,2,12343,6,
			//			132,234,123,3,
			//			1231234,76697,23,23};
			//			matrix_t tA = matrix_create(4,4,testA);
			//			m_elem test2[4*4]={
			//			41529137736.0000,1640232687.00000,1526122764.00000,2636165886.00000
			//			,147684720,3350694,15404202,24007614
			//			,20271816,259605,4532736,2819382
			//			,152144764001.000,4385275,16143795691.0000,26281150510.0000};
			//
			//			matrix_t t2= matrix_create(4,4,test2);
			//
			//			m_elem testr[4*4]={};
			//			matrix_t tr= matrix_create(4,4,testr);
			//			matrix_mult(tA,tA,tr);
			//			matrix_sub(t2,tr,tA);
			//			int error=0;
			//			for(int i=0;i<16;i++){
			//				//if(testA[i]){
			//					debug_message_buffer_sprintf("result %i",(int) (testA[i]*1000));
			//				//}
			//			}

			//Send execution times for debugging
			// Executiontime debugging
//			time_debug.x = 0;
//			time_debug.y = 0;
//			time_debug.z = 0;

			//enable gps push thru:
			//global_data.param[PARAM_GPS_MODE]=20;

			if (global_data.param[PARAM_GPS_MODE] >= 10)
			{
				//Send GPS information
				float_vect3 gps;
				gps.x = gps_utm_north / 100.0f;//m
				gps.y = gps_utm_east / 100.0f;//m
				gps.z = gps_utm_zone;// gps_week;
				debug_vect("GPS", gps);

			}
			else if (global_data.param[PARAM_GPS_MODE] == 9
					|| global_data.param[PARAM_GPS_MODE] == 8)
			{
				//				static float_vect3 gps_local, gps_local_home;
				//				static bool gps_local_home_init = false;
				//				static float gps_cos_home_lat;

				if (global_data.param[PARAM_GPS_MODE] == 8)
				{
					gps_set_local_origin();
					//					gps_local_home_init = false;
				}
				if (gps_lat == 0)
				{
					debug_message_buffer("GPS Signal Lost");
				}
				else
				{
					float_vect3 gps_local, gps_local_velocity;
					gps_get_local_position(&gps_local);
					debug_vect("GPS local", gps_local);
					gps_get_local_velocity(&gps_local_velocity);
					debug_vect("GPS loc velocity", gps_local_velocity);
					//					const float r_earth = 6378140;
					//					if (!gps_local_home_init)
					//					{
					//						gps_local_home.x = gps_lat / 1e7f;
					//						gps_local_home.y = gps_lon / 1e7f;
					//						gps_local_home.z = gps_alt / 100e0f;
					//						gps_cos_home_lat = cos(gps_local_home.x * 3.1415 / 180);
					//						gps_local_home_init = true;
					//						debug_message_buffer("GPS Local Origin saved");
					//					}
					//
					//					gps_local.x = r_earth * tan((gps_lat / 1e7f
					//							- gps_local_home.x) * 3.1415 / 180);
					//					gps_local.y = r_earth * gps_cos_home_lat * tan((gps_lon
					//							/ 1e7f - gps_local_home.y) * 3.1415 / 180);
					//					gps_local.z = gps_alt / 100e0f - gps_local_home.z;
					//					debug_vect("GPS local", gps_local);
				}
			}
			if (global_data.state.gps_mode)
			{
				gps_send_local_origin();
			}
			beep_on_low_voltage();

			if (global_data.state.mav_mode == MAV_MODE_RC_TRAINING)
			{
				send_system_state();
				static uint8_t uart_unconfigured = 1;
				if (uart_unconfigured)
				{
					// Mode for FMSPIC adapter
					uart1_init(19200, COMM_UART_MODE, UART_FIFO_8);
					uart_unconfigured = 0;
				}

				while (1)
				{
					loop_start_time = sys_time_clock_get_time_usec();

					///////////////////////////////////////////////////////////////////////////
					/// RC INTERFACE HACK AT 50 Hz
					///////////////////////////////////////////////////////////////////////////
					if (us_run_every(20000, COUNTER8, loop_start_time))
					{
						// Write start byte
						uart1_transmit(0xFF);

						// Write channels 1-8
						// The format works with FMS and CRRCSim model flight simulators
						for (int i = 1; i < 9; i++)
						{
							uart1_transmit(
									clamp((radio_control_get_channel(i)+1)*127, 0, 254));
						}
						led_toggle(LED_RED);
					}
				}
			}

		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 20 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(50000, COUNTER7, loop_start_time))
		{
			//led_toggle(LED_YELLOW);

			if (global_data.param[PARAM_GPS_MODE] >= 10)
			{
				//get thru all gps messages
				debug_message_send_one();
			}

			communication_send_attitude_position(loop_start_time);

			// Send parameter
			communication_queued_send();

			//infrared distance
			float_vect3 infra;
			infra.x = global_data.ground_distance;
			infra.y = global_data.ground_distance_unfiltered;
			infra.z = global_data.state.ground_distance_ok;
			debug_vect("infrared", infra);
		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 200 Hz functions                                     //
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(5000, COUNTER5, loop_start_time))
		{
			if (global_data.state.status == MAV_STATE_STANDBY)
			{
				//Check if parameters should be written or read
				param_handler();
			}
#if 0
			/*
			 //debug_message_buffer("HAllo Kalman");

			 //altitude kalman filter

			 //initalize matrices
			 const float t = 1.0f / 200.0f;

			 m_elem kal_z_a_a[4*4] =
			 { 1, t, t * t / 2, 0,
			 0, 1, t, 0 ,
			 0, 0, 1, 0 ,
			 0, 0, 0, 1  };
			 matrix_t kal_z_a=matrix_create(4,4,kal_z_a_a);

			 m_elem kal_z_c_a[2*4] =
			 {
			 1, 0, 0, 0 ,
			 0, 0, 1, 1  };
			 matrix_t kal_z_c=matrix_create(2,4,kal_z_c_a);

			 //			m_elem kal_z_gain_a[4*2] =
			 //			{
			 //			 0.003309636393353, 3.737518562014015e-08 ,
			 //			 0.001096997145729, 1.491062811791067e-05 ,
			 //			 6.708962603091765e-06, 0.997017866024232 ,
			 //			 -6.708911968356911e-06, 9.995411940282084e-11  };
			 //			matrix_t kal_z_gain = matrix_create(4, 2, kal_z_gain_a);

			 m_elem kal_z_gain_combo_a[4 * 2] =
			 {
			 0.0148553889079401,	3.73444963864759e-08,
			 0.00555539506146299,	1.49106022715582e-05,
			 0.000421844252811475,	0.997017766710577,
			 -0.000421844052617397,	9.97097528182815e-08};
			 //			{ 0.0131153410622187, 3.73751856201402e-08,
			 //			0.00432570190765015, 1.49106281179107e-05,
			 //			1.33518572806910e-05, 0.997017866024232,
			 //			-1.33516567327080e-05, 0.001 };
			 matrix_t kal_z_gain_combo=matrix_create(4,2,kal_z_gain_combo_a);


			 m_elem kal_z_gain_start_a[4*2] =
			 {
			 0.060188321659420, 3.566208652525075e-16 ,
			 0.008855645697701, 1.495920063190432e-13 ,
			 6.514669086807784e-04, 9.997000796699675e-08 ,
			 -6.514669086807778e-04, 0.999700079925069  };
			 matrix_t kal_z_gain_start=matrix_create(4,2,kal_z_gain_start_a);

			 m_elem kal_z_gain_start_part_a[4*2] = {};
			 matrix_t kal_z_gain_start_part=matrix_create(4,2,kal_z_gain_start_part_a);

			 m_elem kal_z_gain_part_a[4*2] = {};
			 matrix_t kal_z_gain_part=matrix_create(4,2,kal_z_gain_part_a);

			 m_elem kal_z_gain_sum_a[4*2] = {};
			 matrix_t kal_z_gain_sum=matrix_create(4,2,kal_z_gain_sum_a);

			 static m_elem kal_z_x_apriori_a[4*1] =
			 {
			 0 ,
			 0 ,
			 0 ,
			 -9.81  };
			 static matrix_t kal_z_x_apriori;
			 kal_z_x_apriori=matrix_create(4,1,kal_z_x_apriori_a);

			 static m_elem kal_z_x_aposteriori_a[4*1] =
			 {
			 0 ,
			 0 ,
			 0 ,
			 -9.81  };
			 static matrix_t kal_z_x_aposteriori;
			 kal_z_x_aposteriori=matrix_create(4,1,kal_z_x_aposteriori_a);

			 m_elem kal_z_measurement_a[2*1] =
			 {
			 0 ,
			 0  };
			 matrix_t kal_z_measurement=matrix_create(2,1,kal_z_measurement_a);


			 m_elem kal_z_error_a[2*1] =
			 {
			 0 ,
			 0  };
			 matrix_t kal_z_error=matrix_create(2,1,kal_z_error_a);

			 m_elem kal_z_measurement_estimate_a[2*1] =
			 {
			 0 ,
			 0  };
			 matrix_t kal_z_measurement_estimate=matrix_create(2,1,kal_z_measurement_estimate_a);

			 m_elem kal_z_x_update_a[4*1] =
			 {
			 0 ,
			 0 ,
			 0 ,
			 0  };
			 matrix_t kal_z_x_update=matrix_create(4,1,kal_z_x_update_a);

			 static int nopressure=0;
			 if(nopressure++==3){
			 nopressure=0;
			 //prepare measurement data
			 //measurement #1 pressure => relative altitude
			 sensors_pressure_bmp085_read_out();

			 static float altitude_local_origin = 0;

			 if (abs(calc_altitude_pressure(global_data.pressure_raw))
			 < 2000)
			 {
			 if (altitude_local_origin)
			 {
			 M(kal_z_measurement, 0, 0) = -calc_altitude_pressure(
			 global_data.pressure_raw)
			 - altitude_local_origin;
			 }
			 else
			 {
			 altitude_local_origin = -calc_altitude_pressure(
			 global_data.pressure_raw);
			 }
			 mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN],
			 50, M(kal_z_measurement, 0, 0));
			 }
			 }
			 if(nopressure==2){
			 //for temp measurement
			 sensors_pressure_bmp085_read_out();
			 }


			 //measurement #2 acceleration
			 float_vect3 acc_nav;
			 body2navi(&global_data.accel_si, &global_data.attitude,
			 &acc_nav);
			 M(kal_z_measurement,1,0)=acc_nav.z;

			 //			M(kal_z_measurement,0,0)=0;
			 //			M(kal_z_measurement,1,0)=0;

			 //			if (!isnumber(M(kal_z_x_aposteriori, 0, 0))
			 //				||(!isnumber(M(kal_z_x_aposteriori, 1, 0)))
			 //					 ||(!isnumber(M(kal_z_x_aposteriori, 2, 0)))
			 //						|| (!isnumber(M(kal_z_x_aposteriori, 3, 0)))){
			 //				M(kal_z_x_aposteriori, 0, 0) = 0;
			 //				M(kal_z_x_aposteriori, 1, 0) = 0;
			 //				M(kal_z_x_aposteriori, 2, 0) = 0;
			 //				M(kal_z_x_aposteriori, 3, 0) = 0;}

			 //time update
			 //kal_z_x_apriori = kal_z_a * kal_z_x_aposteriori
			 matrix_mult(kal_z_a, kal_z_x_aposteriori, kal_z_x_apriori);

			 //
			 //			if (!isnumber(M(kal_z_x_apriori, 0, 0)))
			 //				M(kal_z_x_apriori, 0, 0) = 0;
			 //			if (!isnumber(M(kal_z_x_apriori, 1, 0)))
			 //				M(kal_z_x_apriori, 1, 0) = 0;
			 //			if (!isnumber(M(kal_z_x_apriori, 2, 0)))
			 //				M(kal_z_x_apriori, 2, 0) = 0;
			 //			if (!isnumber(M(kal_z_x_apriori, 3, 0)))
			 //				M(kal_z_x_apriori, 3, 0) = 0;


			 //measurement update
			 //both measurements
			 //x(:,i+1)=xapriori+(gainfactor*[M_50(:,1) M(:,2)]+(1-gainfactor)*M_start)*(z-C*xapriori);


			 //est=C*xapriori;
			 matrix_mult(kal_z_c, kal_z_x_apriori, kal_z_measurement_estimate);
			 //error=(z-C*xapriori) = measurement-estimate
			 matrix_sub(kal_z_measurement, kal_z_measurement_estimate,
			 kal_z_error);
			 if(nopressure){
			 M(kal_z_error,0,0)=0;
			 }
			 //
			 //				m_elem kal_z_no_pressure_a[2*2] =
			 //				{
			 //				 0,0 ,
			 //				 0,1  };
			 //				matrix_t kal_z_no_pressure=matrix_create(2,1,kal_z_no_pressure_a);
			 //				matrix_mult(kal_z_no_pressure,kal_z_error,kal_z_error); //works because of the structure of matrix
			 //			}

			 //			//dont update from measurements for testing
			 //			M(kal_z_error,0,0)=0;
			 //			M(kal_z_error,1,0)=0;

			 const float gainfactor_steps = 1000;
			 static float gainfactor=0;
			 gainfactor = gainfactor * (1 - 1 / gainfactor_steps)
			 + 1 * 1 / gainfactor_steps;

			 matrix_mult_scalar(gainfactor, kal_z_gain_combo, kal_z_gain_part);

			 matrix_mult_scalar(1 - gainfactor, kal_z_gain_start,
			 kal_z_gain_start_part);

			 matrix_add(kal_z_gain_start_part,kal_z_gain_part,kal_z_gain_sum);

			 //gain*(z-C*xapriori)
			 matrix_mult(kal_z_gain_sum, kal_z_error, kal_z_x_update);

			 //xaposteriori = xapriori + update

			 matrix_add(kal_z_x_apriori,kal_z_x_update,kal_z_x_aposteriori);

			 float_vect3 acc_press;
			 //			global_data.position.z = M(kal_z_x_aposteriori,0,0);
			 acc_press.x = M(kal_z_x_aposteriori,0,0);
			 acc_press.y = M(kal_z_x_aposteriori,1,0);
			 acc_press.z = M(kal_z_x_aposteriori, 2, 0);
			 debug_vect("press_accel", acc_press);



			 //			//testing old single kalman
			 //			float kal_z = calc_altitude_pressure(global_data.pressure_raw);
			 //			if (abs(kal_z) > 2000)
			 //			{
			 //				kal_z = 0;
			 //			}
			 //			else
			 //			{

			 //				const float kal_num = 200;		//Anzahl der Werte über die ~ gefiltert wird
			 //				static float kal_mean = 0;		//Tiefpass =~ Mittelwert
			 //				static float kal_variance = 0;	//Varianz
			 //
			 //				if (kal_mean == 0)
			 //				{
			 //					kal_mean = kal_z;
			 //				}
			 //
			 //				kal_mean = kal_mean * (1 - 1 / kal_num) + kal_z / kal_num;
			 //
			 //				kal_variance = kal_variance * (1 - 1 / kal_num) + (kal_z
			 //						- kal_mean) * (kal_z - kal_mean) / kal_num;

			 //				static float kal_x = 0;
			 //				static float kal_p = 100000000;
			 //				static float kal_k = 1;
			 //
			 //				static float kal_q = 0.00001;
			 //				static float kal_r = 4;
			 //
			 //				//predict
			 //				kal_x = kal_x;
			 //				kal_p = kal_p + kal_q;
			 //
			 //				//correct
			 //				kal_k = kal_p / (kal_p + kal_r);
			 //				kal_x = kal_x + kal_k * (kal_z - kal_x);
			 //				kal_p = (1 - kal_k) * kal_p;
			 //
			 //				float_vect3 kal;
			 //				kal.x = kal_x;
			 //				kal.y = kal_p;
			 //				kal.z = kal_k;
			 //				kal.y = kal_mean;
			 //				kal.z = kal_variance;

			 //debug_vect("alt_kal", kal);

			 //mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN],
			 //	50, kal_z);
			 //					mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 51, global_data.pressure_raw);


			 //for logging
			 //				float_vect3 acc_nav;
			 //				body2navi(&global_data.accel_si, &global_data.attitude,
			 //						&acc_nav);
			 //				float_vect3 acc_press;
			 //				acc_press.x = kal_z;
			 //				acc_press.y = acc_nav.z;
			 //				acc_press.z = global_data.temperature;
			 //				debug_vect("press_accel", acc_press);
			 //			}
			 */
#endif
		}
		///////////////////////////////////////////////////////////////////////////

		else
		{
			// All Tasks are fine and we have no starvation
			last_mainloop_idle = loop_start_time;
		}

		// Read out comm at max rate - takes only a few microseconds in worst case
		communication_receive();

		// MCU load measurement
		uint64_t loop_stop_time = sys_time_clock_get_time_usec();
		global_data.cpu_usage = measure_avg_cpu_load(loop_start_time,
				loop_stop_time, min_mainloop_time);
		global_data.cpu_peak = measure_peak_cpu_load(loop_start_time,
				loop_stop_time, min_mainloop_time);
		time_debug.y = max(time_debug.y, global_data.cpu_usage);
		time_debug.z = max(time_debug.z, global_data.cpu_peak);
		if (loop_start_time - last_mainloop_idle >= 20000)
		{
			debug_message_buffer(
					"CRITICAL WARNING! CPU LOAD TO HIGH. STARVATION!");
			last_mainloop_idle = loop_start_time;//reset to prevent multiple messages
		}
		if (global_data.cpu_usage > 800)
		{
			// CPU load higher than 80%
			debug_message_buffer("CRITICAL WARNING! CPU LOAD HIGHER THAN 80%");
		}
	} // End while(1)

}

/**
 * @brief Update the controller gains
 *
 * Calling this function reads the controller gains from the global_data.param[]
 * data structure. This structure can be written from the GCS and is read from
 * EEPROM on startup.
 */
void update_controller_parameters(void)
{
	// "Zero position drift" attitude output
	// This the same as trimming an aircraft (the trim levers on a remote control)
	// To compensate mechanical imprecision
	// Using this parameter allows to exchange the remote control
	// and leave the trim at zero

	global_data.attitude_setpoint_offset.x
			= global_data.param[PARAM_ATT_OFFSET_X];
	global_data.attitude_setpoint_offset.y
			= global_data.param[PARAM_ATT_OFFSET_Y];
	global_data.attitude_setpoint_offset.z
			= global_data.param[PARAM_ATT_OFFSET_Z];

	/// ATTITUDE PID PARAMETERS

	pid_set_parameters(&nick_controller, global_data.param[PARAM_PID_ATT_P],
			global_data.param[PARAM_PID_ATT_I],
			global_data.param[PARAM_PID_ATT_D],
			global_data.param[PARAM_PID_ATT_AWU]);
	pid_set_parameters(&roll_controller, global_data.param[PARAM_PID_ATT_P],
			global_data.param[PARAM_PID_ATT_I],
			global_data.param[PARAM_PID_ATT_D],
			global_data.param[PARAM_PID_ATT_AWU]);
	pid_set_parameters(&yaw_pos_controller,
			global_data.param[PARAM_PID_YAWPOS_P],
			global_data.param[PARAM_PID_YAWPOS_I],
			global_data.param[PARAM_PID_YAWPOS_D],
			global_data.param[PARAM_PID_YAWPOS_AWU]);
	pid_set_parameters(&yaw_speed_controller,
			global_data.param[PARAM_PID_YAWSPEED_P],
			global_data.param[PARAM_PID_YAWSPEED_I],
			global_data.param[PARAM_PID_YAWSPEED_D],
			global_data.param[PARAM_PID_YAWSPEED_AWU]);

	/// POSITION PID PARAMETERS

	pid_set_parameters(&x_axis_controller, global_data.param[PARAM_PID_POS_P],
			global_data.param[PARAM_PID_POS_I],
			global_data.param[PARAM_PID_POS_D],
			global_data.param[PARAM_PID_POS_AWU]);
	pid_set_parameters(&y_axis_controller, global_data.param[PARAM_PID_POS_P],
			global_data.param[PARAM_PID_POS_I],
			global_data.param[PARAM_PID_POS_D],
			global_data.param[PARAM_PID_POS_AWU]);
	if (global_data.state.fly != FLY_RAMP_UP)
	{
		pid_set_parameters(&z_axis_controller,
				global_data.param[PARAM_PID_POS_Z_P],
				global_data.param[PARAM_PID_POS_Z_I],
				global_data.param[PARAM_PID_POS_Z_D],
				global_data.param[PARAM_PID_POS_Z_AWU]);
	}
}

void update_controller_setpoints(void)
{
	if (global_data.param[PARAM_POSITIONSETPOINT_ACCEPT])
	{
		//ramp for setpoint changes
		float setpoint_step = 0.03;
		float setpoint_step_yaw = 0.05;// 0.01;// 0.004;
		if (global_data.position_setpoint.x + setpoint_step
				< global_data.param[PARAM_POSITION_SETPOINT_X])
		{
			global_data.position_setpoint.x += setpoint_step;
		}
		else if (global_data.position_setpoint.x - setpoint_step
				> global_data.param[PARAM_POSITION_SETPOINT_X])
		{
			global_data.position_setpoint.x -= setpoint_step;
		}

		if (global_data.position_setpoint.y + setpoint_step
				< global_data.param[PARAM_POSITION_SETPOINT_Y])
		{
			global_data.position_setpoint.y += setpoint_step;
		}
		else if (global_data.position_setpoint.y - setpoint_step
				> global_data.param[PARAM_POSITION_SETPOINT_Y])
		{
			global_data.position_setpoint.y -= setpoint_step;
		}

		if (global_data.position_setpoint.z + setpoint_step
				< global_data.param[PARAM_POSITION_SETPOINT_Z])
		{
			global_data.position_setpoint.z += setpoint_step;
		}
		else if (global_data.position_setpoint.z - setpoint_step
				> global_data.param[PARAM_POSITION_SETPOINT_Z])
		{
			global_data.position_setpoint.z -= setpoint_step;
		}

		if (global_data.yaw_pos_setpoint + setpoint_step_yaw
				< global_data.param[PARAM_POSITION_SETPOINT_YAW])
		{
			global_data.yaw_pos_setpoint += setpoint_step_yaw;
		}
		else if (global_data.yaw_pos_setpoint - setpoint_step_yaw
				> global_data.param[PARAM_POSITION_SETPOINT_YAW])
		{
			global_data.yaw_pos_setpoint -= setpoint_step_yaw;
		}

	}
}

#endif
