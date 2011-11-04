/*
 * mainloop.c
 *
 *  Created on: 08.10.2010
 *      Author: mackayl
 */

#include "conf.h"

#include "mainloop_quadrotor.h"
#include "common_mainloop_functions.h"

#include "inttypes.h"
#include "mcu_init.h"

// Include comm
#include "comm.h"

extern mavlink_system_t mavlink_system;

#include "pixhawk/mavlink.h"

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


/**
* @brief Initialize the whole system
*
* All functions that need to be called before the first mainloop iteration
* should be placed here.
*/
void main_init_generic(void)
{

	// Reset to safe values
	global_data_reset();

	// Load default eeprom parameters as fallback
	global_data_reset_param_defaults();

	// LOWLEVEL INIT, ONLY VERY BASIC SYSTEM FUNCTIONS
	hw_init();
	enableIRQ();
	led_init();
	led_on(LED_GREEN);
//	buzzer_init();
	sys_time_init();
	sys_time_periodic_init();
	sys_time_clock_init();
	ppm_init();
	pwm_init();

	// Lowlevel periphel support init
	adc_init();
	// FIXME SDCARD
//	MMC_IO_Init();
	spi_init();
	i2c_init();

	// Sensor init
	sensors_init();
	debug_message_buffer("Sensor initialized");

	// Shutter init
	shutter_init();
	shutter_control(0);

	// Debug output init
	debug_message_init();
	debug_message_buffer("Text message buffer initialized");

	// MEDIUM LEVEL INIT, INITIALIZE I2C, EEPROM, WAIT FOR MOTOR CONTROLLERS
	// Try to reach the EEPROM
	eeprom_check_start();

	// WAIT FOR 2 SECONDS FOR THE USER TO NOT TOUCH THE UNIT
	while (sys_time_clock_get_time_usec() < 2000000)
	{
	}

	// Do the auto-gyro calibration for 1 second
	// Get current temperature
	led_on(LED_RED);
	gyro_init();

//	uint8_t timeout = 3;
//	// Check for SD card
//	while (sys_time_clock_get_time_usec() < 2000000)
//	{
//		while (GetDriveInformation() != F_OK && timeout--)
//		  {
//		   debug_message_buffer("MMC/SD-Card not found ! retrying..");
//		  }
//	}
//
//	if (GetDriveInformation() == F_OK)
//	{
//		debug_message_buffer("MMC/SD-Card SUCCESS: FOUND");
//	}
//	else
//	{
//		debug_message_buffer("MMC/SD-Card FAILURE: NOT FOUND");
//	}
	//FIXME redo init because of SD driver decreasing speed
	//spi_init();
	led_off(LED_RED);

	// Stop trying to reach the EEPROM - if it has not been found by now, assume
	// there is no EEPROM mounted
	if (eeprom_check_ok())
	{
		param_read_all();
		debug_message_buffer("EEPROM detected - reading parameters from EEPROM");

		for (int i = 0; i < ONBOARD_PARAM_COUNT * 2 + 20; i++)
		{
			param_handler();
			//sleep 1 ms
			sys_time_wait(1000);
		}
	}
	else
	{
		debug_message_buffer("NO EEPROM - reading onboard parameters from FLASH");
	}

	// Set mavlink system
	mavlink_system.compid = MAV_COMP_ID_IMU;
	mavlink_system.sysid = global_data.param[PARAM_SYSTEM_ID];

	//Magnet sensor
	hmc5843_init();
	acc_init();

	// Comm parameter init
	mavlink_system.sysid = global_data.param[PARAM_SYSTEM_ID]; // System ID, 1-255
	mavlink_system.compid = global_data.param[PARAM_COMPONENT_ID]; // Component/Subsystem ID, 1-255

	// Comm init has to be
	// AFTER PARAM INIT
	comm_init(MAVLINK_COMM_0);
	comm_init(MAVLINK_COMM_1);

	// UART initialized, now initialize COMM peripherals
	communication_init();
	gps_init();

	us_run_init();

	servos_init();

	//position_kalman3_init();
	//	buzzer_init();

	// Calibration starts (this can take a few seconds)
	//	led_on(LED_GREEN);
	//	led_on(LED_RED);

	// Read out first time battery
	global_data.battery_voltage = battery_get_value();

	global_data.state.mav_mode = MAV_MODE_PREFLIGHT;
	global_data.state.status = MAV_STATE_CALIBRATING;

	send_system_state();

	float_vect3 init_state_accel;
	init_state_accel.x = 0.0f;
	init_state_accel.y = 0.0f;
	init_state_accel.z = -1000.0f;
	float_vect3 init_state_magnet;
	init_state_magnet.x = 1.0f;
	init_state_magnet.y = 0.0f;
	init_state_magnet.z = 0.0f;


	//auto_calibration();


	attitude_observer_init(init_state_accel, init_state_magnet);

	debug_message_buffer("Attitude Filter initialized");
	led_on(LED_RED);

	send_system_state();

	debug_message_buffer("System is initialized");

	// Calibration stopped
	led_off(LED_RED);

	global_data.state.mav_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	global_data.state.status = MAV_STATE_STANDBY;

	send_system_state();

	debug_message_buffer("Checking if remote control is switched on:");
	if (radio_control_status() == RADIO_CONTROL_ON)
	{
		global_data.state.mav_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_TEST_ENABLED;
		debug_message_buffer("RESULT: remote control switched ON");
		debug_message_buffer("Now in MAV_MODE_TEST2 position hold tobi_laurens");
		led_on(LED_GREEN);
	}
	else
	{
		global_data.state.mav_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		debug_message_buffer("RESULT: remote control switched OFF");
		led_off(LED_GREEN);
	}
}

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

	// Default filters, allow Vision, Vicon and optical flow inputs
	vicon_position_kalman_init();
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

			sensors_pressure_bmp085_read_out();

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
			attitude_tobi_laurens();

			if (global_data.state.position_estimation_mode == POSITION_ESTIMATION_MODE_VICON_ONLY ||
				global_data.state.position_estimation_mode == POSITION_ESTIMATION_MODE_VISION_VICON_BACKUP)
			{
				vicon_position_kalman();
			}
			else if (global_data.state.position_estimation_mode == POSITION_ESTIMATION_MODE_GPS_ONLY)
			{
				outdoor_position_kalman();
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

			uint8_t supersampling = 10;
			for (int i = 0; i < supersampling; ++i)
			{
				global_data.sonar_distance += sonar_distance_get(ADC_5_CHANNEL);
			}

			global_data.sonar_distance /= supersampling;

			opt_int.z = valid;
			mavlink_msg_optical_flow_send(global_data.param[PARAM_SEND_DEBUGCHAN], loop_start_time + sys_time_clock_get_unix_offset(), 0, global_data.optflow.x, global_data.optflow.y, global_data.optflow.z, global_data.sonar_distance_filtered);
			//optical_flow_debug_vect_send();
			//debug_vect("opt_int", opt_int);
			optical_flow_start_read(80);

			if (global_data.state.position_estimation_mode
					== POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_INTEGRATING
					|| global_data.state.position_estimation_mode
							== POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_NON_INTEGRATING
					|| global_data.state.position_estimation_mode
							== POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_ADD_VICON_AS_OFFSET
					|| global_data.state.position_estimation_mode
							== POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_ADD_VISION_AS_OFFSET
					|| global_data.state.position_estimation_mode
							== POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_ODOMETRY_ADD_VISION_AS_OFFSET
					|| global_data.state.position_estimation_mode
							== POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_VICON
					|| global_data.state.position_estimation_mode
							== POSITION_ESTIMATION_MODE_GPS_OPTICAL_FLOW
					|| global_data.state.position_estimation_mode
							== POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_GLOBAL_VISION
					|| global_data.state.position_estimation_mode
							== POSITION_ESTIMATION_MODE_OPTICAL_FLOW_ULTRASONIC_VISUAL_ODOMETRY_GLOBAL_VISION)
			{
				optflow_speed_kalman();
			}

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
			led_toggle(LED_RED);

			// Toggle active mode led
			if (global_data.state.mav_mode & MAV_MODE_FLAG_SAFETY_ARMED)
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

			if (global_data.param[PARAM_POSITION_YAW_TRACKING] == 1)
			{
				mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 0,
						90, global_data.param[PARAM_POSITION_SETPOINT_YAW]);
				mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 0,
						91, global_data.yaw_pos_setpoint);
			}
		}
		///////////////////////////////////////////////////////////////////////////


		///////////////////////////////////////////////////////////////////////////
		/// NON-CRITICAL SLOW 1 Hz functions
		///////////////////////////////////////////////////////////////////////////
		else if (us_run_every(1000000, COUNTER9, loop_start_time))
		{
			// Send system state, mode, battery voltage, etc.
			send_system_state();

			// Send position setpoint offset
			debug_vect("pos offs", global_data.position_setpoint_offset);

			// Send current onboard time
			mavlink_msg_system_time_send(MAVLINK_COMM_1, 0,
					sys_time_clock_get_unix_time());

			//update state from received parameters
			sync_state_parameters();

			//debug number of execution
			count = 0;

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
				}
			}
			if (global_data.state.gps_mode)
			{
				gps_send_local_origin();
			}
			beep_on_low_voltage();

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
		else
		{
			global_data.position_setpoint.x = global_data.param[PARAM_POSITION_SETPOINT_X];
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
		else
		{
			global_data.position_setpoint.y = global_data.param[PARAM_POSITION_SETPOINT_Y];
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
		else
		{
			global_data.position_setpoint.z = global_data.param[PARAM_POSITION_SETPOINT_Z];
		}

		float yaw_e = global_data.param[PARAM_POSITION_SETPOINT_YAW] - global_data.yaw_pos_setpoint;

		// don't turn around the wrong side (only works if yaw angle is between +- 180 degree)
		if (yaw_e > 3.14f)
		{
			yaw_e -= 2.0f * 3.14f;
		}
		if (yaw_e < -3.14f)
		{
			yaw_e += 2.0f * 3.14f;
		}


		if (yaw_e > 0 && yaw_e > setpoint_step_yaw)
		{
			global_data.yaw_pos_setpoint += setpoint_step_yaw;
		}
		else if (yaw_e < 0 && -yaw_e > setpoint_step_yaw)
		{
			global_data.yaw_pos_setpoint -= setpoint_step_yaw;
		}
		else
		{
			global_data.yaw_pos_setpoint = global_data.param[PARAM_POSITION_SETPOINT_YAW];
		}

//		//TODO Fix Yaw ramp
//		if (global_data.yaw_pos_setpoint + setpoint_step_yaw
//				< global_data.param[PARAM_POSITION_SETPOINT_YAW])
//		{
//			global_data.yaw_pos_setpoint += setpoint_step_yaw;
//		}
//		else if (global_data.yaw_pos_setpoint - setpoint_step_yaw
//				> global_data.param[PARAM_POSITION_SETPOINT_YAW])
//		{
//			global_data.yaw_pos_setpoint -= setpoint_step_yaw;
//		}
//		else
//		{
//			global_data.yaw_pos_setpoint = global_data.param[PARAM_POSITION_SETPOINT_YAW];
//		}

	}
}

