/*
 * optical_flow.c
 *
 *  Created on: Jun 10, 2011
 *      Author: Laurens Mackay
 */

#include "features.h"
#include "conf.h"
#include "i2c.h"
#include "optical_flow.h"
#include "mav_vect.h"
#include "debug.h"


#ifndef NULL
#define NULL ((void*)0)
#endif


float_vect3 optical_flow_result;
uint8_t optical_flow_valid = 0;

uint8_t optical_flow_get_dxy(uint8_t address, float * delta_x, float * delta_y, float * qual)
{
	*delta_x = optical_flow_result.x;
	*delta_y = optical_flow_result.y;
	*qual = optical_flow_result.z;

	if (optical_flow_valid)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
void optical_flow_debug_vect_send(void){
	debug_vect("opt_flow",optical_flow_result);
}
void optical_flow_start_read(uint8_t address)
{
	i2c_package package_read, package_write;
	package_write.data[0] = OPTICAL_FLOW_CMD_READ_XY;
	package_write.length = 1;
	package_write.direction = I2C_WRITE;
	package_write.slave_address = address;
	package_write.bus_number = OPTICAL_FLOW_BUS;
	package_write.write_read = 1;
	package_write.i2c_done_handler = NULL;

	package_read.length = 3;
	package_read.direction = I2C_READ;
	package_read.slave_address = address;
	package_read.bus_number = OPTICAL_FLOW_BUS;
	package_read.write_read = 1; // make repeated start after this package to receive data
	package_read.i2c_done_handler = (void*) &optical_flow_read_handler;

	i2c_write_read(&package_write, &package_read);

	//i2c_op(&package_write);
}

void optical_flow_read_handler(i2c_package *package)
{
	if (package->i2c_error_code != I2C_CODE_ERROR)
	{
		optical_flow_result.x = ((int8_t*) package->data)[0];
		optical_flow_result.y = ((int8_t*) package->data)[1];
		optical_flow_result.z = package->data[2];

		//debug
		//debug_vect("opt_flow",optical_flow_result);

		//Check if the sensor sees too little features
		if (optical_flow_result.z < 10 || optical_flow_result.z == 255)
		{
			optical_flow_valid = 0;
			//debug_message_buffer("optical_flow not valid");
		}
		else
		{
			optical_flow_valid = 1;
			//debug_message_buffer("optical_flow valid");
		}
	}
	else
	{
		debug_message_buffer("optical_flow i2c Error");
		optical_flow_valid = 0;
	}
}
