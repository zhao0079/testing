/*
 * optical_flow.h
 *
 *  Created on: Jun 10, 2011
 *      Author: Laurens Mackay
 */

#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

#define OPTICAL_FLOW_CMD_READ_XY 0

#define OPTICAL_FLOW_BUS 0

#define OPTICAL_FLOW_ADDRESS 80


uint8_t optical_flow_get_dxy(uint8_t address, float * delta_x, float * delta_y, float * qual);

void optical_flow_debug_vect_send(void);

void optical_flow_start_read(uint8_t address);

void optical_flow_read_handler(i2c_package *package);

#endif /* OPTICAL_FLOW_H_ */
