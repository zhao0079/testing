/*
 * vision_buffer.h
 *
 *  Created on: 13.05.2010
 *      Author: Laurens Mackay
 */

#ifndef VISION_BUFFER_H_
#define VISION_BUFFER_H_
#include "comm.h"
#include "mavlink.h"
void vision_buffer_buffer_init(void);
void vision_buffer_buffer_camera_triggered(uint64_t usec, uint64_t loop_start_time, uint32_t seq);
void vision_buffer_handle_data(mavlink_vision_position_estimate_t* pos);
void vision_buffer_handle_global_data(mavlink_global_vision_position_estimate_t* pos);
#endif /* VISION_BUFFER_H_ */
