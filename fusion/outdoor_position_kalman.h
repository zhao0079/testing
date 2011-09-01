/*
 * outdoor_position_kalman.h
 *
 *  Created on: 02.12.2010
 *      Author: Laurens Mackay
 */

#ifndef OUTDOOR_POSITION_KALMAN_H_
#define OUTDOOR_POSITION_KALMAN_H_

void outdoor_position_kalman_init(void);
void outdoor_position_kalman(void);
void altitude_set_local_origin(void);
void altitude_set_local_origin_offset(float offset);
float outdoor_z_position;

#endif /* OUTDOOR_POSITION_KALMAN_H_ */
