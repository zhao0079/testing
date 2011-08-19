/*
 * sonar_distance.h
 *
 *  Created on: 29.06.2010
 *      Author: Laurens Mackay
 */

#ifndef SONAR_DISTANCE_H_
#define SONAR_DISTANCE_H_

//Infrared distance sensor

static inline float sonar_distance_get(void)
{

	uint16_t adc_value = adc_get_value(ADC_6_CHANNEL);
	float adc_volt = ((float) adc_value) / 310.0f;

	//Calculate distance, 10mV / inch, 0.0254 m = 1 inch
	float distance = adc_volt / 0.010 * 0.0254;
	return distance;
}
#endif /* SONAR_DISTANCE_H_ */
