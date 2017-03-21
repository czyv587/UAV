/*
 * flow.h
 *
 *  Created on: 2016-2-23
 *      Author: Administrator
 */

#ifndef FLOW_H_
#define FLOW_H_

#include "DSP2833x_Device.h"

unsigned char flow1_[26]={0};
typedef  struct i2c_frame
{
	Uint16 frame_count;
	int16 pixel_flow_x_sum;
	int16 pixel_flow_y_sum;
	int16 flow_comp_m_x;
	int16 flow_comp_m_y;
	int16 qual;
	int16 gyro_x_rate;
	int16 gyro_y_rate;
	int16 gyro_z_rate;
	unsigned char gyro_range;
	unsigned char sonar_timestamp;
    int16 ground_distance;
} i2c_frame;

#define I2C_FRAME_SIZE (sizeof(i2c_frame))

typedef struct i2c_integral_frame
{
	Uint16 frame_count_since_last_readout;
	int16 pixel_flow_x_integral;
	int16 pixel_flow_y_integral;
	int16 gyro_x_rate_integral;
	int16 gyro_y_rate_integral;
	int16 gyro_z_rate_integral;
	Uint32 integration_timespan;
	Uint32 sonar_timestamp;
	Uint16 ground_distance;
	int16 gyro_temperature;
	unsigned char qual;
} i2c_integral_frame;

#define I2C_INTEGRAL_FRAME_SIZE (sizeof(i2c_integral_frame))
//i2c_frame f;
//i2c_integral_frame f_integral;

#endif /* FLOW_H_ */
