/*
 * MS5611.h
 *
 *  Created on: 2016-1-5
 *      Author: Administrator
 */

#ifndef MS5611_H_
#define MS5611_H_
#include "DSP2833x_Device.h"
void readTemperatureRequestPressure(void);
void readPressureRequestPressure(void);
void readPressureRequestTemperature(void);
void calculateTemperature(void);
void calculatePressureAltitude(void);
void MS5611_init(void);


#endif /* MS5611_H_ */
