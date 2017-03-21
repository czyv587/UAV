#include "MS5611.h"
#include "MPU6050.h"
#include "define.h"
#include "control.h"
#include "hardware_I2C.h"
#include "imu.h"
#include "math.h"
#include "DSP2833x_Device.h"
#include "I2C.h"
#include "nrf24l01.h"

#define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)
#define MS561101BA_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

Uint16 c1, c2, c3, c4,c5, c6;

Uint32 d1;

Uint32 d2;

long long dT;

void readTemperatureRequestPressure(void)
{
	unsigned char data[3];

	hw_i2cRead(MS561101BA_ADDR_CSB_LOW, 0x00, 3, data);    // Request temperature read
	delay();delay();delay();
	delay();delay();delay();
    d2 = ((Uint32)data[0] << 16) | ((Uint32)data[1] << 8)        | data[2];//温度数值d2
    calculateTemperature();

    hw_Single_MS5611_Write(MS561101BA_ADDR_CSB_LOW, 0x48);//OSR 4096
 //   delay14ms();
}
void readPressureRequestPressure(void)
{
	unsigned char data[3];

	hw_i2cRead(MS561101BA_ADDR_CSB_LOW, 0x00, 3, data);    // Request pressure read
	delay();delay();delay();
	delay();delay();delay();
    d1 = ((Uint32)data[0] << 16) | ((Uint32)data[1] << 8)        | data[2];//气压数值d1
 //   hw_Single_MS5611_Write(MS561101BA_ADDR_CSB_LOW, 0x58);
    hw_Single_MS5611_Write(MS561101BA_ADDR_CSB_LOW, 0x48);
}
void readPressureRequestTemperature(void)
{
	unsigned char data[3];

	hw_i2cRead(MS561101BA_ADDR_CSB_LOW, 0x00, 3, data);    // Request pressure read
	delay();delay();delay();
	delay();delay();delay();
    d1 = ((Uint32)data[0] << 16) | ((Uint32)data[1] << 8)        | data[2];//气压数值d1
    calculatePressureAltitude();
    hw_Single_MS5611_Write(MS561101BA_ADDR_CSB_LOW, 0x58);
 //   delay14ms();
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////
long dd;
Uint32 cc=0;
void calculateTemperature(void)
{
	long ddd=0;
	cc=((Uint32)c5 << 8);
	if(d2>cc)
		{
			dT = d2 - cc;
			ddd=(long)((dT * c6 ) >> 23);
			dd =  2000 + ddd;
		}
	else if(d2<=cc)
		{
			dT = cc - d2;
			ddd=(long)((dT * c6 ) >> 23);
			dd =  2000 - ddd;
		}

}

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////
Uint32 pressureAlt=0;
Uint32 p=0;
void calculatePressureAltitude(void)
{
	long long Aux,OFF2,SENS2,T2;        //温度校验
	long long offset;
	long long sens;

//        int32_t d1Average = 110;
	if(d2>cc)
	{
		offset  = ((Uint32)c2 << 16) + ((c4 * (long long)dT) >> 7);
		sens    = ((Uint32)c1 << 15) + ((c3 * (long long)dT) >> 8);
	}
	else if(d2<=cc)
	{
		offset  = ((Uint32)c2 << 16) - ((c4 * (long long)dT) >> 7);
		sens    = ((Uint32)c1 << 15) - ((c3 * (long long)dT) >> 8);
	}
    if(dd < 2000)
    {
    	T2=(dT*dT)>>31;//约为0   正
    	Aux=(2000-dd)*(2000-dd);//dd*dd;//   正
    	OFF2=2.5*Aux;//约为0   正
    	SENS2 = 1.25*Aux;//约为0   正
    	dd=dd-T2;//正
    	offset=offset-OFF2;//正
    	sens=sens-SENS2;//正
    }
    p = (((d1 * sens) >> 21) - offset) >> 15;

    pressureAlt = (44330.0f * (1.0f - pow((float)p / 101325.0f, 0.190295f)));

//    return pressureAlt;
}

void MS5611_init(void)
{
	unsigned char data[2];

	hw_Single_MS5611_Write(MS561101BA_ADDR_CSB_LOW, 0x1E);      // Reset Device

	Delayms(10); // delay_us(10);
//	while(1)
//	{
		hw_i2cRead(MS561101BA_ADDR_CSB_LOW, 0xA2, 2, data);    // Read Calibration Data C1
        c1 = ((Uint16)data[0] << 8) | data[1];//c1，气压敏感度，SENS
        Delayms(10);

        hw_i2cRead(MS561101BA_ADDR_CSB_LOW, 0xA4, 2, data);    // Read Calibration Data C2
        c2 = ((Uint16)data[0] << 8) | data[1];//c2，气压偏差值，OFF
        Delayms(10);

        hw_i2cRead(MS561101BA_ADDR_CSB_LOW, 0xA6, 2, data);    // Read Calibration Data C3
        c3 = ((Uint16)data[0] << 8) | data[1];//c3，气压敏感度的温度系数，简写为TCS
        Delayms(10);
    hw_i2cRead(MS561101BA_ADDR_CSB_LOW, 0xA8, 2, data);    // Read Calibration Data C4
        c4 = ((Uint16)data[0] << 8) | data[1];//c4，气压偏差值的温度系数，简写为TCO
        Delayms(10);
    hw_i2cRead(MS561101BA_ADDR_CSB_LOW, 0xAA, 2, data);    // Read Calibration Data C5
        c5 = ((Uint16)data[0] << 8) | data[1];//c5，参考温度，Tref
        Delayms(10);
    hw_i2cRead(MS561101BA_ADDR_CSB_LOW, 0xAC, 2, data);    // Read Calibration Data C6
        c6 = ((Uint16)data[0] << 8) | data[1];//c6，温度系数，TEMPSENS
        Delayms(10);
    hw_Single_MS5611_Write(MS561101BA_ADDR_CSB_LOW, 0x58);
 //   delay_us(10);
    delay14ms();
/*
    readTemperatureRequestPressure();
    readPressureRequestTemperature();
    readTemperatureRequestPressure();
    readPressureRequestTemperature();
    readTemperatureRequestPressure();
    readPressureRequestTemperature();
*/
    readTemperatureRequestPressure();
    delay14ms();
    readPressureRequestTemperature();
/*
    delay14ms();delay14ms();
    readPressureRequestPressure();
    delay14ms();delay14ms();
    readPressureRequestPressure();
    delay14ms();delay14ms();
*/


//    calculatePressureAltitude();
//	}
}

