#include "define.h"
#include "DSP2833x_Device.h" 
#include "data_transfer.h"
#include "I2C.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
//#define Gyr_Gain 0.061;

unsigned char data_to_send[50];
unsigned char Data_Check,Send_Status,Send_Senser,Send_RCData,Send_Offset,Send_PID1,Send_PID2,Send_PID3,Send_PID4,Send_PID5,Send_PID6,Send_MotoPwm;
extern float Pitch,Roll,Yaw;
extern unsigned char mpu6050_buffer[14];
extern int mpu6050buffer[14];

extern struct DATA_XYZ_F GYR_F;
extern struct DATA_XYZ ACC_AVG;
extern struct DATA_XYZ GYR_RATE;
extern struct DATA_XYZ ACC;
extern struct DATA_XYZ GYR;
/*void Data_Send_Status(void)
{
	unsigned char _cnt=0;
	int _temp;
	unsigned char sum = 0;
	unsigned char i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(Roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
////////////////	int _temp2 = Alt;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
		
	if(Rc_C.ARMED==0)			data_to_send[_cnt++]=0xA0;	//锁定
	else if(Rc_C.ARMED==1)		data_to_send[_cnt++]=0xA1;////////////////
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
//#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
}*/
void Data_Send_Status(float Pitch,float Roll,float Yaw,unsigned int *gyro,unsigned int *accel)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;
	//unsigned int _temp;
	unsigned char data_to_send[50];
	unsigned char _data[6];
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_data[0] = ((int)(Roll*100) >> 8) & 0xFF;
	_data[1] = (int)(Roll*100)&0xff;
	data_to_send[_cnt++]=_data[0];
	data_to_send[_cnt++]=_data[1];
	_data[2] = ((int)(Pitch*100) >> 8) & 0xFF;
	_data[3] = (int)(Pitch*100)&0xff;
	data_to_send[_cnt++]=_data[2];
	data_to_send[_cnt++]=_data[3];
	_data[4] = ((int)(Yaw*100) >> 8) & 0xFF;
	_data[5] = (int)(Yaw*100)&0xff;
	data_to_send[_cnt++]=_data[4];
	data_to_send[_cnt++]=_data[5];
	
	data_to_send[3] = _cnt-4;
	//和校验
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//串口发送数据
	for(i=0;i<_cnt;i++)
		scib_xmit(data_to_send[i]);
} 
void Data_Send_Senser(void)
{
	unsigned char _cnt=0;
	unsigned char sum = 0;
	unsigned char i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	/*data_to_send[_cnt++]=BYTE1(ACC.X);
	data_to_send[_cnt++]=BYTE0(ACC.X);
	data_to_send[_cnt++]=BYTE1(ACC.Y);
	data_to_send[_cnt++]=BYTE0(ACC.Y);
	data_to_send[_cnt++]=BYTE1(ACC.Z);
	data_to_send[_cnt++]=BYTE0(ACC.Z);
	data_to_send[_cnt++]=BYTE1(GYR_RATE.X);
	data_to_send[_cnt++]=BYTE0(GYR_RATE.X);
	data_to_send[_cnt++]=BYTE1(GYR_RATE.Y);
	data_to_send[_cnt++]=BYTE0(GYR_RATE.Y);
	data_to_send[_cnt++]=BYTE1(GYR_RATE.Z);
	data_to_send[_cnt++]=BYTE0(GYR_RATE.Z);*/
	data_to_send[_cnt++]=((int)(ACC.X/1024)>> 8)& 0xFF;///8192
	data_to_send[_cnt++]=(int)(ACC.X/1024)& 0xFF;
	data_to_send[_cnt++]=((int)(ACC.Y/1024)>> 8)& 0xFF;
	data_to_send[_cnt++]=(int)(ACC.Y/1024)& 0xFF;
	data_to_send[_cnt++]=((int)(ACC.Z/1024)>> 8)& 0xFF;
	data_to_send[_cnt++]=(int)(ACC.Z/1024)& 0xFF;
	data_to_send[_cnt++]=((int)(GYR.X*0.061) >> 8) & 0xFF;
	data_to_send[_cnt++]=(int)(GYR.X*0.061)&0xff;
	data_to_send[_cnt++]=((int)(GYR.Y*0.061) >> 8) & 0xFF;
	data_to_send[_cnt++]=(int)(GYR.Y*0.061)&0xff;
	data_to_send[_cnt++]=((int)(GYR.Z*0.061) >> 8) & 0xFF;
	data_to_send[_cnt++]=(int)(GYR.Z*0.061)&0xff;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
//#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
	mpu6050buffer[0]=(int)(mpu6050_buffer[0]);
	mpu6050buffer[1]=(int)(mpu6050_buffer[1]);
	mpu6050buffer[2]=(int)(mpu6050_buffer[2]);
	mpu6050buffer[3]=(int)(mpu6050_buffer[3]);
	mpu6050buffer[4]=(int)(mpu6050_buffer[4]);
	mpu6050buffer[5]=(int)(mpu6050_buffer[5]);
}

