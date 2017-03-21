#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"	
#include "spi.h"
#include "exti.h"
#include "timer.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "STM32_I2C.h"
#include "math.h"
#include "24L01.h"	
#include "define.h"
#include "imu.h"
#include "control.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
void Data_Send_Status(float Pitch,float Roll,float Yaw,u16 *gyro,u16 *accel);
void Data_Send_Senser(void);
extern struct DATA_XYZ_F GYR_F;
extern struct DATA_XYZ ACC_AVG;  
extern struct DATA_XYZ ACC_OFFSET;
extern struct DATA_XYZ GYR_OFFSET;
extern struct DATA_XYZ HMC;
extern struct DATA_XYZ GYR;
extern float rool,pitch;
extern int Motor1,Motor2,Motor3,Motor4; 
extern struct DATA_XYZ ACC;
extern float Pitch,Rool,Yaw;

u8 RX_buf[32]={0}; 
u8 TX_buf[32]={0};
u8 delays=10;	  ////  IIC delayspeed
u8 wireless=0;
u32 loss_wireless=0;
u8 a1=5,c1=5;
u8 b1=5,d1=5,e1=5;
u8 buf[18]={0X00};
//int iii=0x1234;
extern u8 FLY_EN;  
int main(void)			
{			  
	Stm32_Clock_Init(9);	//系统时钟设置	 
	delay_init(72);	   		//延时初始化
//	delay_ms(1000);
//	Stm32_Clock_Init(9);	//系统时钟设置
//	delay_init(72);	 		
	uart_init(72,115200);	 	//串口初始化为9600
//	uart_init1(115200);	 //串口初始化为115200
//  iii=(iii>>8)&0xff;
	
//	RCC->APB2ENR|=1<<0;	  	  // 开辅助时钟
//	AFIO->MAPR = 0x02000000;  // 禁用JTAG
//	LED_Init();	
//	Show_str_leds();
 	NRF24L01_Init();			//  无线初始化///////////////////////////////////////////////////////////////
  while( NRF24L01_Check());
 	RX_Mode();
	delay_ms(5);
	e1=NRF24L01_Read_Reg(RF_CH); 
	delay_ms(55);
	//LED_RED_ON;
	//delays=20;	     		//  init with low_speed iic	
  //MPU6050_INIT();delay_ms(50);
	//MPU6050_INIT();delay_ms(50);
	//Get_OFFSET(500); 
  //delays=10;			  	//  cap  with high_speed iic
	//if(ACC_OFFSET.X==0&&ACC_OFFSET.Y==0){ALL_LEDS_ON();while(1);}	 // 传感器失败,开灯阻塞 
//	PID_INIT();	  
//	uart_init1(115200);	 //串口初始化为115200
//	TIM2_PWM_Init(999,2);  	  
	//TIM3_Int_Init(24,7199); // 定时2MS 	 
	//delay_ms(800); 	ALL_LEDS_OFF();

	while(1) 
	{	  
		//if(wireless==1) LED_twinkle();
	//	Data_Send_Status(Pitch,Rool,Yaw,0,0);			   		
	//	Data_Send_Senser();
// 		d1=NRF24L01_Read_Reg(CD);
 //		b1=NRF24L01_RxPacket(buf);
 //		if(b1==0)
 //			while(1);
		delay_ms(30);
		b1=NRF24L01_RxPacket(buf);
 		if(b1==0)
		{
			Data_Send_Status(Pitch,Rool,Yaw,0,0);			   		
		  Data_Send_Senser();
// 			while(1);
		}
	} 		  		 
}
////////////////////////////////////////////////// TIMER3 _interrupt in 2ms----->>>>  Control 
void Tx_buf()
{
		if(GYR_F.X<0) 
		{
			TX_buf[0]=1;  		
			TX_buf[1]=(u8)(-GYR_F.X/255);
			TX_buf[2]=(u8)(-(int)GYR_F.X%255);
		}
		else 
		{
			TX_buf[0]=0;
			TX_buf[1]=(u8)( GYR_F.X/255);
			TX_buf[2]=(u8)( (int)GYR_F.X%255);
		}
		if(rool<0) 
		{
			TX_buf[3]=1;  		
			TX_buf[4]=(u8)(-rool/255);
			TX_buf[5]=(u8)(-(int)rool%255);
		}
		else 
		{
			TX_buf[3]=0;
			TX_buf[4]=(u8)( rool/255);
			TX_buf[5]=(u8)( (int)rool%255);
		}	
		TX_buf[6] =Motor1/255;
		TX_buf[7] =Motor1%255;
		TX_buf[8] =Motor2/255;
		TX_buf[9] =Motor2%255;
		TX_buf[10]=Motor3/255;
		TX_buf[11]=Motor3%255;
		TX_buf[12]=Motor4/255;
		TX_buf[13]=Motor4%255;
}
float 	angle=0;
long acc_hold[3]={0};
u16 tms=0;
float p_long=0.0,r_long=0.0;
float adj_r=0.0,adj_p=0.0;
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{					       				   				     	    	 
//     MPU6050_READ();
// 		MPU6050_CONVENT();
// 		ACC_SMOOTH(10);  // ACC 平滑滤波
// 		RX_buf[31]= 0;   ///// 
// 		MahonyIMUupdate(GYR_F.X,GYR_F.Y,GYR_F.Z,(float)ACC_AVG.X,(float)ACC_AVG.Y,(float)ACC_AVG.Z);	//函数内转成弧度
	//	Tx_buf();		
		//NRF24L01_RxPacket(RX_buf,TX_buf);	// CHECK WIRLE LESS		//	
		Data_Send_Status(Pitch,Rool,Yaw,0,0);			   		
		Data_Send_Senser();
	/*	if(RX_buf[31]==0)		  //  Check in every 2ms , and send in every 10ms 
		{
			loss_wireless++;
			if(FLY_EN==1&&loss_wireless>200)  // loss 20ms 
			{
				wireless=0;
				LED_GRE_ON;
				CRASH_LANDING();	
			}
		}
		else if(RX_buf[31]==0XAA)  // 正常接收  
		{
			READ_CONTROL_COMMAND(RX_buf);	
			LED_GRE_OFF;	
			loss_wireless=0;
			wireless=1;	
		} 	
		STABLE_WITH_PID();	  ////  PID CONTROL	      	*/		   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    			    		  			   
}
void Data_Send_Status(float Pitch,float Roll,float Yaw,u16 *gyro,u16 *accel)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;
	unsigned int _temp;
	u8 data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(Roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0-(int)(Pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	//和校验
	for(i=0;i<_cnt;i++)
		sum+= data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//串口发送数据
	for(i=0;i<_cnt;i++)
		AnBT_Uart1_Send_Char(data_to_send[i]);
} 
void Data_Send_Senser(void)
{
	unsigned char _cnt=0;
	unsigned char sum = 0;
	unsigned char i=0;
	u8 data_to_send[50];
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
	data_to_send[_cnt++]=((int)(ACC.X/8192)>> 8)& 0xFF;
	data_to_send[_cnt++]=(int)(ACC.X/8192)& 0xFF;
	data_to_send[_cnt++]=((int)(ACC.Y/8192)>> 8)& 0xFF;
	data_to_send[_cnt++]=(int)(ACC.Y/8192)& 0xFF;
	data_to_send[_cnt++]=((int)(ACC.Z/8192)>> 8)& 0xFF;
	data_to_send[_cnt++]=(int)(ACC.Z/8192)& 0xFF;
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
	for(i=0;i<_cnt;i++)
		AnBT_Uart1_Send_Char(data_to_send[i]);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
}
