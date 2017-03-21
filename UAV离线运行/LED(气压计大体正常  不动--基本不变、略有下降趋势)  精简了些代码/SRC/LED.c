#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "define.h"
#include "I2C.h"
#include "MPU6050.h"
#include "control.h"
#include "imu.h"
#include "fast_math.h"
#include "data_transfer.h"
#include "hardware_I2C.h"
#include "pwm.h"
#include "capture.h"
#include "nrf24l01.h"
#include "24l01.h"
#include  "math.h"
#include "MS5611.h"
//"${CG_TOOL_ROOT}/include"     "${XDAIS_CG_ROOT}/packages/ti/xdais"
extern struct DATA_XYZ_F GYR_F;
extern struct DATA_XYZ ACC_AVG;  
extern struct DATA_XYZ ACC_OFFSET;
extern struct DATA_XYZ GYR_OFFSET;
extern struct DATA_XYZ HMC;
extern struct DATA_XYZ GYR;
extern float roll,pitch;
extern int Motor1,Motor2,Motor3,Motor4; 
extern struct DATA_XYZ ACC;
extern float Pitch,Roll,Yaw;
extern float Roll_core_kp,Roll_core_ki,Roll_core_kd;
extern float Yaw_core_kp,Yaw_core_ki,Yaw_core_kd;
extern unsigned char mpu6050_buffer[14];
//************************************
/*模拟IIC端口输出输入定义*/
#define SCL_H         GpioDataRegs.GPASET.bit.GPIO12 =1;
#define SCL_L         GpioDataRegs.GPACLEAR.bit.GPIO12 =1;
   
#define SDA_H         GpioDataRegs.GPASET.bit.GPIO13 =1;
#define SDA_L         GpioDataRegs.GPACLEAR.bit.GPIO13 =1;

#define DIR_OUT 	  GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;//输出
#define DIR_IN 	  	  GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;//输入
//#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GpioDataRegs.GPADAT.bit.GPIO13
// Select the example to compile in.  Only one example should be set as 1
// the rest should be set as 0.
extern struct DATA_XYZ ACC;
extern struct DATA_XYZ last_ACC;
extern struct DATA_XYZ GYR_RATE;
//unsigned char VAL=5;
short T_X,T_Y,T_Z,T_T;		 //X,Y,Z轴，温度
short T_X1,T_Y1,T_Z1;		 //X,Y,Z轴，温度
int BUF[20];       //接收数据缓存区
int BUF1[20];       //接收数据缓存区
unsigned char BUF_send[26];       //接收数据缓存区
unsigned char BUF_recv[18];       //接收数据缓存区0x55,0x55,0x55,0x55,0x55
int BUF_recv1[18]={0x00};
unsigned char BUF_send1[6];
int BUF_send2[6];
unsigned int BUF_1[10]={0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
unsigned int   readdata0[]={0,0,0,0,0,0,0,0,0};
extern unsigned int readdata[];
extern Uint32 Time3on;
unsigned char buf_test[10]={'0'};
unsigned char buf_test1[2]={0x00,0x55};
int k=0,k1=0,k2=0,check=3,sign=0,b1=5;
char s=0x00;
long pwm_num1=0,pwm_num2=0,pwm_num3=0;
int YAW_X=0,YAW_Y=0,YAW_Z=0;
float YAW_AVG_X=0,YAW_AVG_Y=0,YAW_AVG_Z=0;
float YAW_Angle=0;
int YAW_INIT_X,YAW_INIT_Y,YAW_INIT_Z;
extern float YAW_INIT;
//Uint16 LoopCount;
//Uint16 ErrorCount;
unsigned int temp_out=0;
int temp=0;
extern long dd;
unsigned char TX_DATA[6];  	 //显示据缓存区
int iii,jj;
//**********nrf24l01**********************
extern unsigned char tmp_buf2[32];
extern unsigned char TX_ADDRESS[5];//路由节点地址，更改2-5通道时改第一个地址即可
extern unsigned char RX_ADDRESS[5];//路由节点地址，更改2-5通道时改第一个地址即可
extern unsigned char buf[5];
extern unsigned char buf1[5];
extern unsigned int a,c,e;
extern unsigned int b,d;
extern unsigned int num,num1;
extern unsigned int sta,sta1;
// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG_MPU6050			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	MPU6050_Addr   0xD0	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改
/*
********************************************************************************
** 函数名称 ： WWDG_IRQHandler(void)
** 函数功能 ： 窗口提前唤醒中断
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/ 

//初始化MPU6050，根据需要请参考pdf进行修改************************
 //********串口发送数据***************************************
 void Send_data(unsigned char axis)
 {unsigned char i;
  scib_xmit(axis);
  scib_xmit(':');
  for(i=0;i<6;i++)scib_xmit(TX_DATA[i]);
  scib_xmit(' ');
  scib_xmit(' ');
 }
 void Send_msg()
 {unsigned char i;
  //scib_msg(* msg);
  scib_xmit(':');
  for(i=0;i<6;i++)scib_xmit(TX_DATA[i]);
  scib_xmit(' ');
  scib_xmit(' ');
 }
void Init_MPU6050(void)
{
   	BUF[3]=hw_Single_Write(0x68,PWR_MGMT_1, 0x00);Delayms(5);	//解除休眠状态
	BUF[4]=hw_Single_Write(0x68,SMPLRT_DIV, 0x07);Delayms(5);
	BUF[5]=hw_Single_Write(0x68,CONFIG_MPU6050, 0x06);Delayms(5);
	BUF[6]=hw_Single_Write(0x68,GYRO_CONFIG, 0x18);Delayms(5);
	BUF[7]=hw_Single_Write(0x68,ACCEL_CONFIG, 0x08);Delayms(5);
}
void configtestled(void);
void 	InitI2C(void);
void read_back(void);
void configexgpio(void);
void data_buf(void);
interrupt void ISRTimer0(void);
interrupt void ISRExint1(void);

void main(void)
{
   InitSysCtrl();
   InitScibGpio();
   InitI2CGpio();
   InitI2C();
   InitECapGpio();
   InitECapture1();
   InitECapture2();
   InitECapture3();
   InitECapture4();
   InitECapture5();
   InitECapture6();

   DINT;
   InitPieCtrl();
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();
	/*LoopCount = 0;
    ErrorCount = 0;*/
   MS5611_init();
   temp=dd;
//   while(1);
/*
    NRF24L01_Init();
    a=NRF24L01_Check();
    while(a);
    //NRF24L01_TX_Mode();
 //   c=NRF24L01_Read_Reg(CONFIG_24l01);
    NRF24L01_RX_Mode();
    delay();
 //   e=NRF24L01_Read_Reg(CONFIG_24l01);delay();
    while(b1!=0)
    {b1=NRF24L01_RxPacket(BUF_recv);}
    //PID赋值
    BUF_recv1[0]=(int)(BUF_recv[0]);BUF_recv1[1]=(int)(BUF_recv[1]);
    BUF_recv1[2]=(int)(BUF_recv[2]);BUF_recv1[3]=(int)(BUF_recv[3]);
    BUF_recv1[4]=(int)(BUF_recv[4]);BUF_recv1[5]=(int)(BUF_recv[5]);
    BUF_recv1[6]=(int)(BUF_recv[6]);BUF_recv1[7]=(int)(BUF_recv[7]);
    BUF_recv1[8]=(int)(BUF_recv[8]);BUF_recv1[9]=(int)(BUF_recv[9]);
    BUF_recv1[10]=(int)(BUF_recv[10]);BUF_recv1[11]=(int)(BUF_recv[11]);
    BUF_recv1[12]=(int)(BUF_recv[12]);BUF_recv1[13]=(int)(BUF_recv[13]);
    BUF_recv1[14]=(int)(BUF_recv[14]);BUF_recv1[15]=(int)(BUF_recv[15]);
    BUF_recv1[16]=(int)(BUF_recv[16]);BUF_recv1[17]=(int)(BUF_recv[17]);
    delay();delay();delay();
    NRF24L01_TX_Mode();
    delay();
    //NRF24L01_Read_Buf(TX_ADDR,buf1,5);delay();
    d=NRF24L01_Read_Reg(RF_CH);delay();
*/

//    scib_fifo_init();	   // Initialize the SCI FIFO
//    scib_echoback_init();  // Initalize SCI for echoback
	DELAY_US(10000);
	MPU6050_INIT();Delayms(100);
	MPU6050_INIT();Delayms(100);
//	GPIO_Configuration();//应该不要
	  read_back();
	Get_OFFSET(500);
	Get_YAW_INIT(20);
	PID_INIT();
	
	configexgpio();
	pwm_init();
	
	InitCpuTimers();
	configtestled();
	ConfigCpuTimer(&CpuTimer0, 150, 2500);
	StartCpuTimer0();
	EALLOW;  // This is needed to write to EALLOW protected registers
   	PieVectTable.TINT0 = &ISRTimer0;
   	PieVectTable.XINT1 = &ISRExint1;
   	PieVectTable.ECAP1_INT = &ecap1_isr;
    PieVectTable.ECAP2_INT = &ecap2_isr;
    PieVectTable.ECAP3_INT = &ecap3_isr;
    PieVectTable.ECAP4_INT = &ecap4_isr;
    PieVectTable.ECAP5_INT = &ecap5_isr;
    PieVectTable.ECAP6_INT = &ecap6_isr;
   	EDIS;    // This is needed to disable write to EALLOW protected registers
   	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
	IER |= M_INT1;
	IER |= M_INT4;
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;//外部中断
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx4 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx5 = 1;
    PieCtrlRegs.PIEIER4.bit.INTx6 = 1;
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
	
	while(1)
    {
   	   // GpioDataRegs.GPASET.bit.GPIO10 = 1;
   	   //PWM_CH1(5000);
   	  // PWM_CH2(100);
   	   //motor_start();
		//buf
		//delay_loop();delay_loop();
    }
}
interrupt void ISRTimer0(void)
{
    k++;k1++;k2++;//k是每六次采样一次偏航角；k1是前200次不进行pwm调整偏航角；k2是每四次采样一次气压计
   // Acknowledge this interrupt to receive more interrupts from group 1
    MPU6050_READ();
	MPU6050_CONVENT();
	if(k==6)
	{
		delay();delay();delay();
		delay();delay();delay();
		hw_i2cRead(0x1E,0x03,6,BUF_send1);//读取HMC5883L  xyz  方向的数据
		YAW_X=BUF_send1[0]<<8 | BUF_send1[1];
		YAW_Z=BUF_send1[2]<<8 | BUF_send1[3];
		YAW_Y=BUF_send1[4]<<8 | BUF_send1[5];
//		YAW_Angle= atan2((double)YAW_Y,(double)YAW_X)*(180/3.14159265);//+180
		BUF_send2[0]=(int)(BUF_send1[0]);BUF_send2[1]=(int)(BUF_send1[1]);
		BUF_send2[2]=(int)(BUF_send1[2]);BUF_send2[3]=(int)(BUF_send1[3]);
		BUF_send2[4]=(int)(BUF_send1[4]);BUF_send2[5]=(int)(BUF_send1[5]);
		YAW_SMOOTH(10); //YAW 平滑滤波
		k=0;
	}
	if(k2==4)//k2是第四次采样一次气压计的温度
	{
		delay();delay();delay();
		delay();delay();delay();
		readTemperatureRequestPressure();
	//	calculatePressureAltitude();
//		k2=0;
	}
	if(k2==8)//k2是第八次采样一次气压计的压力
	{
		delay();delay();delay();
		delay();delay();delay();
		readPressureRequestTemperature();
		//		readPressureRequestPressure();
//		calculatePressureAltitude();
		k2=0;
	}
	if((last_ACC.X==ACC.X)&&(last_ACC.Y==ACC.Y)&&(last_ACC.Z==ACC.Z))
	{
		GpioDataRegs.GPASET.bit.GPIO8 = 1;
	}
	ACC_SMOOTH(10);  // ACC 平滑滤波
//	MahonyIMUupdate(GYR_F.X,GYR_F.Y,GYR_F.Z,(float)ACC_AVG.X,(float)ACC_AVG.Y,(float)ACC_AVG.Z);
	MahonyAHRSupdate(GYR_F.X,GYR_F.Y,GYR_F.Z,(float)ACC_AVG.X,(float)ACC_AVG.Y,(float)ACC_AVG.Z, YAW_AVG_X, YAW_AVG_Y, YAW_AVG_Z);
	if(k1<200)
		Yaw=YAW_INIT;//因为融合的yaw都是从0渐变到当前的的偏航角，所以前200次就直接等于初始的偏航角，防止刚开始就开始计算pwm调整偏航角
	if(k1==200)
	{
	//	Yaw= atan2((double)YAW_AVG_Y,(double)YAW_AVG_X)*(180/3.14159265);
		k1=199;
	}
	//	MahonyAHRSupdate(GYR_F.X,GYR_F.Y,GYR_F.Z,(float)ACC_AVG.X,(float)ACC_AVG.Y,(float)ACC_AVG.Z, YAW_AVG_X, YAW_AVG_Y, YAW_AVG_Z);
	data_buf();
//	b=NRF24L01_TxPacket(BUF_send);

//	Data_Send_Status(Pitch,Roll,Yaw,0,0);
//	Data_Send_Senser();
//	if(sign>2)
//		PWM_CH1((int)(Time3on*11.747));PWM_CH2((int)(Time3on*11.747));PWM_CH3((int)(Time3on*11.747));PWM_CH4((int)(Time3on*11.747));
	
//	if(sign==2)
//		{
			pwm_num1=Time3on;
			pwm_num2=286723-Time3on;
			pwm_num3=pwm_num2/12.412;
		//	PWM_CH3(pwm_num3);
//			PWM_CH1(pwm_num3);PWM_CH2(pwm_num3);PWM_CH3(pwm_num3);PWM_CH4(pwm_num3);		//	PWM_CH1(60);PWM_CH2(60);PWM_CH3(60);PWM_CH4(60);
		//	sign=1;
//		}//2ms最高pwm
	
//	control();
//	control1();
	control2();
	GpioDataRegs.GPATOGGLE.bit.GPIO9 = 1;

    last_ACC.X=ACC.X;last_ACC.Y=ACC.Y;last_ACC.Z=ACC.Z;
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    CpuTimer0Regs.TCR.bit.TIF=1;
    CpuTimer0Regs.TCR.bit.TRB=1;
}
interrupt void ISRExint1(void)
{
	sign=sign+1;
	//motor_start();
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
void configtestled(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0; // GPIO8 = GPIO8 通用的 IO
	GpioCtrlRegs.GPADIR.bit.GPIO8 = 1; // GPIO8 配置为输出口
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0; // GPIO9 = GPIO9 通用的 IO
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1; // GPIO9 配置为输出口
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0; // GPIO10 = GPIO10 通用的 IO
	GpioCtrlRegs.GPADIR.bit.GPIO10 = 1; // GPIO20 配置为输出口
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0; // GPIO11= GPIO11 通用的 IO
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1; // GPIO11 配置为输出口
	EDIS;
	GpioDataRegs.GPASET.bit.GPIO8 = 1;
	GpioDataRegs.GPASET.bit.GPIO9 = 1;
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
	GpioDataRegs.GPASET.bit.GPIO11 = 1;
}
void read_back(void)
{
	BUF_1[0]=hw_i2cRead_Single(0x68,MPU6050_RA_PWR_MGMT_1,1);Delayms(5);
	BUF_1[1]=hw_i2cRead_Single(0x68,MPU6050_RA_GYRO_CONFIG,1);Delayms(5);
	BUF_1[2]=hw_i2cRead_Single(0x68,MPU6050_RA_ACCEL_CONFIG,1);Delayms(5);
	BUF_1[3]=hw_i2cRead_Single(0x68,MPU6050_RA_CONFIG,1);Delayms(5);
	BUF_1[4]=hw_i2cRead_Single(0x68,MPU6050_RA_USER_CTRL,1);Delayms(5);
	BUF_1[5]=hw_i2cRead_Single(0x68,MPU6050_RA_INT_PIN_CFG,1);Delayms(5);
}
void configexgpio(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;
	GpioCtrlRegs.GPAQSEL1.bit.GPIO13= 0;
	GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 13;
	XIntruptRegs.XINT1CR.bit.POLARITY= 1;
	XIntruptRegs.XINT1CR.bit.ENABLE = 1;
	EDIS;
}
void data_buf(void)
{
	Roll=Roll*100;
	BUF_send[0]=((int)(Roll) >> 8) & 0xFF;
	BUF_send[1]=(int)(Roll)&0xff;
	Pitch=Pitch*100;
	BUF_send[2]=((int)(Pitch) >> 8) & 0xFF;
	BUF_send[3]=(int)(Pitch)&0xff;
	Yaw=Yaw*100;
	BUF_send[4]=((int)(Yaw) >> 8) & 0xFF;
	BUF_send[5]=(int)(Yaw)&0xff;
	BUF_send[6]=((int)(ACC_AVG.X)>> 8)& 0xFF;///8192
	BUF_send[7]=(int)(ACC_AVG.X)& 0xFF;
	BUF_send[8]=((int)(ACC_AVG.Y)>> 8)& 0xFF;
	BUF_send[9]=(int)(ACC_AVG.Y)& 0xFF;
	BUF_send[10]=((int)(ACC_AVG.Z)>> 8)& 0xFF;
	BUF_send[11]=(int)(ACC_AVG.Z)& 0xFF;
	BUF_send[12]=((int)(GYR_F.X) >> 8) & 0xFF;
	BUF_send[13]=(int)(GYR_F.X)&0xff;
	BUF_send[14]=((int)(GYR_F.Y) >> 8) & 0xFF;
	BUF_send[15]=(int)(GYR_F.Y)&0xff;
	BUF_send[16]=((int)(GYR_F.Z) >> 8) & 0xFF;
	BUF_send[17]=(int)(GYR_F.Z)&0xff;
	BUF_send[18]=((int)(Motor1) >> 8) & 0xFF;
	BUF_send[19]=(int)(Motor1)&0xff;
	BUF_send[20]=((int)(Motor2) >> 8) & 0xFF;
	BUF_send[21]=(int)(Motor2)&0xff;
	BUF_send[22]=((int)(Motor3) >> 8) & 0xFF;
	BUF_send[23]=(int)(Motor3)&0xff;
	BUF_send[24]=((int)(Motor4) >> 8) & 0xFF;
	BUF_send[25]=(int)(Motor4)&0xff;
//	for(iii=0;iii<18;iii++)
//		BUF_send[iii]=iii;
}
//===========================================================================
// No more.
//===========================================================================

