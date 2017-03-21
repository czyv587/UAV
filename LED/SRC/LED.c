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
//#include  "math.h"
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

extern unsigned char mpu6050_buffer[14];
//************************************
/*ģ��IIC�˿�������붨��*/
#define SCL_H         GpioDataRegs.GPASET.bit.GPIO12 =1;
#define SCL_L         GpioDataRegs.GPACLEAR.bit.GPIO12 =1;
   
#define SDA_H         GpioDataRegs.GPASET.bit.GPIO13 =1;
#define SDA_L         GpioDataRegs.GPACLEAR.bit.GPIO13 =1;

#define DIR_OUT 	  GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;//���
#define DIR_IN 	  	  GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;//����
//#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GpioDataRegs.GPADAT.bit.GPIO13
// Select the example to compile in.  Only one example should be set as 1
// the rest should be set as 0.
extern struct DATA_XYZ ACC;
extern struct DATA_XYZ last_ACC;
extern struct DATA_XYZ GYR_RATE;
short T_X,T_Y,T_Z,T_T;		 //X,Y,Z�ᣬ�¶�
short T_X1,T_Y1,T_Z1;		 //X,Y,Z�ᣬ�¶�
int BUF[20];       //�������ݻ�����
int BUF1[20];       //�������ݻ�����
unsigned char BUF_send[18];       //�������ݻ�����
unsigned int BUF_1[10]={0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
unsigned int   readdata0[]={0,0,0,0,0,0,0,0,0};
extern unsigned int readdata[];
extern Uint32 Time3on;
unsigned char buf_test[10]={'0'};
unsigned char buf_test1[2]={0x00,0x55};
int k=0,check=3,sign=0;
char s=0x00;
long pwm_num1=0,pwm_num2=0,pwm_num3=0;
Uint16 LoopCount;
Uint16 ErrorCount;
unsigned int temp_out=0;
unsigned char TX_DATA[6];  	 //��ʾ�ݻ�����
int iii;
//**********nrf24l01**********************
extern unsigned char tmp_buf2[32];
extern unsigned char TX_ADDRESS[5];//·�ɽڵ��ַ������2-5ͨ��ʱ�ĵ�һ����ַ����
extern unsigned char RX_ADDRESS[5];//·�ɽڵ��ַ������2-5ͨ��ʱ�ĵ�һ����ַ����
extern unsigned char buf[5];
extern unsigned char buf1[5];
extern unsigned int a,c;
extern unsigned int b,d;
extern unsigned int num,num1;
extern unsigned int sta,sta1;
// ����MPU6050�ڲ���ַ
//****************************************
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG_MPU6050			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
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

#define	PWR_MGMT_1		0x6B	//��Դ����������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	MPU6050_Addr   0xD0	  //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�
/*
********************************************************************************
** �������� �� WWDG_IRQHandler(void)
** �������� �� ������ǰ�����ж�
** ��    ��	�� ��
** ��    ��	�� ��
** ��    ��	�� ��
********************************************************************************
*/ 

//��ʼ��MPU6050��������Ҫ��ο�pdf�����޸�************************
 //********���ڷ�������***************************************
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
   	BUF[3]=hw_Single_Write(0x68,PWR_MGMT_1, 0x00);Delayms(5);	//�������״̬
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
//   InitScibGpio();
   InitI2CGpio();
	InitI2C();
	InitECapGpio();
   InitECapture1();
   InitECapture2();
   InitECapture3();
   InitECapture4();
   InitECapture5();
   InitECapture6();
   NRF24L01_Init();
   DINT;
   InitPieCtrl();
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();
	LoopCount = 0;
    ErrorCount = 0;
    a=NRF24L01_Check();
    while(a);
    NRF24L01_TX_Mode();
    delay();
    //NRF24L01_Read_Buf(TX_ADDR,buf1,5);delay();
    d=NRF24L01_Read_Reg(RF_CH);delay();
//    scib_fifo_init();	   // Initialize the SCI FIFO
//    scib_echoback_init();  // Initalize SCI for echoback
	DELAY_US(10000);
	MPU6050_INIT();Delayms(100);
	MPU6050_INIT();Delayms(100);
//	GPIO_Configuration();//Ӧ�ò�Ҫ
	  read_back();
	Get_OFFSET(500);
	
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
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;//�ⲿ�ж�
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
    k++;
   // Acknowledge this interrupt to receive more interrupts from group 1
    MPU6050_READ();
	MPU6050_CONVENT();
	if((last_ACC.X==ACC.X)&&(last_ACC.Y==ACC.Y)&&(last_ACC.Z==ACC.Z))
	{
		GpioDataRegs.GPASET.bit.GPIO8 = 1;
	}
	ACC_SMOOTH(10);  // ACC ƽ���˲�
	MahonyIMUupdate(GYR_F.X,GYR_F.Y,GYR_F.Z,(float)ACC_AVG.X,(float)ACC_AVG.Y,(float)ACC_AVG.Z);
	data_buf();
	b=NRF24L01_TxPacket(BUF_send);
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
//		}//2ms���pwm
	
	control();
	
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
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0; // GPIO8 = GPIO8 ͨ�õ� IO
	GpioCtrlRegs.GPADIR.bit.GPIO8 = 1; // GPIO8 ����Ϊ�����
	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0; // GPIO9 = GPIO9 ͨ�õ� IO
	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1; // GPIO9 ����Ϊ�����
	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0; // GPIO10 = GPIO10 ͨ�õ� IO
	GpioCtrlRegs.GPADIR.bit.GPIO10 = 1; // GPIO20 ����Ϊ�����
	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0; // GPIO11= GPIO11 ͨ�õ� IO
	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1; // GPIO11 ����Ϊ�����
	EDIS;
	GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;
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
	BUF_send[0]=((int)(Roll*100) >> 8) & 0xFF;
	BUF_send[1]=(int)(Roll*100)&0xff;
	BUF_send[2]=((int)(Pitch*100) >> 8) & 0xFF;
	BUF_send[3]=(int)(Pitch*100)&0xff;
	BUF_send[4]=((int)(Yaw*100) >> 8) & 0xFF;
	BUF_send[5]=(int)(Yaw*100)&0xff;
	BUF_send[6]=((int)(ACC.X/1024)>> 8)& 0xFF;///8192
	BUF_send[7]=(int)(ACC.X/1024)& 0xFF;
	BUF_send[8]=((int)(ACC.Y/1024)>> 8)& 0xFF;
	BUF_send[9]=(int)(ACC.Y/1024)& 0xFF;
	BUF_send[10]=((int)(ACC.Z/1024)>> 8)& 0xFF;
	BUF_send[11]=(int)(ACC.Z/1024)& 0xFF;
	BUF_send[12]=((int)(GYR.X*0.061) >> 8) & 0xFF;
	BUF_send[13]=(int)(GYR.X*0.061)&0xff;
	BUF_send[14]=((int)(GYR.Y*0.061) >> 8) & 0xFF;
	BUF_send[15]=(int)(GYR.Y*0.061)&0xff;
	BUF_send[16]=((int)(GYR.Z*0.061) >> 8) & 0xFF;
	BUF_send[17]=(int)(GYR.Z*0.061)&0xff;
//	for(iii=0;iii<18;iii++)
//		BUF_send[iii]=iii;
}
//===========================================================================
// No more.
//===========================================================================
