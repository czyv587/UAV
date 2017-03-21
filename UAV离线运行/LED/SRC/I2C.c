#include "I2C.h"
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#define   uchar unsigned char
#define   uint unsigned int	
#define I2C_Direction_Transmitter 0
#define I2C_Direction_Receiver	  1
// Prototype statements for functions found within this file.
#define FALSE 0;
#define TRUE 1;


char  test=0; 				 //IIC用到

char CY=3;
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
/* 函数申明 -----------------------------------------------*/
/*void GPIO_Configuration(void);
void NVIC_Configuration(void);

void scib_echoback_init(void);
void scib_fifo_init(void);
void scib_xmit(int a);
void scib_msg(char *msg);

void Delay(Uint32 nTime);
void Delayms(Uint32 m); */
/* 变量定义 ----------------------------------------------*/


void DATA_printf(uchar *s,short temp_data)
{
	if(temp_data<0){
	temp_data=-temp_data;
    *s='-';
	}
	else *s=' ';
	*++s =temp_data/10000+0x30;
    temp_data=temp_data%10000;     //取余运算
	*++s =temp_data/1000+0x30;
    temp_data=temp_data%1000;     //取余运算
    *++s =temp_data/100+0x30;
    temp_data=temp_data%100;     //取余运算
    *++s =temp_data/10+0x30;
    temp_data=temp_data%10;      //取余运算
    *++s =temp_data+0x30; 	
}
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_delay(void)
{
		
   char i=300; //这里可以优化速度	，经测试最低到5还能写入     30
   while(i) 
   { 
     i--; 
   }  
}

void delay5ms(void)
{
		
   int i=15000;  //5000
   while(i) 
   { 
     i--; 
   }  
}
/*
char I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	DIR_IN;I2C_delay();
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
	DIR_OUT;
	SDA_L;
	//
	DIR_IN;I2C_delay();
	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
	DIR_OUT;
	SDA_L;
	I2C_delay();
	return TRUE;
}
void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
} 
void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}   
void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
} 
char I2C_RecvAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	//I2C_delay();
	DIR_IN;I2C_delay();
	//CY=SDA_read;
	if(SDA_read)
	{
      SCL_L;
	  I2C_delay();
      return FALSE;
	}
	DIR_OUT;
	SCL_L;
	I2C_delay();
	//return CY;
	return TRUE;
}
void I2C_SendByte(char SendByte) //数据从高位到低位//
{
    char i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      //else 
      if(!(SendByte&0x80))
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
unsigned char I2C_RadeByte(void)  //数据从高位到低位//
{ 
    char i=8;
    char ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	  SCL_H;
      I2C_delay();	
      DIR_IN;
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
      DIR_OUT;
    }
    SCL_L;
    return ReceiveByte;
} 
//ZRX          
//单字节写入*******************************************
char Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
	I2C_Start();
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress << 1 | I2C_Direction_Transmitter);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_RecvAck()){I2C_Stop(); return FALSE;}//I2C_RecvAck()
    I2C_SendByte(REG_Address );   //设置低起始地址   
    I2C_RecvAck();	
    I2C_SendByte(REG_data);
    I2C_RecvAck();   
    I2C_Stop(); 
    delay5ms();
    return TRUE;
}
char i2cWriteBuffer(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char len,unsigned char *data)		     //void
{
	int i;
	I2C_Start();
  	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress << 1 | I2C_Direction_Transmitter);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_RecvAck()){I2C_Stop(); return FALSE;}//I2C_RecvAck()
    I2C_SendByte(REG_Address );   //设置低起始地址   
    I2C_RecvAck();	
    for (i = 0; i < len; i++)
    {
    	I2C_SendByte(data[i]);
    	if(I2C_RecvAck())
    	{
    		I2C_Stop();
    		return FALSE;
    	}
    }
    I2C_Stop();
    delay5ms();
    return TRUE;
}
//单字节读取*****************************************
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{   unsigned char REG_data;     	
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress<<1 | I2C_Direction_Transmitter); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_RecvAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((char) REG_Address);   //设置低起始地址      
    I2C_RecvAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress<< 1 | I2C_Direction_Receiver);
    I2C_RecvAck();

	REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return TRUE;
	return REG_data;

}		
unsigned char i2cRead(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char len,unsigned char *buf)
{   //unsigned char REG_data;     	
	if(!I2C_Start())return FALSE;
    I2C_SendByte(SlaveAddress<<1 | I2C_Direction_Transmitter); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_RecvAck()){I2C_Stop();test=1; return FALSE;}
    I2C_SendByte((char) REG_Address);   //设置低起始地址      
    I2C_RecvAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress<< 1 | I2C_Direction_Receiver);
    I2C_RecvAck();
	while(len)
	{
		*buf= I2C_RadeByte();
		if(len==1)
    		I2C_NoAck();
    	else
    		I2C_Ack();
    	buf++;
        len--;
	}
    I2C_Stop();
    return TRUE;
	//return REG_data;
}*/						      
// Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scib_echoback_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

 	ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all =0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA = 1;
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;
	#if (CPU_FRQ_150MHZ)
	      ScibRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 37.5MHz.
	      ScibRegs.SCILBAUD    =0x00E7;
	#endif
	#if (CPU_FRQ_100MHZ)
      ScibRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
      ScibRegs.SCILBAUD    =0x0044;
	#endif
	ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

// Transmit a character from the SCI
void scib_xmit(int a)
{
    while (ScibRegs.SCICTL2.bit.TXRDY == 0) {}
    ScibRegs.SCITXBUF=a;

}

void scib_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scib_xmit(msg[i]);
        i++;
    }
}
void Uart1_Put_Buf(unsigned char *DataToSend , unsigned char data_num)
{
	unsigned char i=0;
	for(i=0;i<data_num;i++)
		{scib_xmit(DataToSend[i]);}
}
// Initalize the SCI FIFO
void scib_fifo_init()
{
    ScibRegs.SCIFFTX.all=0x8000;//复位
}

void InitSci_cGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

/* Configure SCI-C pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation
	
    EDIS;
}
void scic_fifo_init()
{
    ScicRegs.SCIFFTX.all=0xE040;
    ScicRegs.SCIFFRX.all=0x204f;
    ScicRegs.SCIFFCT.all=0x0;
}
void scic_echoback_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

 	ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all =0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA = 1;
	ScicRegs.SCICTL2.bit.RXBKINTENA =1;
	#if (CPU_FRQ_150MHZ)
	      ScicRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 37.5MHz.
	      ScicRegs.SCILBAUD    =0x00E7;
	#endif
	#if (CPU_FRQ_100MHZ)
      ScicRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
      ScicRegs.SCILBAUD    =0x0044;
	#endif
	ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}
void scic_xmit(int a)
{
    while (ScicRegs.SCIFFTX.bit.TXFFST != 0) {}
    ScicRegs.SCITXBUF=a;
}

void scic_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scic_xmit(msg[i]);
        i++;
    }
}

// Initalize the SCI FIFO
/*
********************************************************************************
** 函数名称 ： Delay(vu32 nCount)
** 函数功能 ： 延时函数
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/
 void Delay(unsigned int nCount)
{
  for(; nCount != 0; nCount--);
}

/*
********************************************************************************
** 函数名称 ： void Delayms(vu32 m)
** 函数功能 ： 长延时函数	 m=1,延时1ms
** 输    入	： 无
** 输    出	： 无
** 返    回	： 无
********************************************************************************
*/
 void Delayms(unsigned int m)
{
  Uint32 i;
  
  for(; m != 0; m--)	
       for (i=0; i<50000; i++);
}



void GPIO_Configuration(void)
{
    EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;  // SCL   GPIO
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;  // SDA
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;  // 上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // 
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;   // 输出
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;
    EDIS;
}     
