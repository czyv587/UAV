#include "nrf24l01.h"
#include "24l01.h"
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//unsigned int rdata;

unsigned char tmp_buf2[32]={0X22};
unsigned char TX_ADDRESS[5]  = {0x34,0x43,0x10,0x10,0x01};//路由节点地址，更改2-5通道时改第一个地址即可
unsigned char RX_ADDRESS[5]  = {0x34,0x43,0x10,0x10,0x01};//路由节点地址，更改2-5通道时改第一个地址即可
unsigned char buf[5]={'1','d','a','4','5'};
unsigned char buf1[5]={'a','b','c','d','e'};
unsigned int a=5,c=5;
unsigned int b=5,d=5;
unsigned int num=0,num1=0;
unsigned int sta=0x55,sta1=0x55;

void delay_loop()
{
    long      i;
    for (i = 0; i <1000 ; i++) {}//1000000
}
void delay()
{
    long      i;
    for (i = 0; i <10 ; i++) {}//1000000
}
void error(void)
{
    asm("     ESTOP0");						// Test failed!! Stop!
    for (;;);
}
void spi_init()
{    
	SpiaRegs.SPICCR.all =0x0007;	   //0x47          // Reset on, rising edge, 16-bit char bits  
	SpiaRegs.SPICTL.all =0x000e;    //0x06		     // Enable master mode, normal phase,
                                                 // enable talk, and SPI int disabled.
	SpiaRegs.SPIBRR =0x0004;									
    SpiaRegs.SPICCR.all =0x0087;		//0xc7         // Relinquish SPI from Reset   
    SpiaRegs.SPIPRI.bit.FREE = 1;                // Set so breakpoints don't disturb xmission
}
void led_init()
{
	EALLOW;
	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;  // CE
	GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;  // 上拉电阻
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;   // 输出
	EDIS;
}
//初始化24L01的IO口
void GPIO_Conf_SPI(void)
{
    EALLOW;
	/*GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;  // SCN
	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;  // 上拉电阻
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;   // 输出
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;  // SCN
	GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;  // 上拉电阻
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;   // 输出*/
    
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;  // CE
	GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;  // 上拉电阻
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;   // 输出
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;  // IRQ
	GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0;  // 上拉电阻
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;   // 输入
    GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 0;  //与系统时钟同步
    EDIS;
}     
void spi_fifo_init()										
{
// Initialize SPI FIFO registers
    SpiaRegs.SPIFFTX.all=0xE040;
    SpiaRegs.SPIFFRX.all=0x204f;
    SpiaRegs.SPIFFCT.all=0x0;
}  
void NRF24L01_Init(void)
{
	
	//spi_fifo_init();  
	//针对NRF的特点修改SPI的设置
	//SPI1_SetSpeed(SPI_SPEED_4); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）    // 不用每次都设置SPI速度
	InitSpiaGpio();
	GPIO_Conf_SPI();
	spi_init();    		//初始化SPI	
	NRF24L01_CE_L; 			//使能24L01
	//NRF24L01_SCN_H;			//SPI片选取消	 		 	 
}
void spi_xmit(Uint16 a)
{
	//if(SpiaRegs.SPISTS.bit.BUFFULL_FLAG==1)
       SpiaRegs.SPITXBUF=a;
    //while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG==0)
}    

//SPI 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
unsigned char SPI_ReadWriteByte(unsigned char TxData)
{
	unsigned char rdata=0;
	if(SpiaRegs.SPISTS.bit.BUFFULL_FLAG==0)
    SpiaRegs.SPITXBUF=(TxData<<8);
    while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG==1);
    if(SpiaRegs.SPISTS.bit.INT_FLAG==1)
    rdata = SpiaRegs.SPIRXBUF;
    return rdata;
	/*spi_xmit(TxData);
     // Wait until data is received
     while(SpiaRegs.SPIFFRX.bit.RXFFST !=1) { } 			
     rdata = SpiaRegs.SPIRXBUF;
     return rdata;*/
}
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
unsigned char NRF24L01_Write_Reg(unsigned char reg,unsigned char value)
{
	char status;	
   	//NRF24L01_SCN_L;                 //使能SPI传输
  	status =SPI_ReadWriteByte(reg);//发送寄存器号 
  	SPI_ReadWriteByte(value);      //写入寄存器的值
  	//NRF24L01_SCN_H;                 //禁止SPI传输	   
  	return(status);       			//返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
unsigned char NRF24L01_Read_Reg(unsigned char reg)
{
	char reg_val;	    
// 	NRF24L01_SCN_L;          //使能SPI传输		
  	SPI_ReadWriteByte(reg);   //发送寄存器号
  	SPI_ReadWriteByte(0XFF);
  	reg_val=SPI_ReadWriteByte(0XFF);//读取寄存器内容
//  	NRF24L01_SCN_H;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
unsigned char NRF24L01_Read_Buf(unsigned char reg,unsigned char *pBuf,unsigned char len)
{
	unsigned char status,u8_ctr;       
  	//NRF24L01_SCN_L;           //使能SPI传输
  	status=SPI_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   
  	SPI_ReadWriteByte(0XFF);	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI_ReadWriteByte(0XFF);//读出数据
  	//NRF24L01_SCN_H;       //关闭SPI传输
  	return status;        //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
unsigned char NRF24L01_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char len)
{
	unsigned char status,u8_ctr;	    
 	//NRF24L01_SCN_L;          //使能SPI传输
  	status = SPI_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI_ReadWriteByte(*pBuf++); //写入数据	 
  	//NRF24L01_SCN_H;       //关闭SPI传输
  	return status;          //返回读到的状态值
}				   
 
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
unsigned char VAL=0;
unsigned char sss=0;
unsigned char NRF24L01_RxPacket(unsigned char *rxbuf,unsigned char*txbuf)
{
	unsigned char sta;	 							   
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值   		
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//发送完成
	{	 	 
		VAL=NRF24L01_Read_Reg(0X60);
	//	NRF24L01_Write_Buf(W_ACK_PYLOD,txbuf,VAL);			// 发送ACK 数据包
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,VAL);			// 读取已接收数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);					//清除RX FIFO寄存器 
		return 	128;
	}   
	return 1;//没收到任何数据
}				    
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
unsigned char NRF24L01_TxPacket(unsigned char *txbuf)
{
	//u8 sta;
 	//SPIx_SetSpeed(SPI_SPEED_8);//spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）  
 	 
 	
	NRF24L01_CE_L;delay();//要不要nRF24L01_Flush_TX_FIFO();// 清空 FIFO
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,0x12);//写数据到TX BUF  32个字节
 	NRF24L01_CE_H;delay();//启动发送	   
 	
	while(NRF24L01_IRQ!=0);//等待发送完成
	delay_loop();
	sta1=NRF24L01_Read_Reg(NRF_FIFO_STATUS);delay();  //读取状态寄存器的值	
	sta=NRF24L01_Read_Reg(STATUS);delay();  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta);delay(); //清除TX_DS或MAX_RT中断标志
	
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}

//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了
unsigned char FF1=0,DYN=0; 		   
void NRF24L01_RX_Mode(void)
{
	/*NRF24L01_CE_L;	  
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(unsigned char*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(unsigned char*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
			  
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0X1A);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);	     //设置RF通信频率		
	
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_FEATURE , NRD_EN_DPL | NRF_EN_ACK_PAYLOAD);
    if(NRF24L01_Read_Reg(NRF_FEATURE)== 0x00 && NRF24L01_Read_Reg(NRF_DYNPD) == 0x00)
	{
        NRF24L01_Write_Reg(NRF_ACTIVATE, 0x73);
    }
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_FEATURE , NRD_EN_DPL | NRF_EN_ACK_PAYLOAD);
    NRF24L01_Write_Reg(NRF_WRITE_REG|NRF_DYNPD, NRF_DPL_P0 | NRF_DPL_P1| NRF_DPL_P2| NRF_DPL_P3| NRF_DPL_P4| NRF_DPL_P5);
    	    	  
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x27);//设置TX发射参数,0db增益,1Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	NRF24L01_CE_H; //CE为高,进入接收模式 */
  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x00);    //使能通道0的自动应答  
  	NRF24L01_Write_Reg(WRITE_REG + EN_RXADDR, 0x00);    // 关闭接收通道0  
	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0X00);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
}						 
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE_L;	    
  	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(unsigned char*)TX_ADDRESS,5);delay();//写TX节点地址
  	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(unsigned char*)RX_ADDRESS,5);delay(); //设置TX节点地址,主要为了使能ACK

  	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x00);delay();     //使能通道0的自动应答    
  	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x00);delay(); //使能通道0的接收地址  
  	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0x00);delay();//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);delay();       //设置RF通道为40
  	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x07);delay();  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	
  	NRF24L01_Write_Reg(WRITE_REG+CONFIG_24l01,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF24L01_CE_H;delay();//CE为高,10us后启动发送
}		  

//检测24L01是否存在
//返回值:0，成功;1，失败	
unsigned char NRF24L01_Check(void)
{
	unsigned char buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	unsigned char i;
	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	delay_loop();
	NRF24L01_Read_Buf(TX_ADDR,buf1,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf1[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}	
