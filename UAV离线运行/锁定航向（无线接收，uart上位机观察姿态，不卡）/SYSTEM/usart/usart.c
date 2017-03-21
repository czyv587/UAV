#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//end
//////////////////////////////////////////////////////////////////

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  

  
void USART1_IRQHandler(void)
{
	u8 res;	
#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
	OSIntEnter();    
#endif
	if(USART1->SR&(1<<5))//接收到数据
	{	 
		res=USART1->DR; 
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}else //还没收到0X0D
			{	
				if(res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=res;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}  		 									     
	}
#ifdef OS_CRITICAL_METHOD 	//如果OS_CRITICAL_METHOD定义了,说明使用ucosII了.
	OSIntExit();  											 
#endif
} 
#endif		
								 
//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
//CHECK OK
//091209
void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F;//IO状态设置
	GPIOA->CRH|=0X000008B0;//IO状态设置
		  
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
#if EN_USART1_RX		  //如果使能了接收
	//使能接收中断
	USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART1_IRQChannel,2);//组2，最低优先级 
#endif
}
/*void uart_init1(u32 bound){
    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
     //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

   //Usart1 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
  
   //USART 初始化设置
   
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
   

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
   
    USART_Cmd(USART1, ENABLE);                    //使能串口 

}					 */

void AnBT_Uart1_Send_Char(u8 can)	   //Send_Char
{
	USART1->DR=can;
	while((USART1->SR&0X40)==0);//等待发送结束	 
}
void Send_Str(unsigned char *s)
{
	unsigned char *p;
	p=s;
	while(*p!='\0')
	{
		USART1->DR=*p++;
		while((USART1->SR&0X40)==0);//等待发送结束		
	}
}
/*void AnBT_Uart1_Send_Char(unsigned char ascii_code) 		//圆点博士:发送一个字符
{
	USART_SendData(USART1,ascii_code);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}								//圆点博士:等待直到发送完成
}		*/

void AnBT_Uart1_Send_String(unsigned char* str_buf , unsigned char str_len)		//圆点博士:发送一个指定长度的字符串
{
	unsigned char i;
	if(str_len>AnBT_COM_Buf_Length) str_len=AnBT_COM_Buf_Length;
	AnBT_Uart1_Send_Char(13);																	//圆点博士:发送回车字符
// 	AnBT_Uart1_Send_Char(':');  															//圆点博士:发送字符:
  for(i=0;i<str_len;i++) AnBT_Uart1_Send_Char(str_buf[i]); 	//圆点博士:发送字符:
// 	AnBT_Uart1_Send_Char('/');																//圆点博士:发送字符/
	AnBT_Uart1_Send_Char(13);																	//圆点博士:发送回车字符
}

/*void AnBT_Uart1_Send_Num(unsigned char number) 	//圆点博士:发送一个字符
{
	unsigned char num_low,num_high;
	num_low=number&0x0f;													//圆点博士:取数据低位
	num_high=(number&0xf0)>>4;										//圆点博士:取数据高位
	if(num_high<10)USART_SendData(USART1,num_high+48);
	else USART_SendData(USART1,num_high+55);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}								//圆点博士:等待直到发送完成
	if(num_low<10)USART_SendData(USART1,num_low+48);
	else USART_SendData(USART1,num_low+55);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){}								//圆点博士:等待直到发送完成
}

void AnBT_Uart1_Send_Nums(unsigned char* nums_buf , unsigned char nums_len)		//圆点博士:发送一个指定长度的字符串
{
	unsigned char i;
	if(nums_len>AnBT_COM_Buf_Length) nums_len=AnBT_COM_Buf_Length;
	AnBT_Uart1_Send_Char(13);																										//圆点博士:发送回车字符
	AnBT_Uart1_Send_Char(':');  																								//圆点博士:发送字符:
  for(i=0;i<nums_len;i++) AnBT_Uart1_Send_Num(nums_buf[nums_len-i-1]); 				//圆点博士:发送数字
	AnBT_Uart1_Send_Char('/');																									//圆点博士:发送字符/
}			  */

