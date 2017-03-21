#ifndef I2C_H_
#define I2C_H_

void DATA_printf(unsigned char *s,short temp_data);
void I2C_delay(void);
void delay5ms(void);
/*char I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
char I2C_RecvAck(void); 	 //返回为:=1有ACK,=0无ACK
void I2C_SendByte(char SendByte); //数据从高位到低位//
unsigned char I2C_RadeByte(void);  //数据从高位到低位//
char Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);		     //void
char i2cWriteBuffer(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char len,unsigned char *data);		     //void
unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address);
unsigned char i2cRead(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char len,unsigned char *buf);*/
void scib_echoback_init();
void scib_xmit(int a);
//void scic_xmit(int a);
void scib_msg(char * msg);
void Uart1_Put_Buf(unsigned char *DataToSend , unsigned char data_num);
void scib_fifo_init();
void InitSci_cGpio();
void scic_fifo_init();
void scic_echoback_init();
void scic_xmit(int a);
void scic_msg(char * msg);
void Delay(unsigned int nCount);
void Delayms(unsigned int m);
void Init_MPU6050(void);
void READ_MPU6050(void);
void Send_data(unsigned char axis);
void GPIO_Configuration(void);
#endif /*I2C_H_*/
