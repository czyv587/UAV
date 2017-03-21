#ifndef NRF24L01_H_
#define NRF24L01_H_

#define NRF24L01_IRQ           GpioDataRegs.GPBDAT.bit.GPIO41
//#define NRF24L01_SCN_H         GpioDataRegs.GPBSET.bit.GPIO57 =1;
//#define NRF24L01_SCN_L         GpioDataRegs.GPBCLEAR.bit.GPIO57 =1;
#define NRF24L01_CE_H         GpioDataRegs.GPBSET.bit.GPIO40 =1;
#define NRF24L01_CE_L         GpioDataRegs.GPBCLEAR.bit.GPIO40 =1;
#define LED_H         GpioDataRegs.GPBSET.bit.GPIO60 =1;
#define LED_L         GpioDataRegs.GPBCLEAR.bit.GPIO60 =1;

void delay_loop();
void delay();
void error(void);
void spi_init();
void led_init();
void GPIO_Conf_SPI(void);
void spi_fifo_init();
void NRF24L01_Init(void);
void spi_xmit(unsigned int a);
unsigned char SPI_ReadWriteByte(unsigned char TxData);
unsigned char NRF24L01_Write_Reg(unsigned char reg,unsigned char value);
unsigned char NRF24L01_Read_Reg(unsigned char reg);
unsigned char NRF24L01_Read_Buf(unsigned char reg,unsigned char *pBuf,unsigned char len);
unsigned char NRF24L01_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char len);
unsigned char NRF24L01_RxPacket(unsigned char *rxbuf);
unsigned char NRF24L01_TxPacket(unsigned char *txbuf);
void NRF24L01_RX_Mode(void);
void NRF24L01_TX_Mode(void);
unsigned char NRF24L01_Check(void);	
									

#endif /*NRF24L01_H_*/
