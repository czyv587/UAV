#ifndef HARDWARE_I2C_H_
#define HARDWARE_I2C_H_

unsigned int WriteData(unsigned int	RomAddress, unsigned int	REG_data, unsigned int	number);
unsigned int ReadData(unsigned int	RomAddress, unsigned int number);
unsigned int	I2C_xrdy();
unsigned int	I2C_rrdy();
char hw_Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);		     //void
char hw_Single_MS5611_Write(unsigned char SlaveAddress,unsigned char REG_data);
unsigned char hw_i2cRead(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char len,unsigned char *buf);
unsigned int hw_i2cRead_Single(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char len);

#endif /*HARDWARE_I2C_H_*/
