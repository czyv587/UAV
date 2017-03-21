#include "hardware_I2C.h"
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

unsigned int readdata[]={0,0,0,0,0,0,0,0,0};

unsigned int WriteData(unsigned int	RomAddress, unsigned int	REG_data, unsigned int	number)
{
   if (I2caRegs.I2CSTR.bit.BB == 1)
   {
      return I2C_BUS_BUSY_ERROR;
   }
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = 0x68;//SlaveAddress
   I2caRegs.I2CCNT = number + 1;
   //I2caRegs.I2CDXR = 0x00; //Send high byte of RomAddress    HAddress&0xFF
   I2caRegs.I2CMDR.all = 0x6E20;
  //    while(!I2C_xrdy());
   I2caRegs.I2CDXR = RomAddress&0xFF; //Send low byte of RomAddress
   while(!I2C_xrdy());
   I2caRegs.I2CDXR = REG_data;
 /*  for (i=0; i<number; i++)
   {
      while(!I2C_xrdy());
      I2caRegs.I2CDXR = *Wdata;
      Wdata++;
	  if (I2caRegs.I2CSTR.bit.NACK == 1)
   		  return	I2C_BUS_BUSY_ERROR;
   }   	*/
   return I2C_SUCCESS;   
}
unsigned int ReadData(unsigned int	RomAddress, unsigned int number)
{
   unsigned int  i,Temp,HAddress;
   
   if (I2caRegs.I2CSTR.bit.BB == 1)
   {
       return I2C_BUS_BUSY_ERROR;
   }
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = 0x68;//SlaveAddress;
   I2caRegs.I2CCNT = 1;
   HAddress = RomAddress>>8;
   //I2caRegs.I2CDXR = 0x00; //Send high byte of RomAddress   HAddress&0xFF
   I2caRegs.I2CMDR.all = 0x6620; 
    //  while(!I2C_xrdy());
   I2caRegs.I2CDXR = RomAddress&0xFF; //Send low byte of RomAddress
   if (I2caRegs.I2CSTR.bit.NACK == 1)
   		return	I2C_BUS_BUSY_ERROR;
   //DELAY_US(50);		
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = 0x68;//SlaveAddress;
   I2caRegs.I2CCNT = number;	 
   I2caRegs.I2CMDR.all = 0x6C20; 
   if (I2caRegs.I2CSTR.bit.NACK == 1)
   		return	I2C_BUS_BUSY_ERROR;
   for(i=0;i<number;i++)
   {
      while(!I2C_rrdy());
   	  Temp = I2caRegs.I2CDRR;
	  if (I2caRegs.I2CSTR.bit.NACK == 1)
   		  return	I2C_BUS_BUSY_ERROR;
   	  readdata[i] = Temp;
   }
//   data=(readdata[0]<<8)+readdata[1];
   return I2C_SUCCESS;
}
unsigned int	I2C_xrdy()
{
	unsigned int	t;
	t = I2caRegs.I2CSTR.bit.XRDY;
	return t;
}
unsigned int	I2C_rrdy()
{
	unsigned int	t;
	t = I2caRegs.I2CSTR.bit.RRDY;
	return t;
}
char hw_Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
   if (I2caRegs.I2CSTR.bit.BB == 1)
   {
      return I2C_BUS_BUSY_ERROR;
   }
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = SlaveAddress;//SlaveAddress
   I2caRegs.I2CCNT = 2;//REG_Address+REG_data
   //I2caRegs.I2CDXR = 0x00; //Send high byte of RomAddress    HAddress&0xFF
   I2caRegs.I2CMDR.all = 0x6E20;
  //    while(!I2C_xrdy());
   I2caRegs.I2CDXR = REG_Address; //Send low byte of RomAddress
   while(!I2C_xrdy());
   I2caRegs.I2CDXR = REG_data;
 /*  for (i=0; i<number; i++)
   {
      while(!I2C_xrdy());
      I2caRegs.I2CDXR = *Wdata;
      Wdata++;
	  if (I2caRegs.I2CSTR.bit.NACK == 1)
   		  return	I2C_BUS_BUSY_ERROR;
   }   	*/
   return I2C_SUCCESS;   
}
unsigned char hw_i2cRead(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char len,unsigned char *buf)
{
	unsigned int  i,Temp;
   
   if (I2caRegs.I2CSTR.bit.BB == 1)
   {
       return I2C_BUS_BUSY_ERROR;
   }
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = SlaveAddress;//SlaveAddress;
   I2caRegs.I2CCNT = 1;//SlaveAddress
   //I2caRegs.I2CDXR = 0x00; //Send high byte of RomAddress   HAddress&0xFF
   I2caRegs.I2CMDR.all = 0x6620; 
    //  while(!I2C_xrdy());
   I2caRegs.I2CDXR = REG_Address; //Send low byte of RomAddress
   if (I2caRegs.I2CSTR.bit.NACK == 1)
   		return	I2C_BUS_BUSY_ERROR;
   //DELAY_US(50);		
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = SlaveAddress;//SlaveAddress;
   I2caRegs.I2CCNT = len;	 
   I2caRegs.I2CMDR.all = 0x6C20; 
   if (I2caRegs.I2CSTR.bit.NACK == 1)
   		return	I2C_BUS_BUSY_ERROR;
   for(i=0;i<len;i++)
   {
      while(!I2C_rrdy());
   	  Temp = I2caRegs.I2CDRR;
	  if (I2caRegs.I2CSTR.bit.NACK == 1)
   		  return	I2C_BUS_BUSY_ERROR;
   	  buf[i] = Temp;
   }
//   data=(readdata[0]<<8)+readdata[1];
   return I2C_SUCCESS;
}
unsigned int hw_i2cRead_Single(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char len)
{
	unsigned int  i,Temp,data;
   
   if (I2caRegs.I2CSTR.bit.BB == 1)
   {
       return I2C_BUS_BUSY_ERROR;
   }
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = SlaveAddress;//SlaveAddress;
   I2caRegs.I2CCNT = 1;//SlaveAddress
   //I2caRegs.I2CDXR = 0x00; //Send high byte of RomAddress   HAddress&0xFF
   I2caRegs.I2CMDR.all = 0x6620; 
    //  while(!I2C_xrdy());
   I2caRegs.I2CDXR = REG_Address; //Send low byte of RomAddress
   if (I2caRegs.I2CSTR.bit.NACK == 1)
   		return	I2C_BUS_BUSY_ERROR;
   //DELAY_US(50);		
   while(!I2C_xrdy());
   I2caRegs.I2CSAR = SlaveAddress;//SlaveAddress;
   I2caRegs.I2CCNT = len;	 
   I2caRegs.I2CMDR.all = 0x6C20; 
   if (I2caRegs.I2CSTR.bit.NACK == 1)
   		return	I2C_BUS_BUSY_ERROR;
   for(i=0;i<len;i++)
   {
      while(!I2C_rrdy());
   	  Temp = I2caRegs.I2CDRR;
	  if (I2caRegs.I2CSTR.bit.NACK == 1)
   		  return	I2C_BUS_BUSY_ERROR;
   	  data = Temp;
   }
//   data=(readdata[0]<<8)+readdata[1];
   return data;
}
