#ifndef DATA_TRANSFER_H_
#define DATA_TRANSFER_H_

//void Data_Send_Status(void);
void Data_Send_Status(float Pitch,float Roll,float Yaw,unsigned int *gyro,unsigned int *accel);
void Data_Send_Senser(void);
#endif /*DATA_TRANSFER_H_*/
