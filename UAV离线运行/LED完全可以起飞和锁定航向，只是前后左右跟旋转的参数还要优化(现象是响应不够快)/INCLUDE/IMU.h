#ifndef IMU_H_
#define IMU_H_

void MahonyIMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void ACC_SMOOTH(unsigned char smooth_tms);
void YAW_SMOOTH(unsigned char smooth_tms);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void ACC_Filter_with_FIR(void);
float ADXL345_Get_Angle(float x,float y,float z,unsigned char dir);

#endif /*IMU_H_*/
