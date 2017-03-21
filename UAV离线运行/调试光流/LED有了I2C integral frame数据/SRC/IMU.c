#include "imu.h"
#include "define.h"
#include "mpu6050.h"
//#include "math.h"
#include "fast_math.h"
#include "math.h"
#define Gyr_Gain 0.06097

extern float Pitch,Roll,Yaw;
extern struct DATA_XYZ_F GYR_F;
extern struct DATA_XYZ_F GYR_F_RATE;
extern struct DATA_XYZ ACC;
extern struct DATA_XYZ GYR;
extern struct DATA_XYZ GYR_RATE;
extern struct DATA_XYZ ACC_AVG;

short ACC_X_BUF[10]={0};
short ACC_Y_BUF[10]={0};
short ACC_Z_BUF[10]={0};
int YAW_X_BUF[10]={0};
int YAW_Y_BUF[10]={0};
int YAW_Z_BUF[10]={0};
float Acc=0;
int Acc_int;
extern int YAW_X,YAW_Y,YAW_Z;
extern float YAW_AVG_X,YAW_AVG_Y,YAW_AVG_Z;
int flag_Acc=0;

void ACC_SMOOTH(unsigned char smooth_tms)
{
	static unsigned char filter_cnt=0;
	long temp1=0,temp2=0,temp3=0;
	unsigned char i;

	GYR_F.X = GYR.X * Gyr_Gain;
	GYR_F.Y = GYR.Y * Gyr_Gain;
	GYR_F.Z = GYR.Z * Gyr_Gain;
	GYR_F_RATE.X = GYR_RATE.X * Gyr_Gain;
	GYR_F_RATE.Y = GYR_RATE.Y * Gyr_Gain;
	GYR_F_RATE.Z = GYR_RATE.Z * Gyr_Gain;

	ACC_X_BUF[filter_cnt] = ACC.X;//更新滑动窗口数组
	ACC_Y_BUF[filter_cnt] = ACC.Y;
	ACC_Z_BUF[filter_cnt] = ACC.Z;
	for(i=0;i<smooth_tms;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	ACC_AVG.X = temp1 / (float)smooth_tms;
	ACC_AVG.Y = temp2 / (float)smooth_tms;
	ACC_AVG.Z = temp3 / (float)smooth_tms;
	filter_cnt++;
	if(filter_cnt==smooth_tms)	filter_cnt=0;	 		  
}
void YAW_SMOOTH(unsigned char smooth_tms)
{
	static unsigned char filter_cnt1=0;
	long temp1=0,temp2=0,temp3=0;
	unsigned char i;
	YAW_X_BUF[filter_cnt1] = YAW_X;//更新滑动窗口数组
	YAW_Y_BUF[filter_cnt1] = YAW_Y;
	YAW_Z_BUF[filter_cnt1] = YAW_Z;
	for(i=0;i<smooth_tms;i++)
	{
		temp1 += YAW_X_BUF[i];
		temp2 += YAW_Y_BUF[i];
		temp3 += YAW_Z_BUF[i];
	}
	YAW_AVG_X = temp1 / (float)smooth_tms;
	YAW_AVG_Y= temp2 / (float)smooth_tms;
	YAW_AVG_Z= temp3 / (float)smooth_tms;
	filter_cnt1++;
	if(filter_cnt1==smooth_tms)	filter_cnt1=0;
}

float twoKp = 2;    // 2 * proportional gain (Kp)
float twoKi = 0.005;    // 2 * integral gain (Ki)
float dt =0.0025;
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;  // integral error terms scaled by Ki
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  
float q_0 = 1, q_1 = 0, q_2 = 0, q_3 = 0;
void MahonyIMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
	gx*=0.0174;
    gy*=0.0174;
    gz*=0.0174;
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // Compute and apply integral feedback if enabled
    integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
    integralFBy += twoKi * halfey * dt;
    integralFBz += twoKi * halfez * dt;
    gx += integralFBx;  // apply integral feedback
    gy += integralFBy;
    gz += integralFBz;
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
    // Integrate rate of change of quaternion
    gx *= 0.0125f;//(0.5f * dt);   // pre-multiply common factors
  	gy *= 0.0125f;//(0.5f * dt);
 	gz *= 0.0125f;//(0.5f * dt);   //0.00125f
  	qa = q0;
  	qb = q1;
  	qc = q2;
  	q0 += (-qb * gx - qc * gy - q3 * gz);
  	q1 += (qa * gx + qc * gz - q3 * gy);
  	q2 += (qa * gy - qb * gz + q3 * gx);
  	q3 += (qa * gz + qb * gy - qc * gx);

  	// Normalise quaternion
  	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  	q0 *= recipNorm;
  	q1 *= recipNorm;
  	q2 *= recipNorm;
  	q3 *= recipNorm;

    Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
    Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
	Yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 57.3;//-360;
	Acc=((ax*(2.0f*(q1*q3-q0*q2))+ay*(2.0f*(q0*q1+q2*q3))+az*(q0*q0-q1*q1-q2*q2+q3*q3))-1.0f);
	if(Acc>0)
		flag_Acc=1;
	Acc_int=(int)(Acc*100);
	Acc=(float)(Acc_int)/100;

//	if(Yaw<0) Yaw+=360;
}

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm,norm ;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	gx*=0.0174;
    gy*=0.0174;
    gz*=0.0174;

	Acc=((ax/8192*(2.0f*(q1*q3-q0*q2))+ay/8192*(2.0f*(q0*q1+q2*q3))+az/8192*(q0*q0-q1*q1-q2*q2+q3*q3))-1.0f);//az/8192-1.0f;
	if(Acc>0)
		flag_Acc=1;
	Acc_int=(int)(Acc*100);
	Acc=(float)(Acc_int)/100;

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
			integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;

	// Integrate rate of change of quaternion
	gx *=0.0125f; //(0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *=0.0125f;// (0.5f * (1.0f / sampleFreq));
	gz *=0.0125f;//(0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

    Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
    Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
	  /*if(Roll>90||Roll<-90)
	  {
	  	if(Pitch>0)
		Pitch=180-Pitch;
		if(Pitch<0)
		Pitch=-(180+Pitch);
	  }	  */
	Yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 57.3;//-360;
//	if(Yaw<0) Yaw+=360;

	norm =invSqrt(q_0*q_0+q_1*q_1+q_2*q_2+q_3*q_3);
	q_0=q_0*norm;
	q_1=q_1*norm;
	q_2=q_2*norm;
	q_3=q_3*norm;
	//Acc=((ax*(2.0f*(q_1*q_3-q_0*q_2))+ay*(2.0f*(q_0*q_1+q_2*q_3))+az*(q_0*q_0-q_1*q_1-q_2*q_2+q_3*q_3))-1.0f);
}
extern float Alt_AVG;
float Accdt=0,Accdt1=0,Accdt11=0,Accdt2=0;
float last_Alt_AVG=0,Alt_AVG_V=0;
void high_acc_ms5611(void)
{
	last_Alt_AVG=Alt_AVG;
	Alt_AVG_V=(Alt_AVG-last_Alt_AVG)*50;//厘米/s   2.5*8=20ms   20ms算一次
	Accdt+=(Acc)*9.8*0.25;//厘米/s   dt=2.5/1000s
	Accdt=Accdt*0.985+Alt_AVG_V*0.015;
	Accdt2+=Accdt*0.0025;//厘米      *0.01f即乘以积分周期，我的是2.5ms，故是乘以0.0025
	Accdt2=Accdt2*0.985+Alt_AVG*0.015;
//	Accdt1+=(Acc)*9.8*0.25;//厘米/s   dt=2.5/1000s       //全部信赖Acc，得到速度
//	Accdt11+=Accdt1*0.0025;   //全部信赖Acc，得到高度
}
float ADXL345_Get_Angle(float x,float y,float z,unsigned char dir)
{
	float temp;
 	float res=0;
	switch(dir)
	{
		case 0://与自然Z轴的角度
 			temp=sqrt((x*x+y*y))/z;
 			res=atan(temp);
 			break;
		case 1://与自然X轴的角度
 			temp=x/sqrt((y*y+z*z));
 			res=atan(temp);
 			break;
 		case 2://与自然Y轴的角度
 			temp=y/sqrt((x*x+z*z));
 			res=atan(temp);
 			break;
 	}
	return res*180/3.14;
}
