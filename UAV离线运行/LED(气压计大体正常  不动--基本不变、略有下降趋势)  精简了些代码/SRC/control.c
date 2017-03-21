#include "control.h"
#include "define.h"
#include "pwm.h"
#include "DSP2833x_Device.h"
struct DATA_XYZ_F GYR_F={0.0,0.0,0.0};
struct DATA_XYZ_F GYR_F_RATE={0.0,0.0,0.0};
struct DATA_XYZ_F GYR_F_AVG={0.0,0.0,0.0};
struct DATA_XYZ_F Current_ERROR={0.0,0.0,0.0};

struct DATA_XYZ ACC={0,0,0};
struct DATA_XYZ GYR_CTR={0,0,0};
//struct DATA_XYZ DATA[100]={0,0,0};
struct DATA_XYZ GYR={0,0,0};
struct DATA_XYZ GYR_RATE={0,0,0};
struct DATA_XYZ HMC={0,0,0};
struct DATA_XYZ ACC_OFFSET={0,0,0};
struct DATA_XYZ GYR_OFFSET={0,0,0};
struct DATA_XYZ ACC_AVG={0,0,0};
extern struct DATA_XYZ_F gyr_offset_f;
extern Uint32 Time1on,Time2on,Time4on;
extern long pwm_num3;
float YAW_INIT=0.0;
struct PID PID_DATA;
struct PID YAW_PID;
struct PID EXP_PID;
struct DATA_16 COMD={0,0,0,0};
float Pitch=0.0,Roll=0.0,Yaw=0.0;
float roll=0,pitch=0;
float EXP_YAW=0;
unsigned char ct=0;
unsigned char FLY_EN=0;
unsigned char ThreeD_EN=0;
unsigned int  ThreeD_TMS=0;
unsigned char Motion=0,Last_Motion=0; 
float Dynamic_PID_rate=0.0;
float D2=0.0;
struct DATA_XYZ_F EXP_THTA={0.0,0.0,0.0};

float RC_Pitch=0,RC_Roll=0,RC_Yaw=0;
float Pitch_i=0,Roll_i=0,Yaw_i=0;
float Pitch_shell_kp,Pitch_shell_ki,Roll_shell_kp,Roll_shell_ki,Yaw_shell_kp,Yaw_shell_ki;
float Pitch_shell_ki_out,Roll_shell_ki_out;
float Roll_shell_out=0,Pitch_shell_out=0,Yaw_shell_out=0;
float Pitch_core_kp=0,Pitch_core_ki=0,Pitch_core_kd=0;
float Roll_core_kp=0,Roll_core_ki=0,Roll_core_kd=0;
float Yaw_core_kp=0,Yaw_core_ki=0,Yaw_core_kd=0;
float pitch_core_kp_out=0,pitch_core_ki_out=0,pitch_core_kd_out=0;
float Roll_core_kp_out=0,Roll_core_ki_out=0,Roll_core_kd_out=0;
float Yaw_core_kp_out=0,Yaw_core_ki_out=0,Yaw_core_kd_out=0;
float Pitch_core_out=0,Roll_core_out=0,Yaw_core_out=0;
float MPU6050_GYRO_LAST_X=0,MPU6050_GYRO_LAST_Y=0,MPU6050_GYRO_LAST_Z=0;
float Gyro_radian_old_x=0,Gyro_radian_old_y=0,Gyro_radian_old_z=0;

float Exp_rate=0.012;   
#define MAX_ANGLE 20
float t1=0.0,t2=0.0,t3=0.0;
float NOW_GO_BACK_X=0.0,NOW_GO_BACK_Y=0.0;
float LAST_GO_BACK_X=0.0,LAST_GO_BACK_Y=0.0;
float GO_BACK_X_VAL=0.0,GO_BACK_Y_VAL=0.0;
unsigned int GO_BACK_X_CT=0,GO_BACK_Y_CT=0;
unsigned char  GO_BACK_X_STR=0,GO_BACK_Y_STR=0;
extern int WARNING;
extern unsigned char BUF_recv[18];
void PID_INIT(void)	   
{
	Roll_core_kp=((float)(BUF_recv[0]));//0;//((float)(BUF_recv[0]));
	Roll_core_ki=((float)(BUF_recv[1]))/100;//0/100;//((float)(BUF_recv[1]))/100;
	Roll_core_kd=((float)(BUF_recv[2]));//0;//((float)(BUF_recv[2]));

	Pitch_core_kp=((float)(float)(BUF_recv[3]));//90;//((float)(BUF_recv[3]));
	Pitch_core_ki=((float)(BUF_recv[4]))/100;//30/100;//((float)(BUF_recv[4]))/100;
	Pitch_core_kd=((float)(BUF_recv[5]));//20;//((float)(BUF_recv[5]));

	Yaw_core_kp=((float)(BUF_recv[6]));//((float)(BUF_recv[6]));
	Yaw_core_ki=((float)(BUF_recv[7]))/100;//((float)(BUF_recv[7]))/100;
	Yaw_core_kd=((float)(BUF_recv[8]));//((float)(BUF_recv[8]));

	Roll_shell_kp=((float)(BUF_recv[9]))/10;//0/100000;//((float)(BUF_recv[9]))/100000;
	Roll_shell_ki=((float)(BUF_recv[10]))/100000;//0/1000000;//((float)(BUF_recv[10]))/1000000;


	Pitch_shell_kp=((float)(BUF_recv[12]))/10;//80/100000;//((float)(BUF_recv[12]))/100000;
	Pitch_shell_ki=((float)(BUF_recv[13]))/100000;//0;//((float)(BUF_recv[13]))/1000000;

	Yaw_shell_kp=((float)(BUF_recv[15]))/10;//((float)(BUF_recv[15]))/100;
	Yaw_shell_ki=((float)(BUF_recv[16]))/100000;//((float)(BUF_recv[16]))/100000;

	PID_DATA.P =1;//((float)(BUF_recv[3]));
	PID_DATA.I = 0;//((float)(BUF_recv[4]))/100;
	PID_DATA.D = 7;//((float)(BUF_recv[5]));
	/*	PID_DATA.SUM_ERROR_R=0.0;
	PID_DATA.SUM_ERROR_P=0.0;
	PID_DATA.IMAX = 300;

	YAW_PID.P = 7.5;	//7.5   
	YAW_PID.I = 0;   //0.2
	YAW_PID.D = 1.8;	//1.8	 
	YAW_PID.SUM_ERROR_R=0.0;
	YAW_PID.SUM_ERROR_P=0.0;
	YAW_PID.IMAX = 100;

	EXP_PID.P = 1;	    
	EXP_PID.I = 0;    
	EXP_PID.D = 1.8; 	 
	EXP_PID.SUM_ERROR_R=0.0;
	EXP_PID.SUM_ERROR_P=0.0;
	EXP_PID.IMAX = 100;*/
}

float EXP_BUF_x[50]={0};
float EXP_BUF_y[50]={0};
float avg_exp_x=0,avg_exp_y=0;
void EXP_SMOOTH(unsigned char tms)
{
	static unsigned char cnt=0; 
	unsigned char i=0;
	float temp1=0,temp2=0;
	EXP_BUF_x[cnt] = EXP_THTA.X;//更新滑动窗口数组
	EXP_BUF_y[cnt] = EXP_THTA.Y;//更新滑动窗口数组
	for(i=0;i<tms;i++)
	{
		temp1 += EXP_BUF_x[i]; 
		temp2 += EXP_BUF_y[i]; 
	}
	avg_exp_x = temp1 /(float)tms; 
	avg_exp_y = temp2 /(float)tms;
	cnt++;
	if(cnt==tms)	cnt=0;	 		  
}


float last_exp_x=0.0,last_exp_y=0.0;
float now_exp_x=0.0,now_exp_y=0.0;
float turn=0;
int tt1=0,tt2=0;
int maxx=30;
int goback=0;
float yaw_error=0,last_yaw_error=0.0;
float yaw_p=5,yaw_d=3.065;
float exp_yaw=0.0;
float yaw_tmp=0.0;
float turn_rate=0.00125;

void READ_CONTROL_COMMAND(unsigned char* tmp_buf)			//读取摇杆无线信号方向
{ 
	COMD.THR=  tmp_buf[0]*255+tmp_buf[1]-250;
 	COMD.THR*=0.32;
 	if(COMD.THR>1000) COMD.THR=1000;	
 
	if(COMD.THR>100)
	{
		FLY_EN=1;
	}
	else
	{
		exp_yaw=Yaw;
		FLY_EN=0;
		PID_DATA.SUM_ERROR_R=0;
		PID_DATA.SUM_ERROR_P=0;
	}
	if(tmp_buf[2]&0X80)				  	
		COMD.TURN= ((tmp_buf[2]&0X7F)*255+tmp_buf[3]);  
	else
		COMD.TURN= -(tmp_buf[2]*255+tmp_buf[3]);  
	if(tmp_buf[4]&0X80)				  	
		COMD.LR= ((tmp_buf[4]&0X7F)*255+tmp_buf[5]);  
	else
		COMD.LR= -(tmp_buf[4]*255+tmp_buf[5]); 
	if(tmp_buf[6]&0X80)				  	
		COMD.FB= -((tmp_buf[6]&0X7F)*255+tmp_buf[7]);  
	else
		COMD.FB= (tmp_buf[6]*255+tmp_buf[7]); 

	if(COMD.TURN<200&&COMD.TURN>-200)  COMD.TURN=0;

	Exp_rate=0.0133;
	turn_rate=0.00205;	
	now_exp_x=COMD.FB*Exp_rate;
	now_exp_y=-COMD.LR*Exp_rate;
 	turn=COMD.TURN*turn_rate;
	exp_yaw +=turn;
	if(exp_yaw>360) exp_yaw=0;
	if(exp_yaw<0) exp_yaw=360;
    
 	EXP_THTA.X= now_exp_x;
	EXP_THTA.Y= now_exp_y;
	avg_exp_x= EXP_THTA.X;
	avg_exp_y= EXP_THTA.Y;
	EXP_SMOOTH(2);

	if(FLY_EN==0&&ThreeD_EN==0&&COMD.FB<-2000)
	{
		ThreeD_EN=1;
		PID_DATA.SUM_ERROR_P=0;
		PID_DATA.SUM_ERROR_R=0;
	}
 	if(COMD.FB<5&&COMD.FB>-5)	
		COMD.FB=0 ;
	else
		Motion=1;
	if(COMD.LR<5&&COMD.LR>-5) 
		COMD.LR=0;
	else
		Motion=1;
	if(COMD.FB==0&&COMD.LR==0)
		Motion=0;	 
	
	now_exp_x=COMD.FB*Exp_rate;
	now_exp_y=-COMD.LR*Exp_rate;
    
 	EXP_THTA.X= now_exp_x;
	EXP_THTA.Y= now_exp_y; 
	EXP_SMOOTH(6);
	if(avg_exp_x> MAX_ANGLE)	avg_exp_x= MAX_ANGLE;
	if(avg_exp_x<-MAX_ANGLE)	avg_exp_x=-MAX_ANGLE;	
	if(avg_exp_y> MAX_ANGLE)	avg_exp_y= MAX_ANGLE;
	if(avg_exp_y<-MAX_ANGLE)	avg_exp_y=-MAX_ANGLE;		   // 期望角度 限幅	
/*******************************-**************************************/
	Current_ERROR.X =   avg_exp_x- Roll;
	Current_ERROR.Y =   avg_exp_y- Pitch;

	last_exp_x=now_exp_x;
	last_exp_y=now_exp_y;
}


float last_gyr_x=0.0,last_gyr_y=0.0;
float sum_gyr_x=0.0,sum_gyr_y=0.0;
int Motor1=0,Motor2=0,Motor3=0,Motor4=0;
int Motor1_1=0,Motor2_1=0,Motor3_1=0,Motor4_1=0;
int yaw=0; 	
float yaw_dt=0;

float RI_MAX=0,PI_MAX=0;
 
void STABLE_WITH_PID(void)
{
	Last_Motion=Motion;
////////////////////////////////////////////////////////////////////////////////PID计算
    RI_MAX=PID_DATA.IMAX  ;
 	Roll = PID_DATA.P * Current_ERROR.X;

	if(Current_ERROR.X<60&&Current_ERROR.X>-60)
		PID_DATA.SUM_ERROR_R+= PID_DATA.I*Current_ERROR.X/400.0 ;
	else
		PID_DATA.SUM_ERROR_R=0;

	if(PID_DATA.SUM_ERROR_R> RI_MAX)	PID_DATA.SUM_ERROR_R= RI_MAX;
	if(PID_DATA.SUM_ERROR_R<-RI_MAX)	PID_DATA.SUM_ERROR_R=-RI_MAX;
 
    Roll+=	PID_DATA.SUM_ERROR_R;	  // 加上积分
 
   Roll  +=	 - (PID_DATA.D * (GYR_F_RATE.X-gyr_offset_f.X))  ;
 
////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////// 

	PI_MAX=PID_DATA.IMAX;
	pitch = PID_DATA.P * Current_ERROR.Y ;

	if(Current_ERROR.Y<60&&Current_ERROR.Y>-60)
 		PID_DATA.SUM_ERROR_P+= PID_DATA.I*Current_ERROR.Y/400.0 ;
 	else
 		PID_DATA.SUM_ERROR_P=0;
	if(PID_DATA.SUM_ERROR_P> PI_MAX)	PID_DATA.SUM_ERROR_P= PI_MAX;
	if(PID_DATA.SUM_ERROR_P<-PI_MAX)	PID_DATA.SUM_ERROR_P=-PI_MAX;
   
 	pitch +=  PID_DATA.SUM_ERROR_P;	  // 加上积分  
 
	pitch +=	 - (PID_DATA.D * (GYR_F_RATE.Y-gyr_offset_f.Y)) ; 
/////////////
	yaw_tmp= exp_yaw-Yaw;
	if(yaw_tmp>-180)
	{
		if(yaw_tmp>180)
			yaw_error=( exp_yaw-Yaw-360);
		else
			yaw_error= exp_yaw-Yaw;
	}
	else
		yaw_error= (360+exp_yaw-Yaw);
	yaw =-yaw_p*yaw_error- yaw_d*GYR_F.Z;

// 	yaw = -5 * GYR_F.Z;
//	yaw-=COMD.TURN*0.47;
//	yaw=0;	
////////////////////////////////////////////////////////////////////////////////将输出值融合到四个电机
	Motor1=(int)(COMD.THR + Roll - pitch + yaw);
	Motor2=(int)(COMD.THR + Roll + pitch - yaw);
	Motor3=(int)(COMD.THR - Roll + pitch + yaw);
	Motor4=(int)(COMD.THR - Roll - pitch - yaw);
	Limit_amplitude(&Motor1,&Motor2,&Motor3,&Motor4);
  //FLY_EN=0;
	if(FLY_EN==1&&COMD.THR>200)  
	{
	    Refresh_Or_STOP_MOTOR(1);
		ThreeD_EN=0;			 
		ThreeD_TMS=0;
	}	
	else  Refresh_Or_STOP_MOTOR(2);
}

void CRASH_LANDING()
{
	COMD.THR=  330;
	FLY_EN=1;
	COMD.LR=	0;
	COMD.FB=	0;
	COMD.TURN=  0;  
	Current_ERROR.X=0;
	Current_ERROR.Y=0;
	PID_DATA.SUM_ERROR_R=0;
	PID_DATA.SUM_ERROR_P=0; 	
}

void Limit_amplitude() 
{
	if(Motor1_1>9374) Motor1_1=9374;
	else if(Motor1_1<0) Motor1_1=0;
	if(Motor2_1>9374) Motor2_1=9374;
	else if(Motor2_1<0) Motor2_1=0;
	if(Motor3_1>9374) Motor3_1=9374;
	else if(Motor3_1<0) Motor3_1=0;
	if(Motor4_1>9374) Motor4_1=9374;
	else if(Motor4_1<0) Motor4_1=0;
}
void Refresh_Or_STOP_MOTOR(unsigned char sel)
{
	if(sel==1)
	{
		PWM_CH1(Motor1_1);
		PWM_CH2(Motor2_1);
		PWM_CH3(Motor3_1);
		PWM_CH4(Motor4_1);
		/*PWM_CH1(Motor1);	  //2
		PWM_CH2(Motor2);	  //3
		PWM_CH3(Motor3);	  //4
		PWM_CH4(Motor4);	  //1*/
	}
	else if(sel==2)	
	{
		PWM_CH1(0);
		PWM_CH2(0);
		PWM_CH3(0);
		PWM_CH4(0);
	//	Motor1=0;Motor2=0;Motor3=0;Motor4=0;
	}
}

void control(void)
{
	RC_Pitch=0;//(Rc_D.PITCH-1500)/20;
////////////////////////外环角度环(PID)///////////////////////////////
	Pitch_i+=(RC_Pitch-Pitch);
//-------------Pitch积分限幅----------------//
	Pitch_shell_ki_out=Pitch_shell_ki*Pitch_i;
/*	if(Pitch_shell_ki_out>0.3) Pitch_shell_ki_out=0.3;
	else if(Pitch_shell_ki_out<-0.3) Pitch_shell_ki_out=-0.3;*/
//-------------Pitch  PID-------------------//
	Pitch_shell_out = Pitch_shell_kp*(RC_Pitch-Pitch) + Pitch_shell_ki_out;
	//Pitch_shell_out=0;
//角度保存
//	Pitch_old=Q_ANGLE.Pitch;
/*********************************************************/

	RC_Roll=0;//(Rc_D.ROLL-1500)/20;
	Roll_i+=(RC_Roll-Roll);
//-------------Roll积分限幅----------------//
	Roll_shell_ki_out=Roll_shell_ki*Roll_i;
/*	if(Roll_shell_ki_out>0.3) Roll_shell_ki_out=0.3;
	else if(Roll_shell_ki_out<-0.3) Roll_shell_ki_out=-0.3;*/
//-------------Roll  PID-------------------//
	Roll_shell_out  = -(Roll_shell_kp*(RC_Roll-Roll) + Roll_shell_ki_out);
	Roll_shell_out=0;
//------------Roll角度保存------------------//
//	Roll_old=Q_ANGLE.Rool;
/*********************************************************/

/*    RC_Yaw=(Rc_D.YAW-1500)*10;
//-------------Yaw微分--------------------//
    Yaw_d=MPU6050_GYRO_LAST.Z-Yaw_old;
//-------------Roll  PID-------------------//
    Yaw_shell_out  = Yaw_shell_kp*(RC_Yaw-MPU6050_GYRO_LAST.Z) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
//------------Roll角度保存------------------//
    Yaw_old=MPU6050_GYRO_LAST.Z;*/

////////////////////////内环角速度环(PD)///////////////////////////////
	MPU6050_GYRO_LAST_X=GYR_F_RATE.X-gyr_offset_f.X;
	MPU6050_GYRO_LAST_Y=GYR_F_RATE.Y-gyr_offset_f.Y;
	MPU6050_GYRO_LAST_Z=GYR_F.Z;

	pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out - MPU6050_GYRO_LAST_Y);
	pitch_core_ki_out += Pitch_core_ki * (Pitch_shell_out - MPU6050_GYRO_LAST_Y);//限幅
	//-------------Pitch积分限幅----------------//
		if(pitch_core_ki_out>9000) pitch_core_ki_out=9000;
		else if(pitch_core_ki_out<-9000) pitch_core_ki_out=-9000;
	pitch_core_kd_out = -Pitch_core_kd * (MPU6050_GYRO_LAST_Y   - Gyro_radian_old_y);

	Roll_core_kp_out = Roll_core_kp * (Roll_shell_out  - MPU6050_GYRO_LAST_X);
	Roll_core_ki_out += Roll_core_ki * (Roll_shell_out - MPU6050_GYRO_LAST_X);//限幅
	//-------------Pitch积分限幅----------------//
		if(Roll_core_ki_out>9000) Roll_core_ki_out=9000;
		else if(Roll_core_ki_out<-9000) Roll_core_ki_out=-9000;
	Roll_core_kd_out = -Roll_core_kd * (MPU6050_GYRO_LAST_X - Gyro_radian_old_x);

/*	Yaw_core_kp_out  = Yaw_core_kp * (Yaw_shell_out + MPU6050_GYRO_LAST.Z * 1);
	Yaw_core_ki_out += Yaw_core_ki * (Yaw_shell_out - MPU6050_GYRO_LAST.Z * 3.5);
	Yaw_core_kd_out  = Yaw_core_kd * (MPU6050_GYRO_LAST.Z - Gyro_radian_old_z);*/

    Pitch_core_out = pitch_core_kp_out + pitch_core_ki_out + pitch_core_kd_out;
	Roll_core_out  = Roll_core_kp_out + Roll_core_ki_out + Roll_core_kd_out;
//	Yaw_core_out  = Yaw_core_kp_out +Yaw_core_ki_out + Yaw_core_kd_out;

	Gyro_radian_old_x = MPU6050_GYRO_LAST_X;
	Gyro_radian_old_y = MPU6050_GYRO_LAST_Y;
//	Gyro_radian_old_z = MPU6050_GYRO_LAST_Z;   //储存历史值


//	Motor1=(int)(pwm_num3 + Roll_core_out - Pitch_core_out);//- pitch + Roll - yaw
//	Motor2=(int)(pwm_num3 - Roll_core_out - Pitch_core_out);//- pitch - Roll + yaw
	//Motor2=(int)(2000);
	//Motor3=(int)(pwm_num3 - Roll_core_out + Pitch_core_out);//+ pitch - Roll - yaw
	//Motor4=(int)(pwm_num3 + Roll_core_out + Pitch_core_out);//+ pitch + Roll + yaw

		/*Motor1_1=(int)(pwm_num3 + Roll_core_out - Pitch_core_out);//- pitch + Roll - yaw
		Motor2_1=(int)(pwm_num3 - Roll_core_out - Pitch_core_out);//- pitch - Roll + yaw
		Motor3_1=(int)(pwm_num3 - Roll_core_out + Pitch_core_out);//+ pitch - Roll - yaw
		Motor4_1=(int)(pwm_num3 + Roll_core_out + Pitch_core_out);//+ pitch + Roll + yaw*/

	/*Motor1=(int)(pwm_num3);//- pitch + Roll - yaw
	Motor2=(int)(pwm_num3);//- pitch - Roll + yaw
	Motor3=(int)(pwm_num3);//+ pitch - Roll - yaw
	Motor4=(int)(pwm_num3);//+ pitch + Roll + yaw*/

	/*Motor1=(int)(0);//- pitch + Roll - yaw
	Motor2=(int)(0);//- pitch - Roll + yaw
	Motor3=(int)(0);//+ pitch - Roll - yaw
	Motor4=(int)(0);//+ pitch + Roll + yaw*/

	Limit_amplitude();
	Refresh_Or_STOP_MOTOR(WARNING);//WARNING=1正常；WARNING=2警告
}
void control1(void)//各种符号
{
	now_exp_x=0;
	now_exp_y=0;
	turn=0;
	exp_yaw +=turn;

	avg_exp_x=0;
	avg_exp_y=0;

	Current_ERROR.X =   avg_exp_x- Roll;
	Current_ERROR.Y =   avg_exp_y- Pitch;

	Roll = PID_DATA.P * Current_ERROR.X;
//	PID_DATA.SUM_ERROR_R+= PID_DATA.I*Current_ERROR.X/400.0 ;
//	Roll+=	PID_DATA.SUM_ERROR_R;	  // 加上积分
	Roll+=-(PID_DATA.D * (GYR_F_RATE.X-gyr_offset_f.X))  ;

	pitch = PID_DATA.P * Current_ERROR.Y ;
//	PID_DATA.SUM_ERROR_P+= PID_DATA.I*Current_ERROR.Y/400.0 ;
//	pitch +=  PID_DATA.SUM_ERROR_P;	  // 加上积分
//	pitch +=-(PID_DATA.D * (GYR_F_RATE.Y-gyr_offset_f.Y)) ;

	yaw= PID_DATA.P*(exp_yaw-Yaw);//yaw= yaw_p*(exp_yaw-Yaw);
//	yaw +=-yaw_d*GYR_F.Z;

	//Motor1=(int)(pwm_num3 + Roll);//- pitch + Roll - yaw
	//Motor2=(int)(pwm_num3  - Roll);//- pitch - Roll + yaw

//	Motor3=(int)(pwm_num3  - Roll);//+ pitch - Roll - yaw

	//Motor3=(int)(3000);
//	Motor4=(int)(pwm_num3 + Roll);//+ pitch + Roll + yaw
	/*Motor1=0;//- pitch + Roll - yaw
	Motor2=0;//(int)(pwm_num3);//- pitch - Roll + yaw
	Motor3=0;//(int)(pwm_num3);//+ pitch - Roll - yaw
	Motor4=(int)(pwm_num3);//(int)(pwm_num3);//+ pitch + Roll + yaw*/
	Limit_amplitude();
	Refresh_Or_STOP_MOTOR(WARNING);//WARNING=1正常；WARNING=2警告
	/*PWM_CH1(Motor1);
	PWM_CH2(Motor2);
	PWM_CH3(Motor3);
	PWM_CH4(Motor4);*/
}
/*void control2(void)
{
	Pitch=Pitch/100.0;
	Roll=Roll/100.0;
	if((Time2on>228148)&&(Time2on<300000))
		RC_Pitch=-((float)(Time2on-228148))/1804;//(Rc_D.ROLL-1500)/20;
	else if((Time2on<228148)&&(Time2on>160000))
		RC_Pitch=(((float)(228148-Time2on))/1804);
	//RC_Pitch=0;//(Rc_D.PITCH-1500)/20;
////////////////////////外环角度环(PID)///////////////////////////////
	Pitch_i+=(RC_Pitch-Pitch);
//-------------Pitch积分限幅----------------//
	Pitch_shell_ki_out=Pitch_shell_ki*Pitch_i;
	if(Pitch_shell_ki_out>0.3) Pitch_shell_ki_out=0.3;
	else if(Pitch_shell_ki_out<-0.3) Pitch_shell_ki_out=-0.3;
//-------------Pitch  PID-------------------//
	Pitch_shell_out = Pitch_shell_kp*(RC_Pitch-Pitch) + Pitch_shell_ki_out;
	//Pitch_shell_out=0;
//角度保存
//	Pitch_old=Q_ANGLE.Pitch;
*******************************************************
	if((Time1on>229253)&&(Time1on<300000))
		RC_Roll=((float)(Time1on-229253))/1966;//(Rc_D.ROLL-1500)/20;
	else if((Time1on<229253)&&(Time1on>160000))
		RC_Roll=-(((float)(229253-Time1on))/1966);
//	RC_Roll=0;
	Roll_i+=(RC_Roll-Roll);
//-------------Roll积分限幅----------------//
	Roll_shell_ki_out=Roll_shell_ki*Roll_i;
	if(Roll_shell_ki_out>0.3) Roll_shell_ki_out=0.3;
	else if(Roll_shell_ki_out<-0.3) Roll_shell_ki_out=-0.3;
//-------------Roll  PID-------------------//
//	Roll_shell_out=RC_Roll-Roll;
	Roll_shell_out  = (Roll_shell_kp*(RC_Roll-Roll) + Roll_shell_ki_out);
//	Roll_shell_out=0;
//------------Roll角度保存------------------//
//	Roll_old=Q_ANGLE.Rool;
*******************************************************

    RC_Yaw=(Rc_D.YAW-1500)*10;
//-------------Yaw微分--------------------//
    Yaw_d=MPU6050_GYRO_LAST.Z-Yaw_old;
//-------------Roll  PID-------------------//
    Yaw_shell_out  = Yaw_shell_kp*(RC_Yaw-MPU6050_GYRO_LAST.Z) + Yaw_shell_ki*Yaw_i + Yaw_shell_kd*Yaw_d;
//------------Roll角度保存------------------//
    Yaw_old=MPU6050_GYRO_LAST.Z;

////////////////////////内环角速度环(PD)///////////////////////////////
	MPU6050_GYRO_LAST_X=GYR_F_RATE.X-gyr_offset_f.X;
	MPU6050_GYRO_LAST_Y=GYR_F_RATE.Y-gyr_offset_f.Y;
	MPU6050_GYRO_LAST_Z=GYR_F.Z;

//	Pitch_shell_out=RC_Pitch;
	pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out - MPU6050_GYRO_LAST_Y);
	pitch_core_ki_out += Pitch_core_ki * (Pitch_shell_out - MPU6050_GYRO_LAST_Y);//限幅
	//-------------Pitch积分限幅----------------//
		if(pitch_core_ki_out>9000) pitch_core_ki_out=9000;
		else if(pitch_core_ki_out<-9000) pitch_core_ki_out=-9000;
	pitch_core_kd_out = -Pitch_core_kd * (MPU6050_GYRO_LAST_Y   - Gyro_radian_old_y);

//	Roll_shell_out=RC_Roll;
	Roll_core_kp_out = Roll_core_kp * (Roll_shell_out  - MPU6050_GYRO_LAST_X);
	Roll_core_ki_out += Roll_core_ki * (Roll_shell_out - MPU6050_GYRO_LAST_X);//限幅
	//-------------Pitch积分限幅----------------//
		if(Roll_core_ki_out>9000) Roll_core_ki_out=9000;
		else if(Roll_core_ki_out<-9000) Roll_core_ki_out=-9000;
	Roll_core_kd_out = -Roll_core_kd * (MPU6050_GYRO_LAST_X - Gyro_radian_old_x);

	Yaw_core_kp_out  = Yaw_core_kp * (Yaw_shell_out + MPU6050_GYRO_LAST.Z * 1);
	Yaw_core_ki_out += Yaw_core_ki * (Yaw_shell_out - MPU6050_GYRO_LAST.Z * 3.5);
	Yaw_core_kd_out  = Yaw_core_kd * (MPU6050_GYRO_LAST.Z - Gyro_radian_old_z);

    Pitch_core_out = pitch_core_kp_out + pitch_core_ki_out + pitch_core_kd_out;
	Roll_core_out  = Roll_core_kp_out + Roll_core_ki_out + Roll_core_kd_out;
//	Yaw_core_out  = Yaw_core_kp_out +Yaw_core_ki_out + Yaw_core_kd_out;

	Gyro_radian_old_x = MPU6050_GYRO_LAST_X;
	Gyro_radian_old_y = MPU6050_GYRO_LAST_Y;
//	Gyro_radian_old_z = MPU6050_GYRO_LAST_Z;   //储存历史值


//	Motor1=(int)(pwm_num3 + Roll_core_out - Pitch_core_out);//- pitch + Roll - yaw
	Motor1=(int)(pwm_num3 - Roll_core_out - Pitch_core_out);//- pitch - Roll + yaw
	Motor2=Pitch_shell_out;
	//	Motor1=(int)(1000);
	//Motor3=(int)(pwm_num3 - Roll_core_out + Pitch_core_out);//+ pitch - Roll - yaw
	//Motor4=(int)(pwm_num3 + Roll_core_out + Pitch_core_out);//+ pitch + Roll + yaw
	Motor3=RC_Pitch;//RC_Roll;
	Motor1_1=(int)(pwm_num3 + Roll_core_out - Pitch_core_out);//- pitch + Roll - yaw
	Motor2_1=(int)(pwm_num3 - Roll_core_out - Pitch_core_out);//- pitch - Roll + yaw
	Motor3_1=(int)(pwm_num3 - Roll_core_out + Pitch_core_out);//+ pitch - Roll - yaw
	Motor4_1=(int)(pwm_num3 + Roll_core_out + Pitch_core_out);//+ pitch + Roll + yaw

	Motor1=(int)(pwm_num3);//- pitch + Roll - yaw
	Motor2=(int)(pwm_num3);//- pitch - Roll + yaw
	Motor3=(int)(pwm_num3);//+ pitch - Roll - yaw
	Motor4=(int)(pwm_num3);//+ pitch + Roll + yaw

	Motor1=(int)(0);//- pitch + Roll - yaw
	Motor2=(int)(0);//- pitch - Roll + yaw
	Motor3=(int)(0);//+ pitch - Roll - yaw
	Motor4=(int)(0);//+ pitch + Roll + yaw

	Limit_amplitude();
	Refresh_Or_STOP_MOTOR(WARNING);//WARNING=1正常；WARNING=2警告
}*/
void control2(void)
{
	Pitch=Pitch/100.0;
	Roll=Roll/100.0;
	Yaw=Yaw/100.0;
	if((Time2on>228148)&&(Time2on<300000))
		RC_Pitch=-((float)(Time2on-228148))/1804;//(Rc_D.ROLL-1500)/20;
	else if((Time2on<228148)&&(Time2on>160000))
		RC_Pitch=(((float)(228148-Time2on))/1804);
	//RC_Pitch=0;//(Rc_D.PITCH-1500)/20;
////////////////////////外环角度环(PID)///////////////////////////////
	Pitch_i+=(RC_Pitch-Pitch);
//-------------Pitch积分限幅----------------//
	Pitch_shell_ki_out=Pitch_shell_ki*Pitch_i;
/*	if(Pitch_shell_ki_out>0.3) Pitch_shell_ki_out=0.3;
	else if(Pitch_shell_ki_out<-0.3) Pitch_shell_ki_out=-0.3;*/
//-------------Pitch  PID-------------------//
	Pitch_shell_out = Pitch_shell_kp*(RC_Pitch-Pitch) + Pitch_shell_ki_out;
	//Pitch_shell_out=0;
//角度保存
//	Pitch_old=Q_ANGLE.Pitch;
/*********************************************************/
	if((Time1on>229253)&&(Time1on<300000))
		RC_Roll=((float)(Time1on-229253))/1966;//(Rc_D.ROLL-1500)/20;
	else if((Time1on<229253)&&(Time1on>160000))
		RC_Roll=-(((float)(229253-Time1on))/1966);
//	RC_Roll=0;
	Roll_i+=(RC_Roll-Roll);
//-------------Roll积分限幅----------------//
	Roll_shell_ki_out=Roll_shell_ki*Roll_i;
/*	if(Roll_shell_ki_out>0.3) Roll_shell_ki_out=0.3;
	else if(Roll_shell_ki_out<-0.3) Roll_shell_ki_out=-0.3;*/
//-------------Roll  PID-------------------//
//	Roll_shell_out=RC_Roll-Roll;
	Roll_shell_out  = (Roll_shell_kp*(RC_Roll-Roll) + Roll_shell_ki_out);
//	Roll_shell_out=0;
//------------Roll角度保存------------------//
//	Roll_old=Q_ANGLE.Rool;
/*********************************************************/

 //  RC_Yaw=(Rc_D.YAW-1500)*10;

 //  Yaw_i+=(RC_Yaw+Yaw);
//-------------Yaw积分限幅--------------------//
  // Yaw_shell_ki_out=Yaw_shell_ki*Yaw_i;
//    Yaw_d=MPU6050_GYRO_LAST.Z-Yaw_old;
//-------------Roll  PID-------------------//
	if((Time4on>229271)&&(Time4on<300000))
		RC_Yaw=((float)(Time4on-229271))/1966;//遥控往右，飞机顺时针
	else if((Time4on<229271)&&(Time4on>160000))
		RC_Yaw=-(((float)(229271-Time4on))/1966);
    Yaw_shell_out  = Yaw_shell_kp*((RC_Yaw+YAW_INIT)-Yaw);
//------------Roll角度保存------------------//
  //  Yaw_old=MPU6050_GYRO_LAST.Z;

////////////////////////内环角速度环(PD)///////////////////////////////
	MPU6050_GYRO_LAST_X=GYR_F_RATE.X-gyr_offset_f.X;
	MPU6050_GYRO_LAST_Y=GYR_F_RATE.Y-gyr_offset_f.Y;
	MPU6050_GYRO_LAST_Z=-GYR_F.Z;

//	Pitch_shell_out=RC_Pitch;
	pitch_core_kp_out = Pitch_core_kp * (Pitch_shell_out - MPU6050_GYRO_LAST_Y);
	pitch_core_ki_out += Pitch_core_ki * (Pitch_shell_out - MPU6050_GYRO_LAST_Y);//限幅
	//-------------Pitch积分限幅----------------//
		if(pitch_core_ki_out>9000) pitch_core_ki_out=9000;
		else if(pitch_core_ki_out<-9000) pitch_core_ki_out=-9000;
	pitch_core_kd_out = -Pitch_core_kd * (MPU6050_GYRO_LAST_Y   - Gyro_radian_old_y);

//	Roll_shell_out=RC_Roll;
	Roll_core_kp_out = Roll_core_kp * (Roll_shell_out  - MPU6050_GYRO_LAST_X);
	Roll_core_ki_out += Roll_core_ki * (Roll_shell_out - MPU6050_GYRO_LAST_X);//限幅
	//-------------Pitch积分限幅----------------//
		if(Roll_core_ki_out>9000) Roll_core_ki_out=9000;
		else if(Roll_core_ki_out<-9000) Roll_core_ki_out=-9000;
	Roll_core_kd_out = -Roll_core_kd * (MPU6050_GYRO_LAST_X - Gyro_radian_old_x);

	Yaw_core_kp_out  = Yaw_core_kp * (Yaw_shell_out - MPU6050_GYRO_LAST_Z);// * 1
	Yaw_core_ki_out += Yaw_core_ki * (Yaw_shell_out - MPU6050_GYRO_LAST_Z);// * 3.5
		if(Yaw_core_ki_out>1000) Yaw_core_ki_out=1000;
		else if(Yaw_core_ki_out<-1000) Yaw_core_ki_out=-1000;
//	Yaw_core_kd_out  = Yaw_core_kd * (MPU6050_GYRO_LAST_Z - Gyro_radian_old_z);

    Pitch_core_out = pitch_core_kp_out + pitch_core_ki_out + pitch_core_kd_out;
	Roll_core_out  = Roll_core_kp_out + Roll_core_ki_out + Roll_core_kd_out;
	Yaw_core_out  = Yaw_core_kp_out +Yaw_core_ki_out ;//+ Yaw_core_kd_out

	Gyro_radian_old_x = MPU6050_GYRO_LAST_X;
	Gyro_radian_old_y = MPU6050_GYRO_LAST_Y;
	Gyro_radian_old_z = MPU6050_GYRO_LAST_Z;   //储存历史值


//	Motor1=(int)(pwm_num3 + Roll_core_out - Pitch_core_out);//- pitch + Roll - yaw
	Motor1=(int)(YAW_INIT);//- pitch - Roll + yaw//MPU6050_GYRO_LAST_Z;
	Motor2=(int)(Yaw_core_out);//Pitch_shell_out;//Yaw;
	//	Motor1=(int)(1000);
	//Motor3=(int)(pwm_num3 - Roll_core_out + Pitch_core_out);//+ pitch - Roll - yaw
	//Motor4=(int)(pwm_num3 + Roll_core_out + Pitch_core_out);//+ pitch + Roll + yaw
	Motor3=(int)(pwm_num3 -  Roll_core_out + Pitch_core_out-Yaw_core_out);//RC_Pitch;//RC_Roll;
	//Yaw_core_out=0;
	Motor1_1=(int)(pwm_num3 + Roll_core_out -  Pitch_core_out-Yaw_core_out);//- pitch + Roll - yaw
	Motor2_1=(int)(pwm_num3 -  Roll_core_out -  Pitch_core_out+Yaw_core_out);//- pitch - Roll + yaw
	Motor3_1=(int)(pwm_num3 -  Roll_core_out + Pitch_core_out-Yaw_core_out);//+ pitch - Roll - yaw
	Motor4_1=(int)(pwm_num3 + Roll_core_out + Pitch_core_out+Yaw_core_out);//+ pitch + Roll + yaw

	/*Motor1=(int)(pwm_num3);//- pitch + Roll - yaw
	Motor2=(int)(pwm_num3);//- pitch - Roll + yaw
	Motor3=(int)(pwm_num3);//+ pitch - Roll - yaw
	Motor4=(int)(pwm_num3);//+ pitch + Roll + yaw*/

	/*Motor1=(int)(0);//- pitch + Roll - yaw
	Motor2=(int)(0);//- pitch - Roll + yaw
	Motor3=(int)(0);//+ pitch - Roll - yaw
	Motor4=(int)(0);//+ pitch + Roll + yaw*/

	Limit_amplitude();
	Refresh_Or_STOP_MOTOR(WARNING);//WARNING=1正常；WARNING=2警告
}
