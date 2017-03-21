#ifndef CONTROL_H_
#define CONTROL_H_

void PID_INIT(void)	;
void COMP_TX_BUF(float pitch_or_rool,float gyrs,float comd,float exp_rate,unsigned char*tx);
void READ_CONTROL_COMMAND(unsigned char* tmp_buf);
void STABLE_WITH_PID(void);
void CRASH_LANDING(void);
void Limit_amplitude();
void Refresh_Or_STOP_MOTOR(unsigned char sel);
void EXP_SMOOTH(unsigned char tms);
void control(void);
void control1(void);
void control2(void);
#endif /*CONTROL_H_*/
