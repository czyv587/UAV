//时钟为150MHz，HSPCLKDIV，CLKDIV分别4分频，PWM时钟为150MHz/（4*4）
//周期为400Hz，则pwm计数则为23436，PWM高电平1ms，则需计数9375次，2ms则需9375*2次
//如何根据此时的遥控脉宽得到相应的PWM？还有如下要求：
//1.pwm（0）时对应的pwm高电平计数为9375
//2.pwm(最高)时对应的pwm高电平计数为9375*2
//3.遥控的脉宽差和pwm的脉宽差应该是成正比
//void PWM_CH1(long t)//t的范围就是0<t<9375
//{
//	t=9375+t;//
//	EPwm1Regs.CMPA.half.CMPA = t;
//}
//总的表达式就是：9375+(遥控最低时脉宽-当前脉宽)/(遥控最低脉宽时间-遥控最高脉宽时间)*9375
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "pwm.h"

void InitEPwm1Example()
{
   // Setup TBCLK
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBPRD = 23436;//EPWM1_TIMER_TBPRD;       // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0x2;   // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = 0x2;

   // Setup shadow register load on ZERO
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = 0;//EPWM1_MIN_CMPA;    // Set compare A value
   EPwm1Regs.CMPB = 0;              // Set Compare B value

   // Set actions
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count

   EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B, up count

}
void InitEPwm2Example()
{
   // Setup TBCLK
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm2Regs.TBPRD = 23436;       // Set timer period=2.5ms
   EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0x2;   // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = 0x2;

   // Setup shadow register load on ZERO
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = 0;       // Set compare A value
   EPwm2Regs.CMPB = 0;                 // Set Compare B value

   // Set actions
   EPwm2Regs.AQCTLA.bit.PRD = AQ_SET;             // Clear PWM2A on Period
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;               // Set PWM2A on event A, up count

   EPwm2Regs.AQCTLB.bit.PRD = AQ_SET;             // Clear PWM2B on Period
   EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;               // Set PWM2B on event B, up count
}
void pwm_init()
{
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   InitEPwm1Example();
   InitEPwm2Example();

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;
}
void PWM_CH1(long t)
{
	t=9375+t;//
	EPwm1Regs.CMPA.half.CMPA = t;
}
void PWM_CH2(long t)
{
	t=9375+t;
	EPwm1Regs.CMPB = t;
}
void PWM_CH3(long t)
{
	t=9375+t;
	EPwm2Regs.CMPA.half.CMPA = t;
}
void PWM_CH4(long t)
{
	t=9375+t;
	EPwm2Regs.CMPB = t;
}
