#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "capture.h"

// Configure the start/end period for the timer
Uint32 Time1on=0,Time1off=0,Time2on=0,Time2off=0,Time3on=0,Time3off=0,Time4on=0,Time4off=0,Time5on=0,Time5off=0,Time6on=0,Time6off=0,T1=0,T2=0,T3=0,T4=0;
int WARNING=0;
// Prototype statements for functions found within this file.

void InitECapture1()
{
   ECap1Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap1Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
   
   // Configure peripheral registers
   ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0;      // 连续
//   ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;        // Stop at 4 events
   ECap1Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP2POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap1Regs.ECCTL1.bit.CAP4POL = 1;          // Falling edge
   ECap1Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation         
   ECap1Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation         
   ECap1Regs.ECCTL2.bit.SYNCI_EN = 0;         // disable sync in
   ECap1Regs.ECCTL2.bit.SYNCO_SEL = 2;        // EC_SYNCO_DIS
   ECap1Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units
   ECap1Regs.ECEINT.bit.CEVT2 = 1; 
   ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;
}
void InitECapture2()
{
   ECap2Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap2Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap2Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
   ECap2Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
   
   // Configure peripheral registers
   ECap2Regs.ECCTL2.bit.CONT_ONESHT = 0;      // 连续
//   ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;        // Stop at 4 events
   ECap2Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap2Regs.ECCTL1.bit.CAP2POL = 1;          // Falling edge
   ECap2Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap2Regs.ECCTL1.bit.CAP4POL = 1;          // Falling edge
   ECap2Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation         
   ECap2Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation         
   ECap2Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation         
   ECap2Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation         
   ECap2Regs.ECCTL2.bit.SYNCI_EN = 0;         // disable sync in
   ECap2Regs.ECCTL2.bit.SYNCO_SEL = 2;        // EC_SYNCO_DIS
   ECap2Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units
   ECap2Regs.ECEINT.bit.CEVT2 = 1; 
   ECap2Regs.ECCTL2.bit.TSCTRSTOP = 1;
}
void InitECapture3()
{
   ECap3Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap3Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap3Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
   ECap3Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
   
   // Configure peripheral registers
   ECap3Regs.ECCTL2.bit.CONT_ONESHT = 0;      // 连续
//   ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;        // Stop at 4 events
   ECap3Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap3Regs.ECCTL1.bit.CAP2POL = 1;          // Falling edge
   ECap3Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap3Regs.ECCTL1.bit.CAP4POL = 1;          // Falling edge
   ECap3Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation         
   ECap3Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation         
   ECap3Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation         
   ECap3Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation         
   ECap3Regs.ECCTL2.bit.SYNCI_EN = 0;         // disable sync in
   ECap3Regs.ECCTL2.bit.SYNCO_SEL = 2;        // EC_SYNCO_DIS
   ECap3Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units
   ECap3Regs.ECEINT.bit.CEVT2 = 1; 
   ECap3Regs.ECCTL2.bit.TSCTRSTOP = 1;
}
void InitECapture4()
{
   ECap4Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap4Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap4Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
   ECap4Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
   
   // Configure peripheral registers
   ECap4Regs.ECCTL2.bit.CONT_ONESHT = 0;      // 连续
//   ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;        // Stop at 4 events
   ECap4Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap4Regs.ECCTL1.bit.CAP2POL = 1;          // Falling edge
   ECap4Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap4Regs.ECCTL1.bit.CAP4POL = 1;          // Falling edge
   ECap4Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation         
   ECap4Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation         
   ECap4Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation         
   ECap4Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation         
   ECap4Regs.ECCTL2.bit.SYNCI_EN = 0;         // disable sync in
   ECap4Regs.ECCTL2.bit.SYNCO_SEL = 2;        // EC_SYNCO_DIS
   ECap4Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units
   ECap4Regs.ECEINT.bit.CEVT2 = 1; 
   ECap4Regs.ECCTL2.bit.TSCTRSTOP = 1;
}
void InitECapture5()
{
   ECap5Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap5Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap5Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
   ECap5Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
   
   // Configure peripheral registers
   ECap5Regs.ECCTL2.bit.CONT_ONESHT = 0;      // 连续
//   ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;        // Stop at 4 events
   ECap5Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap5Regs.ECCTL1.bit.CAP2POL = 1;          // Falling edge
   ECap5Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap5Regs.ECCTL1.bit.CAP4POL = 1;          // Falling edge
   ECap5Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation         
   ECap5Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation         
   ECap5Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation         
   ECap5Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation         
   ECap5Regs.ECCTL2.bit.SYNCI_EN = 0;         // disable sync in
   ECap5Regs.ECCTL2.bit.SYNCO_SEL = 2;        // EC_SYNCO_DIS
   ECap5Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units
   ECap5Regs.ECEINT.bit.CEVT2 = 1; 
   ECap5Regs.ECCTL2.bit.TSCTRSTOP = 1;
}
void InitECapture6()
{
   ECap6Regs.ECEINT.all = 0x0000;             // Disable all capture interrupts
   ECap6Regs.ECCLR.all = 0xFFFF;              // Clear all CAP interrupt flags
   ECap6Regs.ECCTL1.bit.CAPLDEN = 0;          // Disable CAP1-CAP4 register loads
   ECap6Regs.ECCTL2.bit.TSCTRSTOP = 0;        // Make sure the counter is stopped
   
   // Configure peripheral registers
   ECap6Regs.ECCTL2.bit.CONT_ONESHT = 0;      // 连续
//   ECap1Regs.ECCTL2.bit.STOP_WRAP = 3;        // Stop at 4 events
   ECap6Regs.ECCTL1.bit.CAP1POL = 0;          // Rising edge
   ECap6Regs.ECCTL1.bit.CAP2POL = 1;          // Falling edge
   ECap6Regs.ECCTL1.bit.CAP3POL = 0;          // Rising edge
   ECap6Regs.ECCTL1.bit.CAP4POL = 1;          // Falling edge
   ECap6Regs.ECCTL1.bit.CTRRST1 = 1;          // Difference operation         
   ECap6Regs.ECCTL1.bit.CTRRST2 = 1;          // Difference operation         
   ECap6Regs.ECCTL1.bit.CTRRST3 = 1;          // Difference operation         
   ECap6Regs.ECCTL1.bit.CTRRST4 = 1;          // Difference operation         
   ECap6Regs.ECCTL2.bit.SYNCI_EN = 0;         // disable sync in
   ECap6Regs.ECCTL2.bit.SYNCO_SEL = 2;        // EC_SYNCO_DIS
   ECap6Regs.ECCTL1.bit.CAPLDEN = 1;          // Enable capture units
   ECap6Regs.ECEINT.bit.CEVT2 = 1; 
   ECap6Regs.ECCTL2.bit.TSCTRSTOP = 1;
}
interrupt void ecap1_isr(void)
{

   ECap1Regs.ECCLR.all=0xFFFF;//clare all flag
	Time1on= ECap1Regs.CAP2;//Time1on
	Time1off= ECap1Regs.CAP3;//Time1off

	//Time2on= ECap1Regs.CAP4;//Time2on
  	//Time2off= ECap1Regs.CAP1; //Time2off 另外三种方式都是按顺序的
 //   T1=t2-t1;T2=t3-t2;T3=t3-t1;//T4=t4-t1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
interrupt void ecap2_isr(void)
{

   ECap2Regs.ECCLR.all=0xFFFF;//clare all flag
	Time2on= ECap2Regs.CAP2;//Time1on
	Time2off= ECap2Regs.CAP3;//Time1off
	//Time2on= ECap2Regs.CAP4;//Time2on
  	//Time2off= ECap2Regs.CAP1; //Time2off 另外三种方式都是按顺序的
 //   T1=t2-t1;T2=t3-t2;T3=t3-t1;//T4=t4-t1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
interrupt void ecap3_isr(void)
{

   ECap3Regs.ECCLR.all=0xFFFF;//clare all flag
	Time3on= ECap3Regs.CAP2;//Time1on
	Time3off= ECap3Regs.CAP3;//Time1off
	
	//Time2on= ECap1Regs.CAP4;//Time2on
  	//Time2off= ECap1Regs.CAP1; //Time2off 另外三种方式都是按顺序的
 //   T1=t2-t1;T2=t3-t2;T3=t3-t1;//T4=t4-t1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
interrupt void ecap4_isr(void)
{

   ECap4Regs.ECCLR.all=0xFFFF;//clare all flag
	Time4on= ECap4Regs.CAP2;//Time1on
	Time4off= ECap4Regs.CAP3;//Time1off
	//Time2on= ECap2Regs.CAP4;//Time2on
  	//Time2off= ECap2Regs.CAP1; //Time2off 另外三种方式都是按顺序的
 //   T1=t2-t1;T2=t3-t2;T3=t3-t1;//T4=t4-t1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
interrupt void ecap5_isr(void)
{

   ECap5Regs.ECCLR.all=0xFFFF;//clare all flag
	Time5on= ECap5Regs.CAP2;//Time1on
	Time5off= ECap5Regs.CAP3;//Time1off
	if((Time5on>140000)&&(Time5on<150000))//WARNING=1正常；WARNING=2警告          144195
		WARNING=1;
	else WARNING=2;//if((Time5on>200000)&&(Time5on<350000))//305046通道5手动关电机        228148信号丢失关电机     直接没信号pwm会飙到最大

	//Time2on= ECap1Regs.CAP4;//Time2on
  	//Time2off= ECap1Regs.CAP1; //Time2off 另外三种方式都是按顺序的
 //   T1=t2-t1;T2=t3-t2;T3=t3-t1;//T4=t4-t1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
interrupt void ecap6_isr(void)
{

   ECap6Regs.ECCLR.all=0xFFFF;//clare all flag
	Time6on= ECap6Regs.CAP2;//Time1on
	Time6off= ECap6Regs.CAP3;//Time1off
	//Time2on= ECap2Regs.CAP4;//Time2on
  	//Time2off= ECap2Regs.CAP1; //Time2off 另外三种方式都是按顺序的
 //   T1=t2-t1;T2=t3-t2;T3=t3-t1;//T4=t4-t1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}
