#ifndef CAPTURE_H_
#define CAPTURE_H_

interrupt void ecap1_isr(void);
interrupt void ecap2_isr(void);
interrupt void ecap3_isr(void);
interrupt void ecap4_isr(void);
interrupt void ecap5_isr(void);
interrupt void ecap6_isr(void);
void InitECapture1(void);
void InitECapture2(void);
void InitECapture3(void);
void InitECapture4(void);
void InitECapture5(void);
void InitECapture6(void);

#endif /*CAPTURE_H_*/
