
#ifndef STM32F103_PID_H
#define STM32F103_PID_H
#include "stm32f10x.h"

void Speed_PI_Adjust(void);                            // ���1��speed adust
void SpeedSetValue(float period);                        // ת�ٸ����ο�ֵ

void Speed_PI_Adjust2(void);                            // ���2
void SpeedSetValue2(float period);
extern float xx;
extern vs32 dac_ave_value;
void GetDacValue(void);
extern vs32 OutPut;
#endif
