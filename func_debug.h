/*
 * func_debug.h
 *
user functions for debug
 */

//overall header file of STM32F10x
#include "stm32f10x.h"

extern float M1_Sspeed;
extern float M1_Mspeed;

//export functions
void FSMC_SRAM_WriteBuffer_test(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);
void FSMC_SRAM_ReadBuffer_test(uint8_t* pBuffer, uint8_t ReadAddr, uint8_t NumByteToRead);

void MOTOR_Speed(void);

void HALL_Phase(void);

void SHOW_Dynamic(void);

