/**
  ******************************************************************************
  * @file    func_debug.c 
  * @author  zay
  * @version V1.0
  * @date    
  * @brief   1.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "func_debug.h"
#include "stdio.h"
    
/* Private define ------------------------------------------------------------*/
#define Bank1_SRAM2_ADDR    ((uint32_t)0x64000000)

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */  
    
    
/***********************************************************************************/
#ifdef  USE_FULL_ASSERT

uint8_t lock;
uint8_t M1_speedPID[15] = {'S','0','0','0','1','2','3','4','5','6','7','8','9','A','*'};
extern uint16_t M1_count;
float test = 5;
uint8_t flag = 1;

extern float P_Factor;
extern float I_Factor;
extern float D_Factor;
extern float M1_Mspeed;
extern float M2_Mspeed;

extern uint16_t round_count;
extern float error_history;
extern float thisError;
extern uint16_t Calc_send;
extern float delta_this_error;
extern uint16_t M1_PWM;
extern uint16_t M2_PWM;
extern float thisError;
extern float LastErrorValue;	
extern float delta_this_error;
extern uint8_t dir;
extern uint8_t dir2;

//PID
extern uint8_t M1_FWD_flag;
extern uint8_t M1_REV_flag;
extern uint8_t neg_logic;

//AD
extern uint16_t ADCConvertedValue;
int AD_dir = 1;
extern int16_t pole_count;

//Range detect
extern int m1_range;
extern uint8_t M1_INSTR_BACK;
extern int change_status;
  



/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	printf("Wrong parameters value: file %s on line %d\r\n", file, line);

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}


/**
  * @brief  Writes a Half-word buffer to the FSMC SRAM memory. 
  * @param  pBuffer : pointer to buffer. 
  * @param  WriteAddr : SRAM memory internal address from which the data 
  *        will be written.
  * @param  NumHalfwordToWrite : number of half-words to write. 
  * @retval None
  */
void FSMC_SRAM_WriteBuffer_test(uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
  for(; NumByteToWrite != 0; NumByteToWrite--) /* while there is data to write */
  {
    /* Transfer data to the memory */
    *(uint8_t *) (Bank1_SRAM2_ADDR + WriteAddr) = *pBuffer++;
    
    /* Increment the address*/  
    WriteAddr += 1;
  }   
}

/**
  * @brief  Reads a block of data from the FSMC SRAM memory.
  * @param  pBuffer : pointer to the buffer that receives the data read 
  *        from the SRAM memory.
  * @param  ReadAddr : SRAM memory internal address to read from.
  * @param  NumHalfwordToRead : number of half-words to read.
  * @retval None
  */
void FSMC_SRAM_ReadBuffer_test(uint8_t* pBuffer, uint8_t ReadAddr, uint8_t NumByteToRead)
{
  for(; NumByteToRead != 0; NumByteToRead--) /* while there is data to read */
  {
    /* Read a half-word from the memory */
    *pBuffer++ = *(__IO uint8_t*) (Bank1_SRAM2_ADDR + ReadAddr);

    /* Increment the address*/  
    ReadAddr += 1;
  }  
}

/**
  * @brief  show speed
  * @param  
  * @retval
  */
void MOTOR_Speed()
{  
  //printf("%f %d\n",M1_Mspeed,M1_count);
  uint8_t num_value[10] = {'0','1','2','3','4','5','6','7','8','9'};
//  uint8_t speed_bit1 = M1_Mspeed/10;
//  uint8_t speed_bit0 = M1_Mspeed - speed_bit1*10;
  
  uint8_t speed_bit1 = M1_Mspeed/10;
  uint8_t speed_bit0 = M1_Mspeed - speed_bit1*10;
//  uint8_t speed_bit1 = test/10;
//  uint8_t speed_bit0 = test - speed_bit1*10;
  /*uint8_t P_bit3 = P_Factor/100;
  uint8_t P_bit2 = (P_Factor - P_bit3*100)/10;
  uint8_t P_bit1 = P_Factor - P_bit3*100 - P_bit2*10;
  
  uint8_t I_bit3 = I_Factor/100;
  uint8_t I_bit2 = (I_Factor - I_bit3*100)/10;
  uint8_t I_bit1 = I_Factor - I_bit3*100 - I_bit2*10;
  
  uint8_t D_bit4 = D_Factor/1000;
  uint8_t D_bit3 = (D_Factor - D_bit4*1000)/100;
  uint8_t D_bit2 = (D_Factor - D_bit4*1000 - D_bit3*100)/10;
  uint8_t D_bit1 = D_Factor - D_bit4*1000 - D_bit3*100 - D_bit2*10;*/
  
  uint8_t P_bit3 = M1_PWM/10000;
  uint8_t P_bit2 = (M1_PWM - P_bit3*10000)/1000;
  uint8_t P_bit1 = (M1_PWM - P_bit3*10000 - P_bit2*1000)/100;
  
  uint8_t I_bit3 = (M1_PWM - P_bit3*10000 - P_bit2*1000 - P_bit1*100)/10;
  uint8_t I_bit2 = M1_PWM - P_bit3*10000 - P_bit2*1000 - P_bit1*100 - I_bit3*10;
  uint8_t I_bit1;
  
  /*TR_num++;
  P_bit3 = M1_PWM/10000;
  P_bit2 = (M1_PWM - P_bit3*10000)/1000;
  P_bit1 = (M1_PWM - P_bit3*10000 - P_bit2*1000)/100;
  I_bit3 = (M1_PWM - P_bit3*10000 - P_bit2*1000 - P_bit1*100)/10;
  uint8_t I_bit2 = M1_PWM - P_bit3*10000 - P_bit2*1000 - P_bit1*100 - I_bit3*10;
  uint8_t I_bit1;*/  
  
  int change_t = (change_status >> 1) & 0x1;
  int change_o = change_status & 0x1;
  
  I_bit2 = change_t;
  I_bit1 = change_o;
 
  uint8_t D_bit4 = error_history/1000;
  uint8_t D_bit3 = (error_history - D_bit4*1000)/100;
  uint8_t D_bit2 = (error_history - D_bit4*1000 - D_bit3*100)/10;
  uint8_t D_bit1 = error_history - D_bit4*1000 - D_bit3*100 - D_bit2*10;
  
  //for ADC transfer data
  uint16_t ADCConvertedValue_8bit;
  ADCConvertedValue_8bit = ADCConvertedValue;
  D_bit4 = ADCConvertedValue_8bit/1000;
  D_bit3 = (ADCConvertedValue_8bit - D_bit4 * 1000) / 100;
  D_bit2 = (ADCConvertedValue_8bit - D_bit4 * 1000 - D_bit3*100)/10;
  D_bit1 = ADCConvertedValue_8bit - D_bit4 * 1000 - D_bit3*100 - D_bit2*10;
  
  /*uint16_t fake_ADCConvertedValue = 2350;
  D_bit4 = fake_ADCConvertedValue/1000;
  D_bit3 = (fake_ADCConvertedValue - D_bit4*1000)/100;
  D_bit2 = (fake_ADCConvertedValue - D_bit4*1000 - D_bit3*100)/10;
  D_bit1 = fake_ADCConvertedValue - D_bit4*1000 - D_bit3*100 - D_bit2*10;*/
 
  //speed value
  M1_speedPID[3] = num_value[speed_bit0];
  M1_speedPID[2] = num_value[speed_bit1];
  M1_speedPID[1] = num_value[dir];
  //PID value
  //debug serial communication
  M1_speedPID[4] = num_value[P_bit3];
  M1_speedPID[5] = num_value[P_bit2];
  M1_speedPID[6] = num_value[P_bit1];
  M1_speedPID[7] = num_value[I_bit3];
  M1_speedPID[8] = num_value[I_bit2];
  M1_speedPID[9] = num_value[I_bit1];
  M1_speedPID[10] = num_value[D_bit4];
  M1_speedPID[11] = num_value[D_bit3];
  M1_speedPID[12] = num_value[D_bit2];
  M1_speedPID[13] = num_value[D_bit1];
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  
  /*if(fake_ADCConvertedValue >= 1000)
    AD_dir = 0;
  
  if(fake_ADCConvertedValue <= 900)
    AD_dir = 1;
  
  if(AD_dir == 1)
    fake_ADCConvertedValue = fake_ADCConvertedValue + 1;
  else if(AD_dir == 0)
    fake_ADCConvertedValue = fake_ADCConvertedValue - 8;*/
  
}

/**
  * @brief  show analog value corresponding to speed measure
  * @param  
  * @retval
  */
void HALL_Phase()
{
  if(lock == 0)
  {
    lock = 1;
    GPIOG->BRR |= 0x00000001;
  }
}

void SHOW_Dynamic()
{
  //printf("M1 : %f thisError : %f LastErrorValue : %f delta_this_error : %f \n",M1_Mspeed,thisError,LastErrorValue,delta_this_error);
  //printf("M1: %f M1_PWM : %d \n", M1_Mspeed,M1_PWM);
  //printf("M2: %f M2_PWM : %d \n", M2_Mspeed,M2_PWM);
  //printf("m1_range is %d \n", m1_range);
  printf(" \n M1_INSTR_BACK = %d and M1_FWD_flag = %d and M1_REV_flag = %d \n", M1_INSTR_BACK, M1_FWD_flag, M1_REV_flag);
  
  //ADC function
  //printf("ADC: %d \n", ADCConvertedValue);
  /*if(fake_ADCConvertedValue >= 300)
    AD_dir = 0;
  
  if(fake_ADCConvertedValue <= 274)
    AD_dir = 1;
  
  if(AD_dir == 1)
    fake_ADCConvertedValue = fake_ADCConvertedValue + 1;
  else if(AD_dir == 0)
    fake_ADCConvertedValue = fake_ADCConvertedValue - 1;*/
    
  //if(neg_logic != 0)
  //printf("dir: %d FWD_flag: %d REV_flag : %d \n", dir, M1_FWD_flag, M1_REV_flag);
  //uint8_t dir = dir_stat>>0x7; 
  //printf("dir: %d \n", dir);
  /*uint8_t num_value[10] = {'0','1','2','3','4','5','6','7','8','9'};
  uint8_t count_bit5 = M1_PWM/10000;
  uint8_t count_bit4 = (M1_PWM - count_bit5*10000)/1000;
  uint8_t count_bit3 = (M1_PWM - count_bit5*10000 - count_bit4*1000)/100;
  uint8_t count_bit2 = (M1_PWM - count_bit5*10000 - count_bit4*1000 - count_bit3*100)/10;
  uint8_t count_bit1 = M1_PWM - count_bit5*10000 - count_bit4*1000 - count_bit3*100 - count_bit2*10;

  //PID value
  M1_speedPID[1] = num_value[count_bit5];
  M1_speedPID[2] = num_value[count_bit4];
  M1_speedPID[3] = num_value[count_bit3];
  M1_speedPID[4] = num_value[count_bit2];
  M1_speedPID[5] = num_value[count_bit1];
  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);*/
}





