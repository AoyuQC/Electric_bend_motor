/**
  ******************************************************************************
  * @file    EXTI/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stdio.h"
#include "buffer.h"
#include "sch.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup EXTI_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t start_put = 0;
uint16_t M1_count = 0,M2_count = 0,M3_count = 0;
uint8_t count = 0;
uint8_t error = 0;
uint8_t remote_data[10] = {0,0,0,0,0,0,0,0,0,0};
extern uint8_t M1_speedPID[];
extern uint8_t INSTR;
extern uint8_t receive;
extern volatile FIFO_TypeDef U1Rx;
extern sTask SCH_Tasks_G[SCH_MAX_TASKS];
extern uint8_t M1_Speed_update;
extern uint8_t M2_Speed_update;
//debug
uint8_t RxBufferr[10]={0,0,0,0,0,0,0,0,0,0};
uint8_t Motor_Status[3]={0,0,0};
uint8_t indexx = 0;
uint8_t dir_count = 0;
uint8_t dir_count2 = 0;
uint8_t debug_count = 0;
uint8_t i = 0;
int16_t pole_count = 0;
uint8_t dir_stat;
uint8_t dir_stat2;

#define Bank1_SRAM2_ADDR    ((uint32_t)0x64000000)

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}


/**
  * @brief 这是调度器的中断服务程序，初始化函数中的定时器设置决定了它了调用频率
  * 这个版本由SysTick定时器中断触发，定时器每1ms触发一次
  */

/***********************************************************************************/
void SysTick_Handler(void)
{
	tByte Index;
        
	// SysTick定时器无须清除中断标志； 
	for(Index = 0;Index < SCH_MAX_TASKS;Index++)
	{
		if(SCH_Tasks_G[Index].pTask)     //表示确实有分配一个任务指针？
		{
			if(SCH_Tasks_G[Index].Delay == 0)
			{
				//任务需要运行
				SCH_Tasks_G[Index].RunMe +=1;	//“RunMe”标志加1
				if(SCH_Tasks_G[Index].Period)
				{
					//调度周期性的任务再次运行
					SCH_Tasks_G[Index].Delay = SCH_Tasks_G[Index].Period-1;//为什么减1：为了第period毫秒runme设为1，立刻执行
				}
			}				
			else
			{	
				//还没准备好允许，延迟减1
				SCH_Tasks_G[Index].Delay -= 1;
			}
		}
	}
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  //interrupt from motor 1 hall sensor
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
    TIM_Cmd(TIM2, DISABLE);
    M1_count = TIM2->CNT;
    TIM2->CNT &= 0x00000000;
    //indicate a new speed value
    dir_stat =  *(__IO uint8_t*) (Bank1_SRAM2_ADDR + 16);
    int dir = dir_stat>>0x7;
    if (dir == 1)
      pole_count++;
    else 
      pole_count--;
    
    M1_Speed_update = 1;
    EXTI_ClearITPendingBit(EXTI_Line6);
    TIM_Cmd(TIM2,ENABLE);
  }
}

/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  //interrupt from motor 2 hall sensor
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    TIM_Cmd(TIM4, DISABLE);
    M2_count = TIM4->CNT;
    TIM4->CNT &= 0x00000000;
    //indicate a new speed value
    dir_stat2 =  *(__IO uint8_t*) (Bank1_SRAM2_ADDR + 17);
    M2_Speed_update = 1;
    EXTI_ClearITPendingBit(EXTI_Line12);
    TIM_Cmd(TIM4,ENABLE);
  }
  //interrupt from motor 3 hall sensor
  if(EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    TIM_Cmd(TIM5, DISABLE);
    M3_count = TIM5->CNT; 
    TIM5->CNT &= 0x00000000;    
    EXTI_ClearITPendingBit(EXTI_Line13);
    TIM_Cmd(TIM5,ENABLE);
  }
}



/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  { 
    //printf("interrupt \n");
    /* Read one byte from the receive data register */
    //RxBufferr[indexx++] = USART_ReceiveData(USART1);
    //RxBufferr[indexx++] = USART_ReceiveData(USART1); 
    remote_data[indexx] = (USART_ReceiveData(USART1) & 0x7F);
    //BufferPut(&U1Rx, RxBufferr[indexx]);
    //printf("ch is %c",ch);
    if(U1Rx.count < USARTBUFFSIZE)
    {
      if(remote_data[indexx] == '$')
      {
        //printf("run to here");
        start_put = 1;
        BufferPut(&U1Rx, remote_data[indexx++]);
      }
      else if(start_put == 1)
      {
        /*if(remote_data[indexx] == 'F')
        {
          //GPIOF->ODR |= 0x00000400;
          //GPIOF->ODR &= 0xFFFFFEFF;
        }
        else if(remote_data[indexx] == 'R')
        {
          //GPIOF->ODR &= 0xFFFFFBFF;
          //GPIOF->ODR |= 0x00000100;
        }*/
        BufferPut(&U1Rx, remote_data[indexx++]);
        if(remote_data[--indexx]  == '*')
        {
          //printf("finish one receive");
          start_put = 0;
          indexx = 0;
          dir_count++;
        }
     }
    }
    else
    {
      error = 1;//printf("BufferGet fail : run of U1Rx \n");
    }
    
    //printf("\n get new value %d",INSTR); debug purpose
    //USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
  
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART1, M1_speedPID[i++]);

    if(i == 16)
    {
      i = 0;
      /* Disable the USART1 Transmit interrupt */
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }    
  }
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
