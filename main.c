/**
  ******************************************************************************
  * @file    main
  * @author  zay
  * @version V1.0
  * @date    04/17/2013
  * @brief   Main program body
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "buffer.h"
#include "sch.h"

/* temp for debug-------------------------------------------------------------*/
#include "Elec_bend.h"
#define BUFFER_SIZE        14
#define WRITE_READ_ADDR    0x00
#include "stdio.h"

uint8_t TestRBuffer[BUFFER_SIZE];
extern uint8_t TxBuffer[19];

MOTOR_ControlTypeDef M_C;
//initialize buffers
volatile FIFO_TypeDef U1Rx; 

//debug Aohua mulitask scheduling methods

uint8_t receive;
//uint8_t TxBuffer[BUFFER_SIZE];
uint32_t WriteReadStatus = 0, Index = 0;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main()
{
  //initial hardware configuraiton
  Hardware_Configuration();
  
  //initialize buffers & variables
  BufferInit(&U1Rx);
  receive = 0;
  
  //Debug
  //Turn ALL LED OFF
  GPIOF->BSRR |= 0x01400000;

  M_C.BRAKE  = ON;
  M_C.ENABLE = OFF;
  M_C.FWDREV = FWD;
  M_C.POWER_Supply = ON;
  M_C.Count_value = 2500;
  M_C.Duty_value = 25000 - 2857; //5RPS
  
  MOTOR_Control(M1,&M_C,CONTROL|SPEED);
  MOTOR_Control(M2,&M_C,CONTROL|SPEED);
  
  //Reset endoscopy position
  Reset_endoscopy_position();
  
  //Test check pole count per round
  //Check_pole_count();
  
  SCH_Init_Task();
  SCH_Add_Task(INSTR_Parse,0,1);
  SCH_Add_Task(MOTOR_PID_Adjust,1,1);
  SCH_Add_Task(SHOW_Dynamic,2,1000);
  //SCH_Add_Task(MOTOR_Speed,3,10);
  //printf("start test \n");
  
  while(1)
  {
    SCH_Dispatch_Tasks();
  }
}
  


 
 
 
 

  
  

  
  
  
