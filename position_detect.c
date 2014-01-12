/**
  ******************************************************************************
  * @file    position_detect.c 
  * @author  zay
  * @version V1.0
  * @date    
  * @brief   1.Reset_endoscopy_position; 2.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
//header file of motor control; solenoid valve control; pump control
#include "motor_solenoid_pump.h"
#include "stdio.h"

int M1_li = 1;
int M1_RANGE = 0x08;
int change_status = 2;
uint8_t M1_INSTR_BACK = 0;
uint8_t li_change = 0;
uint8_t range_lock = 0;
uint8_t half_round = 0;

extern MOTOR_ControlTypeDef M_C;
extern int16_t pole_count;
extern uint8_t M1_FWD_flag;
extern uint8_t M1_REV_flag;

/* Private define ------------------------------------------------------------*/
//conditions for position detection
//state machine
#define M1_RST_REV        0x01
#define M1_RST_FWD        0x02
#define M1_RST_FINISH     0x03
#define M1_START_CROSS    0x04
#define M1_1_CROSS        0x05
#define M1_2_CROSS        0x06
#define M1_3_CROSS        0x07
#define M1_OFF_RANGE      0x08
#define M1_ON_RANGE       0x09
#define M1_LEAVE_RANGE    0x0A
//range_check
#define ON_RANGE          0x10
#define OFF_RANGE         0x11



#define Bank1_SRAM2_ADDR    ((uint32_t)0x64000000)

//uint8_t Read_mx_light(int Mx)

//To reset position of endoscopy
void Reset_endoscopy_position()
{
  //reset position of motor1
  //detect signal of light sensor
  uint8_t temp_stat =  *(__IO uint8_t*) (Bank1_SRAM2_ADDR + 16);
  temp_stat &= 0x40;
  M1_li = temp_stat >> 0x6;
  //initial reset operation
  int RST_OP;
  if(M1_li == 1)
    RST_OP = M1_RST_REV;
  else if(M1_li == 0)
    RST_OP = M1_RST_FWD;
     
  //initial reset status
  int reset_status = 0;
  
  M_C.BRAKE  = OFF;
  M_C.ENABLE = ON;
  
  while(reset_status == 0)
  {
    switch(RST_OP)
    {
      case M1_RST_REV:
      {
        M_C.FWDREV = REV;
        MOTOR_Control(M1,&M_C,CONTROL);
        
        //read light status
        temp_stat =  *(__IO uint8_t*) (Bank1_SRAM2_ADDR + 16);
        temp_stat &= 0x40;
        M1_li = temp_stat >> 0x6;
        if(M1_li == 1)
          RST_OP = M1_RST_REV;
        else if(M1_li == 0)
          RST_OP = M1_RST_FINISH;
        break;
      }
      case M1_RST_FWD:
      {
        M_C.FWDREV = FWD;
        MOTOR_Control(M1,&M_C,CONTROL);
        
        //read light status
        temp_stat =  *(__IO uint8_t*) (Bank1_SRAM2_ADDR + 16);
        temp_stat &= 0x40;
        M1_li = temp_stat >> 0x6;
        if(M1_li == 1)
          RST_OP = M1_RST_FINISH;
        else if(M1_li == 0)
          RST_OP = M1_RST_FWD;
        break;
      }
      case M1_RST_FINISH:
      {
        reset_status = 1;
        break;
      }
      default :
        break;
    }
  }
  
  //brake motor1
  M_C.BRAKE = ON;
  MOTOR_Control(M1,&M_C,CONTROL);
  pole_count = 0;
}

//Check pole count per round
void Check_pole_count()
{
  pole_count = 0;
  //begin with cross state
  int M1_CHECK = M1_START_CROSS;
  int M1_INI = M1_li;
  int M1_1_state = M1_li;
  int M1_2_state = 1 - M1_li;
  int M1_3_state = M1_li;
  int test_num = 1;
  int temp_stat = M1_li;
  M_C.BRAKE = OFF;
  while(1)
  {
    switch(M1_CHECK)
    {
      case M1_START_CROSS:
      {
        printf("\n round %d \n", test_num);
        //initial check status
        if(M1_INI == 1)
        {
          printf(" initial state is black \n");
          M_C.FWDREV = REV;
        }
        if(M1_INI == 0)
        {
          printf(" initial state is white \n");
          M_C.FWDREV = FWD;
        }
        MOTOR_Control(M1,&M_C,CONTROL);
        M1_CHECK = M1_1_CROSS;
        break;
      }
      case M1_1_CROSS:
      {
        //read light status
        temp_stat =  *(__IO uint8_t*) (Bank1_SRAM2_ADDR + 16);
        temp_stat &= 0x40;
        M1_1_state = temp_stat >> 0x6;
        if((M1_INI == 1 && M1_1_state == 0)||(M1_INI == 0 && M1_1_state == 1))
        {
          printf(" 1st cross and pole is %d \n", pole_count);
          M1_CHECK = M1_2_CROSS;
        }
        break;
      }
      case M1_2_CROSS:
      {
        //read light status
        temp_stat =  *(__IO uint8_t*) (Bank1_SRAM2_ADDR + 16);
        temp_stat &= 0x40;
        M1_2_state = temp_stat >> 0x6;
        if((M1_1_state == 0 && M1_2_state == 1)||(M1_1_state == 1 && M1_2_state == 0))
        {
          printf(" 2nd cross and pole is %d \n", pole_count);
          M1_CHECK = M1_3_CROSS;
        }
        break;
      }
      case M1_3_CROSS:
      {
        //read light status
        temp_stat =  *(__IO uint8_t*) (Bank1_SRAM2_ADDR + 16);
        temp_stat &= 0x40;
        M1_3_state = temp_stat >> 0x6;
        if((M1_2_state == 1 && M1_3_state == 0)||(M1_2_state == 0 && M1_3_state == 1))
        {
          printf(" 3rd cross and pole is %d \n", pole_count);
          pole_count = 0;
          test_num++;
          M1_INI = 1 - M1_INI;
          M1_1_state = M1_INI;
          M1_2_state = 1 - M1_2_state;
          M1_3_state = 1 - M1_3_state;
          M1_CHECK = M1_START_CROSS;          
        }
        break;
      }
      default:
        break;
    }
  }
}

//Check move range
int Range_check(int Mx)
{
  //detect motor1 move range
  //motor 1
  int return_state;
  if (Mx == M1)
  {
      //detect signal of light sensor
      uint8_t temp_stat =  *(__IO uint8_t*) (Bank1_SRAM2_ADDR + 16);
      temp_stat &= 0x40;
      uint8_t M1_li_cur = temp_stat >> 0x6;
    
      //show status
      int change_h = change_status >> 2;
      int change_t = (change_status >> 1) & 0x1;
      int change_o = change_status & 0x1;
      if (change_o == 1)
        //LED 1 D39 on
        GPIOF->ODR |= 0x00000040;
      else if(change_o == 0)
        //LED 1 D39 off
        GPIOF->ODR &= 0xFFFFFFBF;
      
      if (change_t == 1)
        //LED 2 D40 on
        GPIOF->ODR |= 0x00000100;
      else if(change_t == 0)
        //LED 2 D40 off
        GPIOF->ODR &= 0xFFFFFEFF;
      
      if (change_h == 1)
        //LED 3 D41 on
        GPIOF->ODR |= 0x00000400;
      else if(change_h == 0)
        //LED 3 D41 off
        GPIOF->ODR &= 0xFFFFFBFF;
      
      
      /*if (change_status == 0)
        //LED 2 D40 on
        GPIOF->ODR |= 0x00000100;
      else 
        //LED 2 D40 off
        GPIOF->ODR &= 0xFFFFFEFF;
      
      if (change_status == 4)
        //LED 3 D41 on
        GPIOF->ODR |= 0x00000400;
      else 
        //LED 3 D41 off
        GPIOF->ODR &= 0xFFFFFBFF;*/
 
      switch (M1_RANGE)
      {
        case M1_OFF_RANGE: 
        {
          M1_RANGE = M1_OFF_RANGE;
          return_state = OFF_RANGE;
          
          if (M1_li != M1_li_cur)
          {
            printf(" light change and pole count is %d \n", pole_count);
            M1_li = M1_li_cur;
            if (pole_count > 200 || pole_count < -200)
            {
                if (M1_FWD_flag == 1)
                  change_status++;
                if (M1_REV_flag == 1)
                  change_status--;
                //printf(" status change and pole count is %d \n", pole_count);
                //printf(" change_status is %d \n", change_status);
                pole_count = 0;
            }
            if ((change_status >= 3 && M1_FWD_flag == 1) || (change_status <= 1 && M1_REV_flag == 1))
            {
                M1_RANGE = M1_ON_RANGE;
                M1_INSTR_BACK = ((M1_FWD_flag << 1) | M1_REV_flag);
                //range_lock = 1;
                return_state = ON_RANGE;
            } else
            {
                M1_RANGE = M1_OFF_RANGE;
                return_state = OFF_RANGE;
            }
          }
          
          //if (((change_status < 4) && (change_status > 0)) || half_round == 1)
          //{
          //}
            
          break;
        }
        /*case M1_LEAVE_RANGE:
        {
          if (change_status > 0 || change_status < 4)
            M1_RANGE = M1_OFF_RANGE;
          else 
            M1_RANGE = M1_LEAVE_RANGE;
          //LED 3 D41 on
          GPIOF->ODR |= 0x00000400;
          
          return_state = OFF_RANGE;
          break;
        }*/
        case M1_ON_RANGE:
        {
          if (M1_INSTR_BACK != ((M1_FWD_flag << 1) | M1_REV_flag))
          {
            //if (range_lock == 1)
            //{
            //  half_round = 1;
            //  range_lock = 0;
            //}
            M1_RANGE = M1_OFF_RANGE;
            return_state = OFF_RANGE;
          } else
          {
            M1_RANGE = M1_ON_RANGE;
            return_state = ON_RANGE;
          }
          //LED 3 D41 off
          //GPIOF->ODR &= 0xFFFFFBFF;
          
          break;
        }
        default:
          break;
      }
  }
  return return_state;
}
      
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  