/**
  ******************************************************************************
  * @file    motor_solenoid_pump.c 
  * @author  zay
  * @version V1.0
  * @date    
  * @brief   1.INSTR_Parse; 2.MOTOR_Control
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "buffer.h"
#include "motor_solenoid_pump.h"
#include "stdio.h"    
    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//conditions for control motors
//INSTR: remote_data[1]:M(0x4d) remote_data[2]:1-3(0x31-33) remote_data[3]:F(0x46) R(0x52) B(0x42)
#define M1F     0x4d3146
#define M1R     0x4d3152
#define M1B     0x4d3142
#define M2F     0x4d3246
#define M2R     0x4d3252
#define M2B     0x4d3242
#define M3F     0x4d3346
#define M3R     0x4d3352
#define M3B     0x4d3342
#define Bank1_SRAM2_ADDR    ((uint32_t)0x64000000)   
//PID debug
#define P       0x50
#define I       0x49
#define D       0x44
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t start_get = 0;
uint8_t TxBuffer[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t SPEED_Bit4 = 0;
uint8_t SPEED_Bit3 = 0;
uint8_t SPEED_Bit2 = 0;
uint8_t SPEED_Bit1 = 0;
uint8_t SPEED_Bit0 = 0;
float PID_Value3 = 0;
float PID_Value2 = 0;
float PID_Value1 = 0;
float PID_Value0 = 0;

uint8_t get_data[8] = {0,0,0,0,0,0,0,0};
uint32_t INSTR;
extern float M1_Sspeed;
extern float M2_Sspeed;
extern uint8_t dir_count;
extern MOTOR_ControlTypeDef M_C;
extern volatile FIFO_TypeDef U1Rx;

extern float P_Factor;
extern float I_Factor;
extern float D_Factor;
extern float I_Time;
extern float D_Time;

extern uint8_t M1_Brake_flag;
extern uint8_t M1_FWD_flag;
extern uint8_t M1_REV_flag;
extern uint8_t M1_dir_change_flag;
extern int16_t M1_PWM;

extern uint8_t M2_Brake_flag;
extern uint8_t M2_FWD_flag;
extern uint8_t M2_REV_flag;
extern uint8_t M2_dir_change_flag;

extern uint8_t m1_on_range;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void MOTOR_Control(MotorType MOTORx, MOTOR_ControlTypeDef* MOTOR_ControlStruct, uint8_t Control);
void INSTR_Parse(void);

//parse instructions
void INSTR_Parse()
{
    uint8_t ch;
    uint8_t index;
    uint8_t i;
    uint8_t count;
    uint8_t flag = 0;
    
    count = U1Rx.count;
    index = 0;
    while(dir_count)
    {
      for(i = 1; i <= count; i++)
      {
        //printf("the i is %d \n",i);
        if(BufferGet(&U1Rx, &ch) == SUCCESS)
        {
          if(ch == '$')
          {
            start_get = 1;
          }
          else if(ch == '*')
          {
            start_get = 0;
            //printf("the index is %d \n",index);
            break;
          }
          else if(start_get == 1)
          {
            get_data[index] = ch;
            index++;
          }  
        }
        else
        {
          printf("BufferGet fail : run of U1Rx \n");
        }
      }      
      
      if(index == 0)
        //printf("no value in U1Rx \n");
        flag = 1;
      else if(index == 4)
      {
        INSTR = get_data[0];
        PID_Value2 = (int)(get_data[1]-'0');
        PID_Value1 = (int)(get_data[2]-'0');
        PID_Value0 = (int)(get_data[3]-'0');
        dir_count--;
      }
      else if(index == 5)
      {
        INSTR = get_data[0];
        PID_Value3 = (int)(get_data[1]-'0');
        PID_Value2 = (int)(get_data[2]-'0');
        PID_Value1 = (int)(get_data[3]-'0');
        PID_Value0 = (int)(get_data[4]-'0');
        dir_count--;
      }  
      else
      {
        //I = U1Rx.count;
        //printf("the I is %d \n",I);
        //printf("one INTR parse \n");
        //printf("index is %d \n",index);
        INSTR = get_data[0]<<16 | get_data[1]<<8 | get_data[2];
        SPEED_Bit4 = (int)(get_data[3]-'0');
        SPEED_Bit3 = (int)(get_data[4]-'0');
        SPEED_Bit2 = (int)(get_data[5]-'0');
        SPEED_Bit1 = (int)(get_data[6]-'0');
        SPEED_Bit0 = (int)(get_data[7]-'0');
        dir_count--;
      }
      
      switch(INSTR)
      {
      case M1F:     // motor 1 run left(FWD)
        //printf("M1 forward \n");
        INSTR = 100;
        //set speed of motor 1: rps
        M1_Sspeed = SPEED_Bit4*10000 + SPEED_Bit3*1000 + SPEED_Bit2*100 + SPEED_Bit1*10 +SPEED_Bit0;
        //flag indicate fwd rev brake
        if(M1_FWD_flag == 0 && M1_REV_flag == 1)
          M1_dir_change_flag = 1;
        M1_FWD_flag = 1;
        M1_REV_flag = 0;
        M1_Brake_flag = 0;
        TIM_Cmd(TIM2, ENABLE);
        //debug
        //M_C.FWDREV = FWD;
        //M_C.ENABLE = ON;
        //M_C.BRAKE  = OFF;
        //MOTOR_Control(M1,&M_C,CONTROL);
        //LED 1 D39 on
        //GPIOF->ODR |= 0x00000040;
        break;
      case M1R:     // motor 1 run right(REV)
        INSTR = 100;
        //set speed of motor 1: rps
        M1_Sspeed = SPEED_Bit4*10000 + SPEED_Bit3*1000 + SPEED_Bit2*100 + SPEED_Bit1*10 +SPEED_Bit0;
        //flag indicate fwd rev brake
        if(M1_FWD_flag == 1 && M1_REV_flag == 0)
          M1_dir_change_flag = 1;
        M1_FWD_flag = 0;
        M1_REV_flag = 1;
        M1_Brake_flag = 0;
        TIM_Cmd(TIM2, ENABLE);
        //debug
        //M_C.FWDREV = REV;
        //M_C.ENABLE = ON;
        //M_C.BRAKE  = OFF;
        //MOTOR_Control(M1,&M_C,CONTROL);
        //LED 1 D39 off
        //GPIOF->ODR &= 0xFFFFFFBF;
        break;
      case M2F:     // motor 2 run left
        INSTR = 100;
        M_C.FWDREV = FWD;
        M_C.ENABLE = ON;
        M_C.BRAKE  = OFF;
        //set speed of motor 2 rps
        M2_Sspeed = SPEED_Bit4*10000 + SPEED_Bit3*1000 + SPEED_Bit2*100 + SPEED_Bit1*10 +SPEED_Bit0;
        MOTOR_Control(M2,&M_C,CONTROL);
        //flag indicate fwd rev brake
        if(M2_FWD_flag == 0 && M2_REV_flag == 1)
          M2_dir_change_flag = 1;
        M2_FWD_flag = 1;
        M2_REV_flag = 0;
        M2_Brake_flag = 0;
        TIM_Cmd(TIM4, ENABLE); 
        break;
      case M2R:     // motor 2 run right
        INSTR = 100;
        M_C.FWDREV = REV;
        M_C.ENABLE = ON;
        M_C.BRAKE  = OFF;
        //set speed of motor 2: rps
        M2_Sspeed = SPEED_Bit4*10000 + SPEED_Bit3*1000 + SPEED_Bit2*100 + SPEED_Bit1*10 +SPEED_Bit0;
        MOTOR_Control(M2,&M_C,CONTROL);
        //flag indicate fwd rev brake
        if(M2_FWD_flag == 1 && M2_REV_flag == 0)
          M2_dir_change_flag = 1;
        M2_FWD_flag = 0;
        M2_REV_flag = 1;
        M2_Brake_flag = 0;
        TIM_Cmd(TIM4, ENABLE);
        break;
      case M1B:     // motor 1 brake
        INSTR = 100;
        M_C.BRAKE  = ON;
        MOTOR_Control(M1,&M_C,CONTROL);
        //flag indicate fwd rev brake
        M1_FWD_flag = 0;
        M1_REV_flag = 0;
        M1_Brake_flag = 1;
        break;
      case M2B:     // motor 2 brake
        INSTR = 100;
        M_C.BRAKE  = ON;
        MOTOR_Control(M2,&M_C,CONTROL);
        //flag indicate fwd rev brake
        M2_FWD_flag = 0;
        M2_REV_flag = 0;
        M2_Brake_flag = 1;
        break;
      //debug purpose
      case P:
        P_Factor = PID_Value2*100 + PID_Value1*10 + PID_Value0;
        //printf("%f %f %f \n",P_Factor,I_Factor,D_Factor);
        INSTR = 100;
        break;
      case I:
        I_Factor = PID_Value2*100 + PID_Value1*10 + PID_Value0;
        //printf("%f %f %f \n",P_Factor,I_Factor,D_Factor);
        INSTR = 100;
        break;
      case D:
        D_Factor = PID_Value3*1000 + PID_Value2*100 + PID_Value1*10 + PID_Value0;
        //printf("%f %f %f \n",P_Factor,I_Factor,D_Factor);
        INSTR = 100;
        break;
      default:    //Disable two motors
        break;
      }
    }
}


void MOTOR_Control(MotorType MOTORx, MOTOR_ControlTypeDef* MOTOR_ControlStruct, uint8_t Control)
{
  //control signals parse
  uint8_t instr_en = Control&0x01;
  uint8_t speed_en = (Control&0x02)>>1;
  
  //instructions parse of motors
  if(instr_en)
  {
    uint8_t  tempreg = 0x00;
    
    tempreg = MOTOR_ControlStruct->POWER_Supply<<3 | (MOTOR_ControlStruct->BRAKE^(uint8_t)0x01)<<2 | MOTOR_ControlStruct->ENABLE<<1 | MOTOR_ControlStruct->FWDREV;
    
    if(MOTORx == M1)
    {
      tempreg &= 0x0F;
      TxBuffer[0] |= tempreg;
      tempreg |= 0xF0;
      TxBuffer[0] &= tempreg;
      //write control signals to CPLD
      *(uint8_t *) (Bank1_SRAM2_ADDR + 0) = TxBuffer[0];
    }
    else if(MOTORx == M2)
    {
      tempreg = tempreg <<4;
      tempreg &= 0xF0;
      TxBuffer[0] |= tempreg;
      tempreg |= 0x0F;
      TxBuffer[0] &= tempreg;
      //write control signals to CPLD
      *(uint8_t *) (Bank1_SRAM2_ADDR + 0) = TxBuffer[0];
    }
    else if(MOTORx == M3)
    {
      tempreg &= 0x0F;
      TxBuffer[9] |= tempreg;
      tempreg |= 0xF0;
      TxBuffer[9] &= tempreg;
      //write control signals to CPLD
      *(uint8_t *) (Bank1_SRAM2_ADDR + 9) = TxBuffer[9];
    }
  }
  
  //speed adjust information of motors
  if(speed_en)
  { 
    if(MOTORx == M1)
    {
      //TxBuffer[1] = (uint8_t)(MOTOR_ControlStruct->Count_value);
      //TxBuffer[2] = (uint8_t)(MOTOR_ControlStruct->Count_value>>8);      
      TxBuffer[3] = (uint8_t)MOTOR_ControlStruct->Duty_value;
      TxBuffer[4] = (uint8_t)(MOTOR_ControlStruct->Duty_value>>8);
      //write pwm data to CPLD
      //*(uint8_t *) (Bank1_SRAM2_ADDR + 1) = TxBuffer[1];
      //*(uint8_t *) (Bank1_SRAM2_ADDR + 2) = TxBuffer[2];
      *(uint8_t *) (Bank1_SRAM2_ADDR + 3) = TxBuffer[3];
      *(uint8_t *) (Bank1_SRAM2_ADDR + 4) = TxBuffer[4];
    }
    else if(MOTORx == M2)
    {
      //TxBuffer[5] = (uint8_t)(MOTOR_ControlStruct->Count_value);
      //TxBuffer[6] = (uint8_t)(MOTOR_ControlStruct->Count_value>>8);
      TxBuffer[7] = (uint8_t)(MOTOR_ControlStruct->Duty_value);
      TxBuffer[8] = (uint8_t)(MOTOR_ControlStruct->Duty_value>>8);
      //write pwm data to CPLD
      //*(uint8_t *) (Bank1_SRAM2_ADDR + 5) = TxBuffer[5];
      //*(uint8_t *) (Bank1_SRAM2_ADDR + 6) = TxBuffer[6];
      *(uint8_t *) (Bank1_SRAM2_ADDR + 7) = TxBuffer[7];
      *(uint8_t *) (Bank1_SRAM2_ADDR + 8) = TxBuffer[8];
    }
    else if(MOTORx == M3)
    {
      //TxBuffer[10] = (uint8_t)(MOTOR_ControlStruct->Count_value);
      //TxBuffer[11] = (uint8_t)(MOTOR_ControlStruct->Count_value>>8);
      TxBuffer[12] = (uint8_t)(MOTOR_ControlStruct->Duty_value);
      TxBuffer[13] = (uint8_t)(MOTOR_ControlStruct->Duty_value>>8);
      //write pwm data to CPLD
      //*(uint8_t *) (Bank1_SRAM2_ADDR + 10) = TxBuffer[10];
      //*(uint8_t *) (Bank1_SRAM2_ADDR + 11) = TxBuffer[11];
      *(uint8_t *) (Bank1_SRAM2_ADDR + 12) = TxBuffer[12];
      *(uint8_t *) (Bank1_SRAM2_ADDR + 13) = TxBuffer[13];
    }
  }
}