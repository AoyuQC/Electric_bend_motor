#include "pid.h" 
#include "stm32f10x_it.h"
//header file of motor control; solenoid valve control; pump control
#include "motor_solenoid_pump.h"
//header file of Reset_endoscopy_position;
#include "position_detect.h"

extern MOTOR_ControlTypeDef M_C; //motor speed adjust

int j;
int16_t M1_PWM=0;
int16_t M2_PWM=0;
int16_t Fake_PWM=0;
int16_t Calc_send=0;
float P_Factor = 600;    // 比例系数 800
float I_Time = 50;
float D_Time = 1;
float Sample_Time = 0.001;
float I_Factor = 5;    // 积分系数 0.01 1
float D_Factor = 0;   // 微分系数 4.01 40

float thisError = 0;
float setError = 0;
float LastErrorValue = 0;	
float delta_this_error = 0;
float delta_last_error = 0;
float deltaCtrValue = 0;    

float thisError2 = 0;
float setError2 = 0;
float LastErrorValue2 = 0;
float delta_this_error2 = 0;
float delta_last_error2 = 0;
float deltaCtrValue2 = 0;

float error_history=0;
float error_history2=0;

float M1_Sspeed = 0;
float M1_Sspeed_back = 0;
float M1_Mspeed = 0;
float M2_Sspeed = 0;
float M2_Sspeed_back = 0;
float M2_Mspeed = 0;

uint16_t M1_Zerospeed_time = 0;
uint8_t M1_real_delay_time = 0;
uint8_t dir_delay_error = 0;
uint16_t step_dur_time = 0;
uint8_t M1_Brake_flag = 0;
uint8_t M1_FWD_flag = 0;
uint8_t M1_REV_flag = 0;
uint8_t M1_dir_change_flag = 0;
uint8_t m1_brake = 0;
uint8_t M1_Speed_update = 0;
uint8_t M1_Step = 0;
uint8_t NEG_Logic_flag = 0;
uint8_t dir = 0;

uint16_t M2_Zerospeed_time = 0;
uint8_t M2_real_delay_time = 0;
uint8_t dir_delay_error2 = 0;
uint16_t step_dur_time2 = 0;
uint8_t M2_Brake_flag = 0;
uint8_t M2_FWD_flag = 0;
uint8_t M2_REV_flag = 0;
uint8_t M2_dir_change_flag = 0;
uint8_t m2_brake = 0;
uint8_t M2_Speed_update = 0;
uint8_t M2_Step = 0;
uint8_t NEG_Logic_flag2 = 0;
uint8_t dir2 = 0;
uint8_t neg_logic = 0;

extern uint8_t dir_stat;
extern uint8_t dir_stat2;

extern uint16_t M1_count;
extern uint16_t M2_count;
extern int16_t pole_count;

#define Brake_ON      0x1
#define Brake_OFF     0x0
#define Brake_OUT     0x2

#define FWD_range     0x2D0  
#define REV_range     -0x2D0

uint8_t M1_1ms_Brake = 0;
uint8_t NEG_loose = 0;
extern uint8_t dir_stat;
uint8_t dir_last_time=1;
uint8_t real_dir_flag=0;
int8_t temp_thisError = 0;

uint8_t M2_1ms_Brake = 0;
uint8_t NEG_loose2 = 0;
extern uint8_t dir_stat2;
uint8_t dir_last_time2=1;
uint8_t real_dir_flag2=0;
int8_t temp_thisError2 = 0;

int m1_range;

/*incremental PID computation function */
static int M1_PID_Calc() 
{
  //for debug purpose param calculated every time
  //I_Factor = P_Factor/I_Time*sample_Time;
  //D_Factor = P_Factor*D_Time/sample_Time;
  //M1_Mspeed: measure speed of motor1
  //M1_Sspeed: set value of motor1 
  //thisError =(M1_Sspeed - M1_Mspeed);
 
  if(thisError>30)thisError=30;
  if(thisError<-30)thisError=-30;
 
  //proportion part
  deltaCtrValue = (P_Factor)*thisError;
  
  //integral part
  if(M1_Brake_flag == 0)
    error_history = error_history + thisError;
  else
    error_history = 0;
    
  if(error_history >= 1000)
    error_history = 1000;
  if(error_history <= -1000)
    error_history = -1000;
  
 //integral separation
  uint8_t I_Adjust_index = 1;
  if(setError>15 || setError<-15)
    if(thisError>15 || thisError<-15)
      I_Adjust_index = 0;
    else
      I_Adjust_index = 1;
  else
    I_Adjust_index = 1;
  deltaCtrValue = deltaCtrValue + (I_Adjust_index)*(I_Factor)*(error_history);
 
  //differentiation part
  delta_this_error = thisError - LastErrorValue; 
  //incomplete derivatation
  //d_delta_this_error = D_Factor*(1-alpha)*d_delta_this_error + alpha*d_delta_last_error;
  //deltaCtrValue = deltaCtrValue + d_delta_this_error;
  deltaCtrValue = deltaCtrValue + (D_Factor)*(delta_this_error);
  
  //update parameters
  LastErrorValue = thisError;       
  return (deltaCtrValue);           //the result is too big ,exceed output range,so reduce
} 

static int M2_PID_Calc() 
{
  if(thisError2>30)thisError2=30;
  if(thisError2<-30)thisError2=-30;
 
  //proportion part
  deltaCtrValue2 = (P_Factor)*thisError2;
  
  //integral part
  if(M2_Brake_flag == 0)
    error_history2 = error_history2 + thisError2;
  else
    error_history2 = 0;
    
  if(error_history2 >= 1000)
    error_history2 = 1000;
  if(error_history2 <= -1000)
    error_history2 = -1000;
  
 //integral separation
  uint8_t I_Adjust_index = 1;
  if(setError2>15 || setError2<-15)
    if(thisError2>15 || thisError2<-15)
      I_Adjust_index = 0;
    else
      I_Adjust_index = 1;
  else
    I_Adjust_index = 1;
  deltaCtrValue2 = deltaCtrValue2 + (I_Adjust_index)*(I_Factor)*(error_history2);
 
  //differentiation part
  delta_this_error2 = thisError2 - LastErrorValue2; 
  //incomplete derivatation
  //d_delta_this_error = D_Factor*(1-alpha)*d_delta_this_error + alpha*d_delta_last_error;
  //deltaCtrValue = deltaCtrValue + d_delta_this_error;
  deltaCtrValue2 = deltaCtrValue2 + (D_Factor)*(delta_this_error2);
  
  //update parameters
  LastErrorValue2 = thisError2;       
  return (deltaCtrValue2);           //the result is too big ,exceed output range,so reduce
} 

static void M1_Step_Action()
{
    uint16_t step_start_time = M1_Zerospeed_time - 500;
    if(m1_brake == 0)
    {
      step_dur_time++;
      M1_PWM = 1000;
      if(step_dur_time > 800)
      {
        m1_brake = 1;
        step_dur_time = 0;
      }
      M_C.Duty_value = 25000 - M1_PWM;  
      MOTOR_Control(M1,&M_C,SPEED);
    }
    else
    {
      M_C.BRAKE = ON;
      MOTOR_Control(M1,&M_C,CONTROL);
    }
    
    if(step_start_time%1000 == 0)
    {
      m1_brake = 0;
      M_C.BRAKE = OFF;
      MOTOR_Control(M1,&M_C,CONTROL);
    }
}

static void M2_Step_Action()
{
    uint16_t step_start_time = M2_Zerospeed_time - 500;
    if(m2_brake == 0)
    {
      step_dur_time2++;
      M2_PWM = 1000;
      if(step_dur_time2 > 800)
      {
        m2_brake = 1;
        step_dur_time2 = 0;
      }
      M_C.Duty_value = 25000 - M2_PWM;  
      MOTOR_Control(M2,&M_C,SPEED);
    }
    else
    {
      M_C.BRAKE = ON;
      MOTOR_Control(M2,&M_C,CONTROL);
    }
    
    if(step_start_time%1000 == 0)
    {
      m2_brake = 0;
      M_C.BRAKE = OFF;
      MOTOR_Control(M2,&M_C,CONTROL);
    }
}

//motor speed adjust function
void MOTOR_PID_Adjust()
{
  //check motor1 move range
  m1_range = Range_check(M1);
  if (m1_range == OFF_RANGE)
  {
    //calculate time of code
    //GPIOB->ODR |= 0x00000100;
    //uint16_t speed;
    //speed calculation 
    //formula: 800k(frequency of TIMx）/Mx_count*6(4)(hall sensor change 36(24) times a round)：rps
    //speed calculation for 12v BLDC
    //M1_Mspeed = 400000/(M1_count*3);
    //speed calculation for 24v electric_bend 24v BLDC
    if (M1_Speed_update == 1)
    {
      dir = dir_stat>>0x7;
      if(dir != dir_last_time)
        real_dir_flag = 1;
    
      M1_Mspeed = 100000/(M1_count*3);
      thisError = M1_Sspeed - M1_Mspeed;
      setError = M1_Sspeed - M1_Sspeed_back;
      
      dir_last_time = dir;
      M1_Sspeed_back = M1_Sspeed;
      if((M1_dir_change_flag == 0 && thisError > -3 && NEG_Logic_flag == 1)||((dir == 1 && M1_FWD_flag == 0 && M1_REV_flag == 1) || (dir == 0 && M1_FWD_flag == 1 && M1_REV_flag == 0)))
      {
        NEG_Logic_flag = 0;
        TIM_Cmd(TIM2, DISABLE);
      }
       
       //reset speed value && update flag && zero_speed_count (millisecond)
       M1_Speed_update = 0;
       //reset param && flag for motor step action
       M1_Zerospeed_time = 0;
       M1_Step = 0;
       m1_brake = 0;
       step_dur_time = 0;
    }else
    { 
      if(M1_FWD_flag == 1 || M1_REV_flag == 1)
      {
        M1_Zerospeed_time++;
        if(M1_Zerospeed_time == 500)
        {
          M1_Mspeed = 0;
          M1_Step = 1;
          M_C.BRAKE  = ON;
          MOTOR_Control(M1,&M_C,CONTROL);
        }else if(M1_Step == 1)
        {
          M1_Step_Action();
        }
      }
    }
    
    if(M1_FWD_flag == 1 || M1_REV_flag == 1)
    {
      if(M1_dir_change_flag == 1 && dir_delay_error == 0)
      {
        M1_real_delay_time++;
        if(M1_real_delay_time == 100)
        {
          dir_delay_error = 1;
          M1_real_delay_time = 0;  
        }
        M1_PWM = 2000;
        error_history = 0;
      }else
      {
        M1_real_delay_time = 0;
        int16_t temp_PID_Calc;
        temp_PID_Calc = M1_PID_Calc();
        M1_PWM = temp_PID_Calc;
      }
      
      if(M1_PWM<0) 
      {
        //if(M1_dir_change_flag == 1 && dir_delay_error == 0)
        //  M1_PWM = -M1_PWM;
        //else
        M1_PWM = 0;
      }
      
      if(M1_PWM>=25000) 
      {
        M1_PWM = 24999;
      } 
      
      //enable or disable neg logic depending on dir_change_flag
      if(M1_dir_change_flag == 0 && thisError <= -7 && NEG_Logic_flag == 0 && ((dir == 1 && M1_FWD_flag == 1 && M1_REV_flag == 0) || (dir == 0 && M1_FWD_flag == 0 && M1_REV_flag == 1)))
      {
        NEG_Logic_flag = 1;
        //GPIOF->ODR |= 0x00000100;
      }
      
      if(dir_delay_error == 1)
      {
        if((dir == 1 && M1_FWD_flag == 1 && M1_REV_flag == 0) || (dir == 0 && M1_FWD_flag == 0 && M1_REV_flag == 1))
        {
          real_dir_flag = 1;
          dir_delay_error = 0;
        }
      }    
      
      if(M1_dir_change_flag == 1 && real_dir_flag == 1 && ((dir == 1 && M1_FWD_flag == 1 && M1_REV_flag == 0) || (dir == 0 && M1_FWD_flag == 0 && M1_REV_flag == 1)))
      {
        //GPIOF->ODR |= 0x00000100;
        M1_dir_change_flag = 0;
        real_dir_flag = 0;
      }
      
      //generate negative logic
      if(NEG_Logic_flag == 1)
      {
        //debug abnormal neglogic
        neg_logic = 1;
        
        if(M1_FWD_flag == 1)
          M_C.FWDREV = REV;
        else if(M1_REV_flag == 1)
          M_C.FWDREV = FWD;
      }else
      {
        if(M1_FWD_flag == 1)
          M_C.FWDREV = FWD;
        else if(M1_REV_flag == 1)
          M_C.FWDREV = REV;
        TIM_Cmd(TIM2, ENABLE);
        //M_C.Duty_value = 25000 - M1_PWM;
      }
      M_C.BRAKE = OFF;
      M_C.ENABLE = ON;
      
      /*if (M_C.FWDREV == FWD)
        //LED 2 D40 on
        GPIOF->ODR |= 0x00000100;
      else if (M_C.FWDREV == REV)
        //LED 2 D40 off
        GPIOF->ODR &= 0xFFFFFEFF;*/
        
      M_C.Duty_value = 25000 - M1_PWM;
      //debug constant PWM
      //M_C.Duty_value = 25000 - 0;
      MOTOR_Control(M1,&M_C,CONTROL|SPEED);
      //LED 2 D40 off
      //GPIOF->ODR &= 0xFFFFFEFF;
    }
  }else if(m1_range == ON_RANGE)
  {
    M_C.BRAKE = ON;
    MOTOR_Control(M1,&M_C,CONTROL);
    //LED 2 D40 on
    //GPIOF->ODR |= 0x00000100;
    //printf("m1_range is %d and ON_RANGE is %d and OFF_RANGE is %d \n", m1_range, ON_RANGE, OFF_RANGE);
  }
  
  if(M2_Speed_update == 1)
  {
    dir2 = dir_stat2>>0x7;
    if(dir2 != dir_last_time2)
      real_dir_flag2 = 1;
  
    M2_Mspeed = 100000/(M2_count*3);
    thisError2 = M2_Sspeed - M2_Mspeed;
    setError2 = M2_Sspeed - M2_Sspeed_back;
    
    dir_last_time2 = dir2;
    M2_Sspeed_back = M2_Sspeed;
    if((M2_dir_change_flag == 0 && thisError2 > -3 && NEG_Logic_flag2 == 1)||(dir2 == 1 && M2_FWD_flag == 0 && M2_REV_flag == 1)||(dir2 == 0 && M2_FWD_flag == 1 && M2_REV_flag == 0))
    {
      NEG_Logic_flag2 = 0;
      TIM_Cmd(TIM4, DISABLE);
    }
     
     //reset speed value && update flag && zero_speed_count (millisecond)
     M2_Speed_update = 0;
     //reset param && flag for motor step action
     M2_Zerospeed_time = 0;
     M2_Step = 0;
     m2_brake = 0;
     step_dur_time2 = 0;
  }else
  { 
    if(M2_FWD_flag == 1 || M2_REV_flag == 1)
    {
      M2_Zerospeed_time++;
      if(M2_Zerospeed_time == 500)
      {
        M2_Mspeed = 0;
        M2_Step = 1;
        M_C.BRAKE  = ON;
        MOTOR_Control(M2,&M_C,CONTROL);
      }else if(M2_Step == 1)
      {
        M2_Step_Action();
      }
    }
  }
  
  if(M2_FWD_flag == 1 || M2_REV_flag == 1)
  {
    if(M2_dir_change_flag == 1 && dir_delay_error2 == 0)
    {
      M2_real_delay_time++;
      if(M2_real_delay_time == 100)
      {
        dir_delay_error2 = 1;
        M2_real_delay_time = 0;  
      }
      M2_PWM = 2000;
      error_history2 = 0;
    }else
    {
      M2_real_delay_time = 0;
      int16_t temp_PID_Calc;
      temp_PID_Calc = M2_PID_Calc();
      M2_PWM = temp_PID_Calc;
    }
    
    if(M2_PWM<0) 
    {
      //if(M1_dir_change_flag == 1 && dir_delay_error == 0)
      //  M1_PWM = -M1_PWM;
      //else
      M2_PWM = 0;
    }
    
    if(M2_PWM>=25000) 
    {
      M2_PWM = 24999;
    } 
    
    //enable or disable neg logic depending on dir_change_flag
    if(M2_dir_change_flag == 0 && thisError2 <= -7 && NEG_Logic_flag2 == 0 && ((dir2 == 1 && M2_FWD_flag == 1 && M2_REV_flag == 0) || (dir2 == 0 && M2_FWD_flag == 0 && M2_REV_flag == 1)))
    {
      NEG_Logic_flag2 = 1;
      //GPIOF->ODR |= 0x00000100;
    }
    
    if(dir_delay_error2 == 1)
    {
      if((dir2 == 1 && M2_FWD_flag == 1 && M2_REV_flag == 0) || (dir2 == 0 && M2_FWD_flag == 0 && M2_REV_flag == 1))
      {
        real_dir_flag2 = 1;
        dir_delay_error2 = 0;
      }
    }    
    
    if(M2_dir_change_flag == 1 && real_dir_flag2 == 1 && ((dir2 == 1 && M2_FWD_flag == 1 && M2_REV_flag == 0) || (dir2 == 0 && M2_FWD_flag == 0 && M2_REV_flag == 1)))
    {
      //GPIOF->ODR |= 0x00000100;
      M2_dir_change_flag = 0;
      real_dir_flag2 = 0;
    }
    
    //generate negative logic
    if(NEG_Logic_flag2 == 1)
    {
      if(M2_FWD_flag == 1)
      {
        M_C.FWDREV = REV;
       }
      else if(M2_REV_flag == 1)
        M_C.FWDREV = FWD;
    }else
    {
      if(M2_FWD_flag == 1)
        M_C.FWDREV = FWD;
      else if(M2_REV_flag == 1)
        M_C.FWDREV = REV;
      TIM_Cmd(TIM4, ENABLE);
        //M_C.Duty_value = 25000 - M1_PWM;
    }
    //debug 5 rps
    //M2_PWM = 2870;
    M_C.Duty_value = 25000 - M2_PWM;
    //debug constant PWM
    //M_C.Duty_value = 25000 - 0;
    
    MOTOR_Control(M2,&M_C,CONTROL|SPEED);
  }
} 





