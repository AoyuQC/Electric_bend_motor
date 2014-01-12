/** 
   * @brief  MOTOR Control Structure definition  
   */
#include "STM32f10x.h"

//defnition of motor control types
#define CONTROL               ((uint8_t)0x01)  //instructions for motors
#define SPEED                 ((uint8_t)0x02)  //speed value of motors

//definition of motor control signls
#define ON                    ((uint8_t)0x01)
#define OFF                   ((uint8_t)0x00)
#define FWD                   ((uint8_t)0x01)
#define REV                   ((uint8_t)0x00)


typedef enum{M1 = 1, M2 = 2, M3 = 3} MotorType;
typedef struct
{
  uint8_t POWER_Supply; //enable signal of BTS5235 for power supply
  uint8_t BRAKE;        //brake signal of L6235
  uint8_t ENABLE;       //enable signal of L6235
  uint8_t FWDREV;       //FWDREV signal of L6235
  uint16_t Count_value; //decide the frequency of PWM: 50Mhz/Count_value
  uint16_t Duty_value;  //decide the duty cycle, must smaller than Count_value
} MOTOR_ControlTypeDef;

void remote_control(void);
void INSTR_Parse(void);
void MOTOR_Control(MotorType MOTORx, MOTOR_ControlTypeDef* MOTOR_ControlStruct, uint8_t Control);
void MOTOR_PID_Adjust(void);