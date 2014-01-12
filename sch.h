#ifndef STM32F103_SCH_H
#define STM32F103_SCH_H

#include "stm32f10x.h"
/* Private variables ---------------------------------------------------------*/
typedef uint32_t tByte;
typedef uint32_t tWord;
typedef uint32_t tLong;

typedef struct
{
	//ָ�������ָ�루������һ����void��void����������
	void (*pTask)(void);
	//�ӳ٣�ʱ�ֱ꣩������������һ�Σ�����
	//��ϸ˵���μ�SCH_Add_Task*()
	tWord Delay;
	//��������֮��ļ����ʱ�꣩
	//��ϸ˵���μ�SCH_Add_Task()
	tWord Period;
	//��������Ҫ����ʱ���ɵ���������1
	tByte RunMe;
} sTask;

//�ڳ��������ڼ����һʱ����������������Ŀ
//
//ÿ���½���Ŀ���������
#define SCH_MAX_TASKS 10

/* Private function prototypes -----------------------------------------------*/

void SysTick_Handler(void);
tByte SCH_Add_Task(void (*pFunction)(),
						const tWord Delay,
						const tWord PERIOD);
void SCH_Dispatch_Tasks(void);
void SCH_Init_Task(void);
void SCH_Delete_Task(const tByte TASK_INDEX);
void SCH_Go_To_Sleep(void);
void SCH_Start(void);

#endif
