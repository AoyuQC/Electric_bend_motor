#ifndef STM32F103_SCH_H
#define STM32F103_SCH_H

#include "stm32f10x.h"
/* Private variables ---------------------------------------------------------*/
typedef uint32_t tByte;
typedef uint32_t tWord;
typedef uint32_t tLong;

typedef struct
{
	//指向任务的指针（必须是一个“void（void）“函数）
	void (*pTask)(void);
	//延迟（时标）直到函数将（下一次）运行
	//详细说明参加SCH_Add_Task*()
	tWord Delay;
	//连续运行之间的间隔（时标）
	//详细说明参见SCH_Add_Task()
	tWord Period;
	//当任务需要运行时（由调度器）加1
	tByte RunMe;
} sTask;

//在程序运行期间的任一时刻允许的任务最大数目
//
//每个新建项目都必须调整
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
