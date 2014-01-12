#include "sch.h"
#define SEI()    __set_PRIMASK(0) //__enable_irq

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/***********************************************************************************/
//任务队列
sTask SCH_Tasks_G[SCH_MAX_TASKS];


/***********************************************************************************/

//使任务（函数）每隔一定间隔或在用户定义的延迟之后执行
tByte SCH_Add_Task(void (*pFunction)(),
						const tWord Delay,
						const tWord PERIOD)
{		  
	tByte Index = 0;
	//首先在队列中找到一个空隙（如果有的话）
	while((SCH_Tasks_G[Index].pTask != 0)&&(Index < SCH_MAX_TASKS))
	{
		Index++;
	}
	//是否已经到达队列的结尾?
	//如果任务队列满，产生错误提示
	assert_param(!(Index == SCH_MAX_TASKS));


	//如果能运行到这里，则说明任务队列中有空间
	SCH_Tasks_G[Index].pTask = pFunction;
	SCH_Tasks_G[Index].Delay = Delay;
	SCH_Tasks_G[Index].Period = PERIOD;
	SCH_Tasks_G[Index].RunMe = 0;
	return Index;	//返回任务的位置（以便以后删除）
}

/***********************************************************************************/

//现在不需要让单片机进入睡眠状态
void SCH_Go_To_Sleep(void)
{

}

void SCH_Delete_Task(const tByte TASK_INDEX)
{
//试图删除没有的任务
	assert_param(!(SCH_Tasks_G[TASK_INDEX].pTask == 0));
	
	SCH_Tasks_G[TASK_INDEX].pTask  = 0x0000;
	SCH_Tasks_G[TASK_INDEX].Delay  = 0;
	SCH_Tasks_G[TASK_INDEX].Period = 0;
	SCH_Tasks_G[TASK_INDEX].RunMe  = 0;
}

void SCH_Init_Task(void)
{
	tByte i;
	for(i = 0;i < SCH_MAX_TASKS;i++)
	{
		SCH_Tasks_G[i].pTask  = 0x0000;
		SCH_Tasks_G[i].Delay  = 0;
		SCH_Tasks_G[i].Period = 0;
		SCH_Tasks_G[i].RunMe  = 0;
	}
}



//tByte SCH_Delete_Task(const tByte TASK_INDEX)
//{
//	tByte Return_code;
//	if(SCH_Tasks_G[TASK_INDEX].pTask ==0)
//	{
//		//这里没有任务。。。
//		//
//		//设置全局错误变量
//				//Error_code_G = ERROR;
//		//同时返回错误代码
//		Return_code = ERROR;
//	}
//	else
//	{
//		Return_code = SUCCESS;
//	}
//	SCH_Tasks_G[TASK_INDEX].pTask  = 0x0000;
//	SCH_Tasks_G[TASK_INDEX].Delay  = 0;
//	SCH_Tasks_G[TASK_INDEX].Period = 0;
//	SCH_Tasks_G[TASK_INDEX].RunMe  = 0;
//	return Return_code;	//返回状态
//}

/***********************************************************************************/

//这是调度程序。当任务（函数）需要运行时，SCH_Dispatch_Tasks()将运行它。
//这个函数必须被主循环（重复）调用
void SCH_Dispatch_Tasks(void)
{
	tByte Index;
	//调度（运行）下一个任务（如果有任务就绪）
	for(Index = 0;Index < SCH_MAX_TASKS;Index++)
	{

		if(SCH_Tasks_G[Index].RunMe > 0)
		{
			SCH_Tasks_G[Index].pTask();	//执行任务
			SCH_Tasks_G[Index].RunMe -= 1;	//复位/减小RunMe标志
			//周期性的任务将自动地再次执行
			//如果这是个“单次”任务，将它从队列中删除
			if(SCH_Tasks_G[Index].Period == 0)
			{
				SCH_Delete_Task(Index);
			}
		}
	}
	//报告系统状况
//	SCH_Report_Status();
	//这里调度器进入空闲模式
//	SCH_Go_To_Sleep();
}

/***********************************************************************************/

void SCH_Start(void)
{
  SEI();
}

/***********************************************************************************/

