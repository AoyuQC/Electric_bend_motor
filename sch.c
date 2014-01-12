#include "sch.h"
#define SEI()    __set_PRIMASK(0) //__enable_irq

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/***********************************************************************************/
//�������
sTask SCH_Tasks_G[SCH_MAX_TASKS];


/***********************************************************************************/

//ʹ���񣨺�����ÿ��һ����������û�������ӳ�֮��ִ��
tByte SCH_Add_Task(void (*pFunction)(),
						const tWord Delay,
						const tWord PERIOD)
{		  
	tByte Index = 0;
	//�����ڶ������ҵ�һ����϶������еĻ���
	while((SCH_Tasks_G[Index].pTask != 0)&&(Index < SCH_MAX_TASKS))
	{
		Index++;
	}
	//�Ƿ��Ѿ�������еĽ�β?
	//������������������������ʾ
	assert_param(!(Index == SCH_MAX_TASKS));


	//��������е������˵������������пռ�
	SCH_Tasks_G[Index].pTask = pFunction;
	SCH_Tasks_G[Index].Delay = Delay;
	SCH_Tasks_G[Index].Period = PERIOD;
	SCH_Tasks_G[Index].RunMe = 0;
	return Index;	//���������λ�ã��Ա��Ժ�ɾ����
}

/***********************************************************************************/

//���ڲ���Ҫ�õ�Ƭ������˯��״̬
void SCH_Go_To_Sleep(void)
{

}

void SCH_Delete_Task(const tByte TASK_INDEX)
{
//��ͼɾ��û�е�����
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
//		//����û�����񡣡���
//		//
//		//����ȫ�ִ������
//				//Error_code_G = ERROR;
//		//ͬʱ���ش������
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
//	return Return_code;	//����״̬
//}

/***********************************************************************************/

//���ǵ��ȳ��򡣵����񣨺�������Ҫ����ʱ��SCH_Dispatch_Tasks()����������
//����������뱻��ѭ�����ظ�������
void SCH_Dispatch_Tasks(void)
{
	tByte Index;
	//���ȣ����У���һ��������������������
	for(Index = 0;Index < SCH_MAX_TASKS;Index++)
	{

		if(SCH_Tasks_G[Index].RunMe > 0)
		{
			SCH_Tasks_G[Index].pTask();	//ִ������
			SCH_Tasks_G[Index].RunMe -= 1;	//��λ/��СRunMe��־
			//�����Ե������Զ����ٴ�ִ��
			//������Ǹ������Ρ����񣬽����Ӷ�����ɾ��
			if(SCH_Tasks_G[Index].Period == 0)
			{
				SCH_Delete_Task(Index);
			}
		}
	}
	//����ϵͳ״��
//	SCH_Report_Status();
	//����������������ģʽ
//	SCH_Go_To_Sleep();
}

/***********************************************************************************/

void SCH_Start(void)
{
  SEI();
}

/***********************************************************************************/

