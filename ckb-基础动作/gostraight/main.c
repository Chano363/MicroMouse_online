#include<stdio.h>
#include"user.h"
#include"simulation.h"

int main()
{
	communicationInit();

	printf("仿真算法加载完成！\n");

	if (userMain() == 1)	// 用户主动结束仿真
	{
		//printf("仿真结束：用户主动结束！\n");
	}
	else 
	{
		//printf("仿真结束：return 0 ！\n");
	}

	printf("仿真程序结束！\n");
	return 0;

}
