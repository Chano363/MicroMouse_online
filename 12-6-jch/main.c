#include<stdio.h>
#include"user.h"
#pragma comment(lib,"simulation.lib")
void communicationInit(void);
int main()
{
    communicationInit(); //通信初始化
    printf("仿真算法加载完成！\n");
    if (userMain() == 1)
    {
        printf("仿真正常结束！\n");
    }
    else
    {
        printf("仿真异常结束！\n");
    }
    return 0;
}
