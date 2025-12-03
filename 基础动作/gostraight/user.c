#include<stdio.h>
#include"simulation.h"
#pragma comment(lib,"simulation.lib")
/**/

int userMain(void)
{
	int switchmodel = 3;	//0-右转	1-左转	2-回转	3-直行
	if (switchmodel == 3)
	{
		double dist = 0;
		double speedLMax = 0;
		double speedRMax = 0;
		double accelerate = 15.5;

		readMessage();
		Get_RandomDcMotor();

		SetPID_Right(8, 0.43, 0.8);
		SetPID_Left(13, 0.65, 0.8);

		while (dist <= 4)
		{
			SetSpeed_Right(Get_speedRight() + accelerate);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(Get_speedLeft() + accelerate);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			speedLMax = Get_speedLeft();
			speedRMax = Get_speedRight();
			dist = GetDistCent();
		}

		while (dist <= 12)
		{
			SetSpeed_Right(speedRMax);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(speedLMax);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			dist = GetDistCent();
		}

		while (dist <= 16)
		{
			if (abs(Get_speedRight()) < 0.001 || abs(Get_speedLeft()) < 0.001)
			{
				break;
			}
			SetSpeed_Right(Get_speedRight() - accelerate);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(Get_speedLeft() - accelerate);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			dist = GetDistCent();
			if (Get_speedRight() < 0 || Get_speedLeft() < 0)
			{
				SetSpeed_Right(Get_speedRight() + accelerate);
				MotorRight_Update();
				DcMotor_Right();

				SetSpeed_Left(Get_speedLeft() + accelerate);
				MotorLeft_Update();
				DcMotor_Left();
				Save_Speed();
				dist = GetDistCent();
			}
		}
		Trans_Speed(speedLMax, speedRMax);	//左轮&右轮期望顶棚速度
	}
}
