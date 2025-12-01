#include<stdio.h>
#include"simulation.h"
#pragma comment(lib,"simulation.lib")
/**/

int userMain(void)
{
	int switchmodel = 0;	//0-右转	1-左转	2-回转	3-直行

	if (switchmodel == 0)
	{
		double angel = 0;
		double speedLMax = 0;
		double speedRMax = 0;
		double accelerate = 0.5;

		readMessage();
		Get_RandomDcMotor();

		SetPID_Right(11, 0.3, 0.2);
		SetPID_Left(11, 0.3, 0.2);

		while (angel <= 20)
		{
			SetSpeed_Right(Get_speedRight() - accelerate);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(Get_speedLeft() + accelerate);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			angel = Get_Angel();
			speedLMax = Get_speedLeft();
			speedRMax = Get_speedRight();
		}

		while (angel <= 70)
		{
			SetSpeed_Right(speedRMax);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(speedLMax);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			angel = Get_Angel();
		}

		while (angel <= 90)
		{
			if (abs(Get_speedRight()) < 0.001 || abs(Get_speedLeft()) < 0.001)
			{
				break;
			}
			SetSpeed_Right(Get_speedRight() + accelerate);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(Get_speedLeft() - accelerate);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			angel = Get_Angel();
			if (Get_speedRight() > 0 || Get_speedLeft() < 0)
			{
				SetSpeed_Right(Get_speedRight() - accelerate);
				MotorRight_Update();
				DcMotor_Right();

				SetSpeed_Left(Get_speedLeft() + accelerate);
				MotorLeft_Update();
				DcMotor_Left();
				Save_Speed();
				angel = Get_Angel();
			}
		}
		Trans_Speed(speedLMax, speedRMax);	//左轮&右轮期望顶棚速度
	}

	if (switchmodel == 1)
	{
		double angel = 0;
		double speedLMax = 0;
		double speedRMax = 0;
		double accelerate = 0.5;

		readMessage();
		Get_RandomDcMotor();

		SetPID_Right(11, 0.3, 0.2);
		SetPID_Left(11, 0.3, 0.2);

		while (angel >= -20)
		{
			SetSpeed_Right(Get_speedRight() + accelerate);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(Get_speedLeft() - accelerate);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			angel = Get_Angel();
			speedLMax = Get_speedLeft();
			speedRMax = Get_speedRight();
		}

		while (angel >= -70)
		{
			SetSpeed_Right(speedRMax);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(speedLMax);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			angel = Get_Angel();
		}

		while (angel >= -90)
		{
			if (abs(Get_speedRight()) < 0.001 || abs(Get_speedLeft()) < 0.001)
			{
				break;
			}
			SetSpeed_Right(Get_speedRight() - accelerate);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(Get_speedLeft() + accelerate);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			angel = Get_Angel();
			if (Get_speedRight() < 0 || Get_speedLeft() > 0)
			{
				SetSpeed_Right(Get_speedRight() + accelerate);
				MotorRight_Update();
				DcMotor_Right();

				SetSpeed_Left(Get_speedLeft() - accelerate);
				MotorLeft_Update();
				DcMotor_Left();
				Save_Speed();
				angel = Get_Angel();
			}
		}
		Trans_Speed(speedLMax, speedRMax);	//左轮&右轮期望顶棚速度
	}

	if (switchmodel == 2)
	{
		double angel = 0;
		double speedLMax = 0;
		double speedRMax = 0;
		double accelerate = 0.5;

		readMessage();
		Get_RandomDcMotor();

		SetPID_Right(11, 0.3, 0.2);
		SetPID_Left(11, 0.3, 0.2);

		while (angel <= 30)
		{
			SetSpeed_Right(Get_speedRight() - accelerate);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(Get_speedLeft() + accelerate);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			angel = Get_Angel();
			speedLMax = Get_speedLeft();
			speedRMax = Get_speedRight();
		}

		while (angel <= 150)
		{
			SetSpeed_Right(speedRMax);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(speedLMax);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			angel = Get_Angel();
		}

		while (angel <= 180)
		{
			if (abs(Get_speedRight()) < 0.001 || abs(Get_speedLeft()) < 0.001)
			{
				break;
			}
			SetSpeed_Right(Get_speedRight() + accelerate);
			MotorRight_Update();
			DcMotor_Right();

			SetSpeed_Left(Get_speedLeft() - accelerate);
			MotorLeft_Update();
			DcMotor_Left();
			Save_Speed();
			angel = Get_Angel();
			if (Get_speedRight() > 0 || Get_speedLeft() < 0)
			{
				SetSpeed_Right(Get_speedRight() - accelerate);
				MotorRight_Update();
				DcMotor_Right();

				SetSpeed_Left(Get_speedLeft() + accelerate);
				MotorLeft_Update();
				DcMotor_Left();
				Save_Speed();
				angel = Get_Angel();
			}
		}
		Trans_Speed(speedLMax, speedRMax);	//左轮&右轮期望顶棚速度
	}

	if (switchmodel == 3)
	{
		double dist = 0;
		double speedLMax = 0;
		double speedRMax = 0;
		double accelerate = 0.7;

		readMessage();
		Get_RandomDcMotor();

		SetPID_Right(11, 0.4, 0.3);
		SetPID_Left(9, 0.4, 0.7);

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
