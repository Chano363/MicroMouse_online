#pragma once
#include "windows.h"
#pragma comment(lib,"simulation.lib")


//========================================================
// 【禁止修改】仿真系统交互相关
//========================================================
int sendMessage(void);
int readMessage(void);
void communicationInit(void);
void SendMessages(char);
void SendDouble(double,double);
void MotorLeft_Update(void);
void DcMotor_Left(void);
void SetPID_Left(double, double, double);
void SetSpeed_Left(double);
void MotorRight_Update(void);
void DcMotor_Right(void);
void SetPID_Right(double, double, double);
void SetSpeed_Right(double);
void sendToBuffer(char);
void Save_Speed(void);
void Trans_Speed(double, double);
double Get_Angel(void);
double Get_speedLeft(void);
double Get_speedRight(void);
double GetDistCent(void);
void Get_RandomDcMotor(void);
//========================================================
