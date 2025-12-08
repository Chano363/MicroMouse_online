#pragma once


//========================================================
// 【禁止修改】仿真系统交互相关函数
//========================================================
int userMain(void);
//========================================================



//========================================================
// 建议将 user.c 中的自定义函数声明在下面，以防止编译错误
//========================================================
void updateMap(void);
void objectGoTo(char cXdst, char cYdst);
void rightMethod(void);
int mazeIsSearched(int ucDirTemp);
int crosswayCheck(char cX, char cY);
void mapStepEdit(char  cX, char  cY);
//========================================================
