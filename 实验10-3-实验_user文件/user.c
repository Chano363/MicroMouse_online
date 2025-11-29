#include<stdio.h>
#include"user.h"

int userMain(void)
{

	int temp;
	int canMoveNum;
	int GucXStart = 0;
	int GucYStart = 0;

	int GoalGet = 0;
	int GoalX, GoalY;

	mouseInit();

	while (1)
	{
		// 若进入未搜索地区，则更新墙壁情况
		if (GucMapGet[GmcMouse.cX][GmcMouse.cY] == 0)
		{
			updateMap();
		}

		// 根据运行状态，分类处理
		switch (GucMouseTask)
		{

		case WAIT:
			
			GucMouseTask = START;
			break;

		case START:

			if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08)
			{
				GucXStart = MAZETYPE - 1;
				GmcMouse.cX = MAZETYPE - 1;

				if (GmcMouse.cY < MAZETYPE - 1)
				{
					GucMapBlock[MAZETYPE - 1][GmcMouse.cY + 1] = GucMapBlock[0][GmcMouse.cY + 1];
					GucMapBlock[0][GmcMouse.cY + 1] = 0xf0;
				}
				temp = GmcMouse.cY;
				do
				{
					// 转换墙壁信息
					GucMapBlock[MAZETYPE - 1][temp] = GucMapBlock[0][temp];
					GucMapBlock[MAZETYPE - 2][temp] = 0xD0;
					GucMapBlock[0][temp] = 0xf0;
					GucMapBlock[1][temp] = 0xf0;

					// 转换已搜索区域信息
					GucMapGet[MAZETYPE - 1][temp] = 1;
					GucMapGet[0][temp] = 0;

				} while (temp--);
				GucMapBlock[MAZETYPE - 2][GmcMouse.cY] = 0xf0;

				GucMouseTask = MAZESEARCH;
			}
			else if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02)
			{
				GucMouseTask = MAZESEARCH;
			}
			else MoveOneBlock();
			break;

		case MAZESEARCH:

			if ((((GmcMouse.cX == 7) || (GmcMouse.cX == 8)) && ((GmcMouse.cY == 7) || (GmcMouse.cY == 8))) && !GoalGet)
			{
				GoalGet = 1;
				GoalX = GmcMouse.cX;
				GoalY = GmcMouse.cY;
			}

			canMoveNum = crosswayCheck(GmcMouse.cX, GmcMouse.cY);

			if (canMoveNum > 1)
			{
				GmcCrossway[crosswayNum].cX = GmcMouse.cX;
				GmcCrossway[crosswayNum].cY = GmcMouse.cY;
				crosswayNum++;

				rightMethod();
				break;
			}
			else if (canMoveNum == 1)
			{
				rightMethod();
				break;
			}
			else
			{
				// 没有可前进方向，准备返回

				if (crosswayNum > 0)
				{
					// 存在岔路信息，返回上一个岔路坐标
					crosswayNum--;
					objectGoTo(GmcCrossway[crosswayNum].cX, GmcCrossway[crosswayNum].cY);
					break;
				}
				else
				{
					// 不存在岔路信息，搜索阶段已完成，返回起点
					objectGoTo(GucXStart, GucYStart);
					TurnBack();
					GucMouseTask = SPURT;
					break;
				}
			}

		case SPURT:

			objectGoTo(GoalX, GoalY);
			GucMouseTask = END;
			break;

		case END:

			mouseEnd();
			return 1; // 用户算法执行完毕，主动结束仿真
			break;
		}

	}

	mouseEnd();
	return 0;	// 用户算法执行完毕，主动结束仿真

}

// 更新迷宫墙壁信息
void updateMap(void)
{
	uchar ucMap = 0;
	uchar temp, temp1;

	GucMapGet[GmcMouse.cX][GmcMouse.cY] = 1;

	ucMap |= MOUSEWAY_B;

	if (leftHasWall)
	{
		ucMap &= ~MOUSEWAY_L;
	}
	else
	{
		ucMap |= MOUSEWAY_L;
	}
	if (frontHasWall)
	{
		ucMap &= ~MOUSEWAY_F;

	}
	else
	{
		ucMap |= MOUSEWAY_F;
	}
	if (rightHasWall)
	{
		ucMap &= ~MOUSEWAY_R;

	}
	else
	{
		ucMap |= MOUSEWAY_R;
	}

	//ucMap = (ucMap & 0x0F);
	//temp1 = ucMap;
	//temp = (temp1 << 4);
	//ucMap = (ucMap | temp);
	GucMapBlock[GmcMouse.cX][GmcMouse.cY] = ucMap;


	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x0f) == 0x00)
	{
		GucMapBlock[GmcMouse.cX][GmcMouse.cY] = ucMap;

		if (GmcMouse.cX > 0)
		{
			if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x08) == 0x00)
			{
				GucMapBlock[GmcMouse.cX - 1][GmcMouse.cY] = ((GucMapBlock[GmcMouse.cX - 1][GmcMouse.cY]) & 0xdf);
			}
		}

		if (GmcMouse.cX < MAZETYPE - 1)
		{
			if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x02) == 0x00)
			{
				GucMapBlock[GmcMouse.cX + 1][GmcMouse.cY] = ((GucMapBlock[GmcMouse.cX + 1][GmcMouse.cY]) & 0x7f);
			}
		}

		if (GmcMouse.cY > 0)
		{
			if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x04) == 0x00)
			{
				GucMapBlock[GmcMouse.cX][GmcMouse.cY - 1] = ((GucMapBlock[GmcMouse.cX][GmcMouse.cY - 1]) & 0xef);
			}
		}

		if (GmcMouse.cY < MAZETYPE - 1)
		{
			if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & 0x01) == 0x00)
			{
				GucMapBlock[GmcMouse.cX][GmcMouse.cY + 1] = ((GucMapBlock[GmcMouse.cX][GmcMouse.cY + 1]) & 0xbf);
			}
		}
	}
}
// 右手法则选动作
void rightMethod(void)
{
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_R) &&
		(mazeIsSearched(MOUSERIGHT) == 0))
	{
		StepTurnRight();
		MoveOneBlock();
		return;
	}
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_F) &&
		(mazeIsSearched(MOUSEFRONT) == 0))
	{
		MoveOneBlock();
		return;
	}
	if ((GucMapBlock[GmcMouse.cX][GmcMouse.cY] & MOUSEWAY_L) &&
		(mazeIsSearched(MOUSELEFT) == 0))
	{
		StepTurnLeft();
		MoveOneBlock();
		return;
	}
	else
	{
		TurnBack();
		return;
	}
}
// 检查相对方向的迷宫格，是否搜索过
int mazeIsSearched(int  ucDirTemp)
{
	int cX = 0, cY = 0;

	switch (ucDirTemp) {

	case MOUSEFRONT:
		ucDirTemp = GucMouseDir;
		break;

	case MOUSELEFT:
		ucDirTemp = (GucMouseDir + 3) % 4;
		break;

	case MOUSERIGHT:
		ucDirTemp = (GucMouseDir + 1) % 4;
		break;

	default:
		break;
	}

	switch (ucDirTemp) {

	case 0:
		cX = GmcMouse.cX;
		cY = GmcMouse.cY + 1;
		break;

	case 1:
		cX = GmcMouse.cX + 1;
		cY = GmcMouse.cY;
		break;

	case 2:
		cX = GmcMouse.cX;
		cY = GmcMouse.cY - 1;
		break;

	case 3:
		cX = GmcMouse.cX - 1;
		cY = GmcMouse.cY;
		break;

	default:
		break;
	}

	return(GucMapGet[cX][cY]);
}
// 计算可前进方向数目
int crosswayCheck(char cX, char cY)
{
	int moveDirNum = 0;

	if ((GucMapBlock[cX][cY] & 0x01) &&
		(GucMapGet[cX][cY + 1] == 0)) {
		moveDirNum++;
	}
	if ((GucMapBlock[cX][cY] & 0x02) &&
		(GucMapGet[cX + 1][cY] == 0)) {
		moveDirNum++;
	}
	if ((GucMapBlock[cX][cY] & 0x04) &&
		(GucMapGet[cX][cY - 1] == 0)) {
		moveDirNum++;
	}
	if ((GucMapBlock[cX][cY] & 0x08) &&
		(GucMapGet[cX - 1][cY] == 0)) {
		moveDirNum++;
	}

	//printf("crosswayCheackNum(%d,%d) = %d\n", GmcMouse.cX, GmcMouse.cY,moveDirNum);

	return moveDirNum;
}
// 计算洪水数据
void mapStepEdit(char  cX, char  cY)
{
	uchar n = 0;
	uchar ucStep = 0;
	uchar ucStat = 0;
	uchar i, j;

	MAZECOOR GmcStack[MAZETYPE * MAZETYPE] = { 0 };

	GmcStack[n].cX = cX;
	GmcStack[n].cY = cY;
	n++;


	for (i = 0; i < MAZETYPE; i++)
	{
		for (j = 0; j < MAZETYPE; j++)
		{
			GucMapStep[i][j] = 255;
		}
	}

	while (n)
	{
		GucMapStep[cX][cY] = ucStep++;

		ucStat = 0;
		if ((GucMapBlock[cX][cY] & 0x01) && (GucMapStep[cX][cY + 1] > (ucStep)))
		{
			ucStat++;
		}
		if ((GucMapBlock[cX][cY] & 0x02) && (GucMapStep[cX + 1][cY] > (ucStep)))
		{
			ucStat++;
		}
		if ((GucMapBlock[cX][cY] & 0x04) && (GucMapStep[cX][cY - 1] > (ucStep)))
		{
			ucStat++;
		}
		if ((GucMapBlock[cX][cY] & 0x08) && (GucMapStep[cX - 1][cY] > (ucStep)))
		{
			ucStat++;
		}

		if (ucStat == 0)
		{
			n--;
			cX = GmcStack[n].cX;
			cY = GmcStack[n].cY;
			ucStep = GucMapStep[cX][cY];
		}
		else
		{
			if (ucStat > 1)
			{
				GmcStack[n].cX = cX;
				GmcStack[n].cY = cY;
				n++;
			}

			if ((GucMapBlock[cX][cY] & 0x01) && (GucMapStep[cX][cY + 1] > (ucStep)))
			{
				cY++;
				continue;
			}
			if ((GucMapBlock[cX][cY] & 0x02) && (GucMapStep[cX + 1][cY] > (ucStep)))
			{
				cX++;
				continue;
			}
			if ((GucMapBlock[cX][cY] & 0x04) && (GucMapStep[cX][cY - 1] > (ucStep)))
			{
				cY--;
				continue;
			}
			if ((GucMapBlock[cX][cY] & 0x08) && (GucMapStep[cX - 1][cY] > (ucStep)))
			{
				cX--;
				continue;
			}
		}
	}
}
// 根据最短路径引导至目标点（本次新增）
void objectGoTo(char  cXdst, char  cYdst)
{
	int ucStep = 1;
	char cNBlock = 0, cDirTemp;
	char cX, cY;

	cX = GmcMouse.cX;
	cY = GmcMouse.cY;
	mapStepEdit(cXdst, cYdst);

	while ((cX != cXdst) || (cY != cYdst))
	{
		ucStep = GucMapStep[cX][cY];

		if ((GucMapBlock[cX][cY] & 0x01) && (GucMapStep[cX][cY + 1] == ucStep - 1))
		{
			cDirTemp = UP;
			if (cDirTemp == GucMouseDir)
			{
				cNBlock++;
				cY++;
				continue;
			}
		}
		if ((GucMapBlock[cX][cY] & 0x02) && (GucMapStep[cX + 1][cY] == ucStep - 1))
		{
			cDirTemp = RIGHT;
			if (cDirTemp == GucMouseDir)
			{
				cNBlock++;
				cX++;
				continue;
			}
		}
		if ((GucMapBlock[cX][cY] & 0x04) && (GucMapStep[cX][cY - 1] == ucStep - 1))
		{
			cDirTemp = DOWN;
			if (cDirTemp == GucMouseDir)
			{
				cNBlock++;
				cY--;
				continue;
			}
		}
		if ((GucMapBlock[cX][cY] & 0x08) && (GucMapStep[cX - 1][cY] == ucStep - 1))
		{
			cDirTemp = LEFT;
			if (cDirTemp == GucMouseDir)
			{
				cNBlock++;
				cX--;
				continue;
			}
		}

		cDirTemp = (cDirTemp + 4 - GucMouseDir) % 4;

		if (cNBlock)
		{
			for (int i = 0; i < cNBlock; i++)
			{
				MoveOneBlock();
			}

		}
		cNBlock = 0;

		switch (cDirTemp)
		{

		case 1:
			StepTurnRight();
			break;

		case 2:
			TurnBack();
			break;

		case 3:
			StepTurnLeft();
			break;

		default:
			break;
		}

		cX = GmcMouse.cX;
		cY = GmcMouse.cY;
	}
	if (cNBlock)
	{
		for (int i = 0; i < cNBlock; i++)
		{
			MoveOneBlock();
		}
	}
}
