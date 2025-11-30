#include<stdio.h>
#include"user.h"
#include"flood.h"
#include"aStar.h"
int GoalGet = 0;
int GoalX, GoalY;
int GucXStart = 0;
int GucYStart = 0;

int isBlockedByWall(int fromX, int fromY, int toX, int toY);
int shouldUseAStar();
void executeCompletePath(MAZECOOR* path, int pathLength);
MAZECOOR findNearestUnexplored(void);

int userMain(void)
{

	int temp;
	int canMoveNum;

	mouseInit();

	while (1)
	{
		printf("状态: %d, 位置: (%d,%d), 方向: %d, 目标获取: %d\n",
			GucMouseTask, GmcMouse.cX, GmcMouse.cY, GucMouseDir, GoalGet);

		if (GmcMouse.cX >= 6 && GmcMouse.cX <= 9 && GmcMouse.cY >= 6 && GmcMouse.cY <= 9) {
			printf("== 在终点附近 ==\n");
		}
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
			// 发现目标点
			if (((GmcMouse.cX == 7) || (GmcMouse.cX == 8)) &&
				((GmcMouse.cY == 7) || (GmcMouse.cY == 8)) && !GoalGet) {
				GoalGet = 1;
				GoalX = GmcMouse.cX;
				GoalY = GmcMouse.cY;
				printf("*** 目标发现: (%d,%d) ***\n", GoalX, GoalY);
			}

			if (GoalGet) {
				printf("*** 开始冲刺阶段 ***\n");
				GucMouseTask = SPURT;
				break;
			}

			canMoveNum = crosswayCheck(GmcMouse.cX, GmcMouse.cY);

			if (canMoveNum > 0) {
				if (shouldUseAStar()) {
					aStarMethod();
				}
				else {
					floodFillMethod();
				}
			}
			else {
				smartBacktrack();
			}
			break;

		case SPURT:
			printf("SPURT: 从(%d,%d)冲刺\n", GmcMouse.cX, GmcMouse.cY);

			// 直接使用A*找到回起点的最优路径
			MAZECOOR pathToStart[MAX_PATH_LENGTH];
			int pathLengthToStart = 0;

			if (aStarSearch(GmcMouse.cX, GmcMouse.cY, GucXStart, GucYStart,
				pathToStart, &pathLengthToStart)) {
				printf("找到回起点路径，长度=%d\n", pathLengthToStart);
				// 执行整个路径
				executeCompletePath(pathToStart, pathLengthToStart);
			}
			else {
				// A*失败，使用洪水填充
				objectGoTo(GucXStart, GucYStart);
			}

			// 到达起点后转向并冲刺到终点
			if (GmcMouse.cX == GucXStart && GmcMouse.cY == GucYStart) {
				printf("到达起点，转向并冲刺到终点\n");
				TurnBack();

				MAZECOOR pathToGoal[MAX_PATH_LENGTH];
				int pathLengthToGoal = 0;

				if (aStarSearch(GmcMouse.cX, GmcMouse.cY, GoalX, GoalY,
					pathToGoal, &pathLengthToGoal)) {
					executeCompletePath(pathToGoal, pathLengthToGoal);
				}
				else {
					objectGoTo(GoalX, GoalY);
				}

				if (GmcMouse.cX == GoalX && GmcMouse.cY == GoalY) {
					printf("*** 任务完成! ***\n");
					GucMouseTask = END;
				}
			}
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
	char cNBlock = 0, cDirTemp = -1;
	char cX, cY;

	cX = GmcMouse.cX;
	cY = GmcMouse.cY;
	mapStepEdit(cXdst, cYdst);

	while ((cX != cXdst) || (cY != cYdst))
	{
		ucStep = GucMapStep[cX][cY];
		cDirTemp = -1;

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

// 找到最近的未搜索格子
MAZECOOR findNearestUnexplored(void)
{
	MAZECOOR result = { -1, -1 };
	int minDistance = MAZETYPE * MAZETYPE;

	for (int i = 0; i < MAZETYPE; i++) {
		for (int j = 0; j < MAZETYPE; j++) {
			if (GucMapGet[i][j] == 0) {
				int distance = abs(i - GmcMouse.cX) + abs(j - GmcMouse.cY);
				if (distance < minDistance) {
					minDistance = distance;
					result.cX = i;
					result.cY = j;
				}
			}
		}
	}

	return result;
}
int shouldUseAStar(void) {
	MAZECOOR target = findNearestUnexplored();
	if (target.cX == -1) return 0;

	int distance = abs(target.cX - GmcMouse.cX) + abs(target.cY - GmcMouse.cY);

	if (distance <= 6 &&                          // 目标不太远
		!isBlockedByWall(GmcMouse.cX, GmcMouse.cY, target.cX, target.cY) && // 没有直接墙壁阻挡
		crosswayCheck(GmcMouse.cX, GmcMouse.cY) <= 2) { // 当前不是复杂路口
		return 1;
	}

	return 0;
}

int isBlockedByWall(int fromX, int fromY, int toX, int toY) {
	if (fromX == toX && abs(fromY - toY) == 1) {
		// 垂直相邻
		if (toY > fromY) return !(GucMapBlock[fromX][fromY] & 0x01); // 检查上墙
		else return !(GucMapBlock[fromX][fromY] & 0x04); // 检查下墙
	}
	else if (fromY == toY && abs(fromX - toX) == 1) {
		// 水平相邻  
		if (toX > fromX) return !(GucMapBlock[fromX][fromY] & 0x02); // 检查右墙
		else return !(GucMapBlock[fromX][fromY] & 0x08); // 检查左墙
	}
	return 0; // 不相邻，认为不阻挡
}
// 执行完整路径
void executeCompletePath(MAZECOOR* path, int pathLength) {
	printf("执行完整路径，长度=%d\n", pathLength);
	for (int i = 0; i < pathLength - 1; i++) {
		printf("路径段 %d: (%d,%d) -> (%d,%d)\n",
			i, path[i].cX, path[i].cY, path[i + 1].cX, path[i + 1].cY);

		// 计算移动方向
		int moveDir;
		if (path[i + 1].cX == path[i].cX) {
			moveDir = (path[i + 1].cY > path[i].cY) ? UP : DOWN;
		}
		else {
			moveDir = (path[i + 1].cX > path[i].cX) ? RIGHT : LEFT;
		}

		// 转向并移动
		int turnAngle = (moveDir - GucMouseDir + 4) % 4;
		switch (turnAngle) {
		case 1: StepTurnRight(); break;
		case 2: TurnBack(); break;
		case 3: StepTurnLeft(); break;
		default: break;
		}

		MoveOneBlock();

		// 验证位置
		if (GmcMouse.cX != path[i + 1].cX || GmcMouse.cY != path[i + 1].cY) {
			printf("!!! 移动未到达预期位置 !!!\n");
			break;
		}
	}
}