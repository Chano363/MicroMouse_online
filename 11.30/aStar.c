#include "aStar.h"
#include "flood.h"

extern int GoalGet, GoalX, GoalY, GucXStart, GucYStart;

// 启发式函数（曼哈顿距离）
int heuristic(int x1, int y1, int x2, int y2) {
	return abs(x1 - x2) + abs(y1 - y2);
}

// 获取可通行的邻居节点
void getNeighbors(int x, int y, MAZECOOR neighbors[], int* count) {
	*count = 0;

	// 上方向
	if ((GucMapBlock[x][y] & 0x01) && y + 1 < MAZETYPE) {
		neighbors[*count].cX = x;
		neighbors[*count].cY = y + 1;
		(*count)++;
	}

	// 右方向
	if ((GucMapBlock[x][y] & 0x02) && x + 1 < MAZETYPE) {
		neighbors[*count].cX = x + 1;
		neighbors[*count].cY = y;
		(*count)++;
	}

	// 下方向
	if ((GucMapBlock[x][y] & 0x04) && y - 1 >= 0) {
		neighbors[*count].cX = x;
		neighbors[*count].cY = y - 1;
		(*count)++;
	}

	// 左方向
	if ((GucMapBlock[x][y] & 0x08) && x - 1 >= 0) {
		neighbors[*count].cX = x - 1;
		neighbors[*count].cY = y;
		(*count)++;
	}
}

// A*路径搜索算法
int aStarSearch(int startX, int startY, int goalX, int goalY, MAZECOOR* path, int* pathLength) {
	AStarNode nodes[MAZETYPE][MAZETYPE];
	MAZECOOR openList[MAZETYPE * MAZETYPE];
	int openListCount = 0;

	// 初始化节点
	for (int i = 0; i < MAZETYPE; i++) {
		for (int j = 0; j < MAZETYPE; j++) {
			nodes[i][j].cX = i;
			nodes[i][j].cY = j;
			nodes[i][j].f = 9999;
			nodes[i][j].g = 9999;
			nodes[i][j].h = 0;
			nodes[i][j].parentX = -1;
			nodes[i][j].parentY = -1;
			nodes[i][j].closed = 0;
		}
	}

	// 设置起始节点
	nodes[startX][startY].g = 0;
	nodes[startX][startY].h = heuristic(startX, startY, goalX, goalY);
	nodes[startX][startY].f = nodes[startX][startY].g + nodes[startX][startY].h;

	// 将起始节点加入开放列表
	openList[openListCount].cX = startX;
	openList[openListCount].cY = startY;
	openListCount++;

	while (openListCount > 0) {
		// 找到开放列表中f值最小的节点
		int currentIndex = 0;
		int minF = nodes[openList[0].cX][openList[0].cY].f;

		for (int i = 1; i < openListCount; i++) {
			if (nodes[openList[i].cX][openList[i].cY].f < minF) {
				minF = nodes[openList[i].cX][openList[i].cY].f;
				currentIndex = i;
			}
		}

		int currentX = openList[currentIndex].cX;
		int currentY = openList[currentIndex].cY;

		// 如果到达目标，构建路径
		if (currentX == goalX && currentY == goalY) {
			// 回溯构建路径
			*pathLength = 0;
			int tempX = currentX, tempY = currentY;

			while (tempX != -1 && tempY != -1) {
				path[*pathLength].cX = tempX;
				path[*pathLength].cY = tempY;
				(*pathLength)++;

				int parentX = nodes[tempX][tempY].parentX;
				int parentY = nodes[tempX][tempY].parentY;
				tempX = parentX;
				tempY = parentY;
			}

			// 反转路径（从起点到终点）
			for (int i = 0; i < *pathLength / 2; i++) {
				MAZECOOR temp = path[i];
				path[i] = path[*pathLength - 1 - i];
				path[*pathLength - 1 - i] = temp;
			}

			return 1; // 找到路径
		}

		// 从开放列表移除当前节点
		openList[currentIndex] = openList[openListCount - 1];
		openListCount--;
		nodes[currentX][currentY].closed = 1;

		// 检查邻居节点
		MAZECOOR neighbors[4];
		int neighborCount;
		getNeighbors(currentX, currentY, neighbors, &neighborCount);

		for (int i = 0; i < neighborCount; i++) {
			int neighborX = neighbors[i].cX;
			int neighborY = neighbors[i].cY;

			if (nodes[neighborX][neighborY].closed) {
				continue;
			}

			int tentativeG = nodes[currentX][currentY].g + 1;

			if (tentativeG < nodes[neighborX][neighborY].g) {
				// 找到更优路径
				nodes[neighborX][neighborY].parentX = currentX;
				nodes[neighborX][neighborY].parentY = currentY;
				nodes[neighborX][neighborY].g = tentativeG;
				nodes[neighborX][neighborY].h = heuristic(neighborX, neighborY, goalX, goalY);
				nodes[neighborX][neighborY].f = nodes[neighborX][neighborY].g + nodes[neighborX][neighborY].h;

				// 如果不在开放列表中，加入
				int inOpenList = 0;
				for (int j = 0; j < openListCount; j++) {
					if (openList[j].cX == neighborX && openList[j].cY == neighborY) {
						inOpenList = 1;
						break;
					}
				}

				if (!inOpenList) {
					openList[openListCount].cX = neighborX;
					openList[openListCount].cY = neighborY;
					openListCount++;
				}
			}
		}
	}

	return 0; // 未找到路径
}
// 基于A*的智能搜索方法
void aStarMethod(void) {
	int targetX, targetY;

	// 确定目标点
	if (GoalGet) {
		targetX = GoalX;
		targetY = GoalY;
	}
	else {
		MAZECOOR nearestUnexplored = findNearestUnexplored();
		if (nearestUnexplored.cX != -1) {
			targetX = nearestUnexplored.cX;
			targetY = nearestUnexplored.cY;
			printf("A*目标: 未探索区域(%d,%d)\n", targetX, targetY);
		}
		else {
			// 所有区域都已探索，前往中心
			targetX = 7;
			targetY = 7;
		}
	}

	int distance = abs(targetX - GmcMouse.cX) + abs(targetY - GmcMouse.cY);
	if (distance > 8) {
		printf("目标过远(%d)，使用洪水填充\n", distance);
		floodFillMethod();
		return;
	}

	MAZECOOR path[MAX_PATH_LENGTH];
	int pathLength = 0;

	if (aStarSearch(GmcMouse.cX, GmcMouse.cY, targetX, targetY, path, &pathLength) && pathLength > 1) {
		// 找到路径，移动到下一个格子
		int nextX = path[1].cX;
		int nextY = path[1].cY;

		// 计算移动方向
		int moveDir;
		if (nextX == GmcMouse.cX) {
			if (nextY > GmcMouse.cY) moveDir = UP;
			else moveDir = DOWN;
		}
		else {
			if (nextX > GmcMouse.cX) moveDir = RIGHT;
			else moveDir = LEFT;
		}

		// 计算转向角度并执行
		int turnAngle = (moveDir - GucMouseDir + 4) % 4;
		switch (turnAngle) {
		case 1: StepTurnRight(); break;
		case 2: TurnBack(); break;
		case 3: StepTurnLeft(); break;
		default: break;
		}

		MoveOneBlock();

	}
	else {
		// A*找不到路径，fallback到洪水填充或右手法则
		floodFillMethod();
	}
}