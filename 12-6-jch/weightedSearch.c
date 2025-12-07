#include "weightedSearch.h"
#include "flood.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

extern int GoalGet, GoalX, GoalY, GucXStart, GucYStart;

// 优先队列
typedef struct {
    int x, y;
    int f; // f = g + h
} PriorityNode;

// 简单优先队列
typedef struct {
    PriorityNode* nodes;
    int capacity;
    int size;
} PriorityQueue;

// 优先队列各操作
void pq_init(PriorityQueue* pq, int capacity) {
    pq->nodes = (PriorityNode*)malloc(capacity * sizeof(PriorityNode));
    pq->capacity = capacity;
    pq->size = 0;
}

void pq_push(PriorityQueue* pq, int x, int y, int f) {
    if (pq->size >= pq->capacity) return;

    // 简单实现：插入到末尾，然后上浮调整
    int i = pq->size++;
    pq->nodes[i].x = x;
    pq->nodes[i].y = y;
    pq->nodes[i].f = f;

    // 上浮调整
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (pq->nodes[parent].f <= pq->nodes[i].f) break;

        // 交换
        PriorityNode temp = pq->nodes[parent];
        pq->nodes[parent] = pq->nodes[i];
        pq->nodes[i] = temp;
        i = parent;
    }
}

PriorityNode pq_pop(PriorityQueue* pq) {
    PriorityNode result = pq->nodes[0];
    pq->nodes[0] = pq->nodes[--pq->size];

    // 下沉调整
    int i = 0;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;

        if (left < pq->size && pq->nodes[left].f < pq->nodes[smallest].f)
            smallest = left;
        if (right < pq->size && pq->nodes[right].f < pq->nodes[smallest].f)
            smallest = right;
        if (smallest == i) break;

        PriorityNode temp = pq->nodes[i];
        pq->nodes[i] = pq->nodes[smallest];
        pq->nodes[smallest] = temp;
        i = smallest;
    }

    return result;
}

void pq_free(PriorityQueue* pq) {
    free(pq->nodes);
}

int pq_empty(PriorityQueue* pq) {
    return pq->size == 0;
}



// 返回曼哈顿距离
int Manhattan(int x1, int y1, int x2, int y2) 
{
    return abs(x1 - x2) + abs(y1 - y2);
}

// 获取可通行节点
void getNeighbors(int x, int y, MAZECOOR neighbors[], int* count) 
{
    *count = 0;

    // 上
    if ((GucMapBlock[x][y] & 0x01) && y + 1 < MAZETYPE)
    {
        neighbors[*count].cX = x;
        neighbors[*count].cY = y + 1;
        (*count)++;
    }

    // 左
    if ((GucMapBlock[x][y] & 0x08) && x - 1 >= 0)
    {
        neighbors[*count].cX = x - 1;
        neighbors[*count].cY = y;
        (*count)++;
    }

    // 右
    if ((GucMapBlock[x][y] & 0x02) && x + 1 < MAZETYPE) 
    {
        neighbors[*count].cX = x + 1;
        neighbors[*count].cY = y;
        (*count)++;
    }

    // 下
    if ((GucMapBlock[x][y] & 0x04) && y - 1 >= 0)
    {
        neighbors[*count].cX = x;
        neighbors[*count].cY = y - 1;
        (*count)++;
    }
}

// 转向惩罚
int whether_turn(int parentX, int parentY, int currentX, int currentY, int nextX, int nextY) 
{
    // 如果没有父节点/是起点，没有转向惩罚
    if (parentX == -1 || parentY == -1) return 0;

    // 计算从父节点到当前节点的方向
    int currentDirX = currentX - parentX;
    int currentDirY = currentY - parentY;

    // 计算从当前节点到下一个节点的方向
    int nextDirX = nextX - currentX;
    int nextDirY = nextY - currentY;

    // 方向相同，没有转向惩罚
    if (currentDirX == nextDirX && currentDirY == nextDirY) 
    {
        return 0;
    }

    // 方向不同，添加转向惩罚
    return 3; // 转向惩罚权重，可以调整
}

// 优化后的转向权重搜索算法（减少拐点）
int weightedSearch(int startX, int startY, int goalX, int goalY, MAZECOOR* path, int* pathLength) 
{
    wsNode nodes[MAZETYPE][MAZETYPE];
    MAZECOOR neighbors[4];
    PriorityQueue openList;
    int neighborCount;

    pq_init(&openList, MAZETYPE * MAZETYPE);

    // 初始化节点
    for (int i = 0; i < MAZETYPE; i++) {
        for (int j = 0; j < MAZETYPE; j++) {
            nodes[i][j].cX = i;
            nodes[i][j].cY = j;
            nodes[i][j].f = INT_MAX;
            nodes[i][j].g = INT_MAX;
            nodes[i][j].h = Manhattan(i, j, goalX, goalY);
            nodes[i][j].parentX = -1;
            nodes[i][j].parentY = -1;
            nodes[i][j].closed = 0;
        }
    }
    // 起始节点
    nodes[startX][startY].g = 0;
    nodes[startX][startY].f = nodes[startX][startY].g + nodes[startX][startY].h;
    pq_push(&openList, startX, startY, nodes[startX][startY].f);

    while (!pq_empty(&openList)) 
    {
        // 找到开放列表中f值最小的节点
        PriorityNode current = pq_pop(&openList);
        int currentX = current.x;
        int currentY = current.y;

        // 如果节点已关闭，跳过
        if (nodes[currentX][currentY].closed) 
        {
            continue;
        }

        nodes[currentX][currentY].closed = 1;

        // 到达终点
        if (currentX == goalX && currentY == goalY) 
        {
            // 回溯路径
            *pathLength = 0;
            int tempX = currentX;
            int tempY = currentY;

            while (tempX != -1 && tempY != -1) 
            {
                path[*pathLength].cX = tempX;
                path[*pathLength].cY = tempY;
                (*pathLength)++;

                int parentX = nodes[tempX][tempY].parentX;
                int parentY = nodes[tempX][tempY].parentY;
                tempX = parentX;
                tempY = parentY;
            }

            // 反转路径（起点到终点）
            for (int i = 0; i < *pathLength / 2; i++) 
            {
                MAZECOOR temp = path[i];
                path[i] = path[*pathLength - 1 - i];
                path[*pathLength - 1 - i] = temp;
            }

            pq_free(&openList);
            return 1; //返回已找到终点
        }

        // 检查邻居节点
        getNeighbors(currentX, currentY, neighbors, &neighborCount);

        for (int i = 0; i < neighborCount; i++) 
        {
            int neighborX = neighbors[i].cX;
            int neighborY = neighbors[i].cY;

            // 如果邻居节点已关闭，跳过
            if (nodes[neighborX][neighborY].closed) 
            {
                continue;
            }

            // 计算转向惩罚
            int turnPenalty = whether_turn
            (
                nodes[currentX][currentY].parentX,
                nodes[currentX][currentY].parentY,
                currentX, currentY,
                neighborX, neighborY
            );

            // 计算从起点经过当前节点到邻居的代价
            int tentative_g = nodes[currentX][currentY].g + 1 + turnPenalty;

            // 如果找到更优路径
            if (tentative_g < nodes[neighborX][neighborY].g) 
            {
                // 更新邻居节点
                nodes[neighborX][neighborY].parentX = currentX;
                nodes[neighborX][neighborY].parentY = currentY;
                nodes[neighborX][neighborY].g = tentative_g;
                nodes[neighborX][neighborY].f = tentative_g + nodes[neighborX][neighborY].h;

                // 将邻居加入开放列表
                pq_push(&openList, neighborX, neighborY, nodes[neighborX][neighborY].f);
            }
        }
    }

    pq_free(&openList);
    return 0; // 未找到路径
}

// 路径平滑函数（减少拐点）
void ws_Path(MAZECOOR* path, int* pathLength)
{
    if (*pathLength <= 2) return;

    int smoothed = 1;
    while (smoothed) 
    {
        smoothed = 0;
        int i = 0;

        while (i < *pathLength - 2) 
        {
            int x1 = path[i].cX, y1 = path[i].cY;
            int x2 = path[i + 1].cX, y2 = path[i + 1].cY;
            int x3 = path[i + 2].cX, y3 = path[i + 2].cY;

            // 检查三点是否共线
            int isCollinear = 0;
            if ((x1 == x2 && x2 == x3) ||  (y1 == y2 && y2 == y3))   
            { 
                isCollinear = 1;
            }

            // 检查从点1到点3是否有直接路径
            int hasDirectPath = 0;
            if (x1 == x3) 
            {
                int startY = (y1 < y3) ? y1 : y3;
                int endY = (y1 > y3) ? y1 : y3;
                hasDirectPath = 1;

                for (int y = startY; y < endY; y++) 
                {
                    if (!(GucMapBlock[x1][y] & 0x01)) { // 竖直方向
                        hasDirectPath = 0;
                        break;
                    }
                }
            }
            else if (y1 == y3) 
            {
                int startX = (x1 < x3) ? x1 : x3;
                int endX = (x1 > x3) ? x1 : x3;
                hasDirectPath = 1;

                for (int x = startX; x < endX; x++) {
                    if (!(GucMapBlock[x][y1] & 0x02)) { // 水平方向
                        hasDirectPath = 0;
                        break;
                    }
                }
            }

            // 如果可以跳过中间点
            if (isCollinear || hasDirectPath) {
                // 移除中间点
                for (int j = i + 1; j < *pathLength - 1; j++) {
                    path[j] = path[j + 1];
                }
                (*pathLength)--;
                smoothed = 1;
            }
            else {
                i++;
            }
        }
    }
}

// 基于转弯加权的智能搜索方法
void ws_Method(void) 
{
    int targetX, targetY;

    // 确定目标点
    if (GoalGet) 
    {
        targetX = GoalX;
        targetY = GoalY;
    }
    else 
    {
        MAZECOOR nearestUnexplored = findNearestUnexplored();
        if (nearestUnexplored.cX != -1) 
        {
            targetX = nearestUnexplored.cX;
            targetY = nearestUnexplored.cY;
        }
        else 
        {
            // 所有区域都已探索，前往中心
            targetX = 7;
            targetY = 7;
        }
    }

    // 使用转弯加权搜索路径
    MAZECOOR path[MAX_PATH_LENGTH];
    int pathLength = 0;

    if (weightedSearch(GmcMouse.cX, GmcMouse.cY, targetX, targetY, path, &pathLength)) 
    {
        // 路径平滑处理
        ws_Path(path, &pathLength);

        if (pathLength > 1) 
        {
            // 移动到下一个格子
            int nextX = path[1].cX;
            int nextY = path[1].cY;

            // 计算移动方向
            int moveDir;
            if (nextX == GmcMouse.cX) 
            {
                moveDir = (nextY > GmcMouse.cY) ? UP : DOWN;
            }
            else {
                moveDir = (nextX > GmcMouse.cX) ? RIGHT : LEFT;
            }

            // 计算转向角度并执行
            int turnAngle = (moveDir - GucMouseDir + 4) % 4;
            switch (turnAngle) 
            {
            case 1: StepTurnRight(); break;
            case 2: TurnBack(); break;
            case 3: StepTurnLeft(); break;
            default: break;
            }

            MoveOneBlock();
        }
    }
    else 
    {
        // 找不到路径，使用洪水填充
        floodFillMethod();
    }
}