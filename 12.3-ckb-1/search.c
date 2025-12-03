#include "search.h"

extern int GoalGet;
extern int GoalX, GoalY;
extern int GucXStart;
extern int GucYStart;

int GucContinueSearchCount = 0;
int GucContinueSearchSteps = 0;
float GucSearchCoverage = 0.0f;
MAZECOOR visitedHistory[20];
int visitedHistoryIndex = 0;

void smartFloodFill(void);
void addToVisitedHistory(int x, int y);

// 计算当前搜索覆盖率
float calculateCoverage(void) {
    int explored = 0;
    int total = MAZETYPE * MAZETYPE;

    for (int i = 0; i < MAZETYPE; i++) {
        for (int j = 0; j < MAZETYPE; j++) {
            if (GucMapGet[i][j]) explored++;
        }
    }

    return (float)explored / total;
}

// 判断是否需要继续搜索
int shouldContinueSearch(void) {
    // 1. 检查覆盖率是否达到目标
    GucSearchCoverage = calculateCoverage();
    if (GucSearchCoverage > 0.95f) { // 达到95%覆盖率，可以结束
        return 0;
    }

    // 2. 检查是否有"关键"未探索区域
    MAZECOOR testTarget;
    if (!findCriticalUnexplored(&testTarget)) {
        return 0; // 没有关键未探索区域
    }

    // 3. 检查未探索区域是否值得探索
    int unexploredCount = 0;
    int nearUnexplored = 0;

    for (int i = 0; i < MAZETYPE; i++) {
        for (int j = 0; j < MAZETYPE; j++) {
            if (GucMapGet[i][j] == 0) {
                unexploredCount++;
                // 检查是否靠近已知路径
                if (abs(i - GoalX) + abs(j - GoalY) <= 8 ||
                    abs(i - GucXStart) + abs(j - GucYStart) <= 8) {
                    nearUnexplored++;
                }
            }
        }
    }

    // 未探索区域太少，或者没有靠近路径的未探索区域
    if (unexploredCount < 3 || nearUnexplored == 0) {
        return 0;
    }

    return 1;
}

// 寻找关键未探索区域（靠近已知路径的未探索格子）
int findCriticalUnexplored(MAZECOOR* target) {
    MAZECOOR bestTarget = { -1, -1 };
    int bestScore = -9999;

    for (int i = 0; i < MAZETYPE; i++) {
        for (int j = 0; j < MAZETYPE; j++) {
            if (GucMapGet[i][j] == 0) { // 未探索
                int score = calculateCriticalScore(i, j);

                if (score > bestScore) {
                    bestScore = score;
                    bestTarget.cX = i;
                    bestTarget.cY = j;
                }
            }
        }
    }

    if (bestTarget.cX != -1) {
        *target = bestTarget;
        return 1;
    }

    return 0;
}

int calculateCriticalScore(int x, int y) {
    int score = 0;

    // 1. 距离当前鼠标位置（增加权重，避免跑远）
    int distanceToMouse = abs(x - GmcMouse.cX) + abs(y - GmcMouse.cY);
    score -= distanceToMouse * 6;  // 从3增加到6，更强调距离

    // 2. 特别奖励：如果目标在当前的"前进方向"上
    int dx = x - GmcMouse.cX;
    int dy = y - GmcMouse.cY;

    // 计算目标相对于当前方向的位置
    if ((dx == 0 && dy > 0 && GucMouseDir == UP) ||      // 前方
        (dx > 0 && dy == 0 && GucMouseDir == RIGHT) ||
        (dx == 0 && dy < 0 && GucMouseDir == DOWN) ||
        (dx < 0 && dy == 0 && GucMouseDir == LEFT)) {
        score += 60;  // 大幅奖励前方目标
    }
    else if ((dx == 0 && dy > 0 && GucMouseDir == DOWN) ||  // 后方（掉头方向）
        (dx > 0 && dy == 0 && GucMouseDir == LEFT) ||
        (dx == 0 && dy < 0 && GucMouseDir == UP) ||
        (dx < 0 && dy == 0 && GucMouseDir == RIGHT)) {
        score -= 40;  // 大幅惩罚后方目标
    }

    // 3. 周围的已探索格子数量（保持高权重）
    int exploredNeighbors = 0;
    if (x > 0 && GucMapGet[x - 1][y]) exploredNeighbors++;
    if (x < MAZETYPE - 1 && GucMapGet[x + 1][y]) exploredNeighbors++;
    if (y > 0 && GucMapGet[x][y - 1]) exploredNeighbors++;
    if (y < MAZETYPE - 1 && GucMapGet[x][y + 1]) exploredNeighbors++;
    score += exploredNeighbors * 25;  // 保持25

    // 4. 距离起点（适当降低权重）
    int distanceToStart = abs(x - GucXStart) + abs(y - GucYStart);
    score += (MAZETYPE - distanceToStart);  // 适当权重

    // 5. 距离终点（适当权重）
    int distanceToGoal = abs(x - GoalX) + abs(y - GoalY);
    score += (MAZETYPE - distanceToGoal);  // 恢复适当权重

    // 6. 奖励关键路径
    if (abs(x - GucXStart) == abs(x - GoalX) ||
        abs(y - GucYStart) == abs(y - GoalY)) {
        score += 10;
    }

    // 7. 惩罚重复访问
    if (isRecentlyVisited(x, y)) {
        score -= 40;
    }

    // 8. 惩罚孤立区域
    if (exploredNeighbors == 0) {
        // 检查2步内是否有已探索区域
        int exploredInRange = 0;
        for (int dx2 = -2; dx2 <= 2; dx2++) {
            for (int dy2 = -2; dy2 <= 2; dy2++) {
                if (dx2 == 0 && dy2 == 0) continue;
                int nx = x + dx2;
                int ny = y + dy2;
                if (nx >= 0 && nx < MAZETYPE && ny >= 0 && ny < MAZETYPE) {
                    if (GucMapGet[nx][ny]) exploredInRange++;
                }
            }
        }
        if (exploredInRange < 2) {
            score -= 50;  // 孤立区域惩罚
        }
    }

    return score;
}
// 继续搜索方法（偏向于探索关键路径）
void continueSearchMethod(void) {
    MAZECOOR target;

    // 50%概率使用A*导航到关键区域，50%概率使用智能洪水填充
    if (findCriticalUnexplored(&target) && (rand() % 100 < 50)) {
        // 使用A*导航到这个关键未探索区域
        MAZECOOR path[MAX_PATH_LENGTH];
        int pathLength = 0;

        if (aStarSearch(GmcMouse.cX, GmcMouse.cY, target.cX, target.cY,
            path, &pathLength)) {
            smoothPath(path, &pathLength);

            if (pathLength > 1) {
                // 只执行下一步，而不是整个路径
                int nextX = path[1].cX;
                int nextY = path[1].cY;

                // 检查是否可以移动
                if (isBlockedByWall(GmcMouse.cX, GmcMouse.cY, nextX, nextY) == 0) {
                    // 计算并执行转向
                    int moveDir = calculateDirection(GmcMouse.cX, GmcMouse.cY, nextX, nextY);
                    int turnAngle = (moveDir - GucMouseDir + 4) % 4;

                    switch (turnAngle) {
                    case 1: StepTurnRight(); break;
                    case 2: TurnBack(); break;
                    case 3: StepTurnLeft(); break;
                    default: break;
                    }

                    MoveOneBlock();
                    addToVisitedHistory(GmcMouse.cX, GmcMouse.cY);
                    return;
                }
            }
        }
    }

    // 使用智能洪水填充
    smartFloodFill();
}

// 智能洪水填充（在继续搜索阶段使用）
void smartFloodFill(void) {
    // 计算到关键区域的洪水值
    MAZECOOR keyArea;

    // 降低随机性，更倾向于当前方向
    int randValue = rand() % 100;
    if (randValue < 70) {  // 70%概率保持当前关键区域
        // 检查当前前进方向上是否有未探索区域
        int checkX = GmcMouse.cX, checkY = GmcMouse.cY;
        switch (GucMouseDir) {
        case UP: checkY++; break;
        case RIGHT: checkX++; break;
        case DOWN: checkY--; break;
        case LEFT: checkX--; break;
        }

        if (checkX >= 0 && checkX < MAZETYPE && checkY >= 0 && checkY < MAZETYPE) {
            if (GucMapGet[checkX][checkY] == 0) {
                keyArea.cX = checkX;
                keyArea.cY = checkY;
            }
            else {
                keyArea.cX = GoalX;
                keyArea.cY = GoalY;
            }
        }
        else {
            keyArea.cX = GoalX;
            keyArea.cY = GoalY;
        }
    }
    else if (randValue < 85) {
        keyArea.cX = GucXStart;
        keyArea.cY = GucYStart;
    }
    else {
        keyArea.cX = GoalX;
        keyArea.cY = GoalY;
    }

    mapStepEdit(keyArea.cX, keyArea.cY);

    // 选择移动方向 - 特别奖励当前方向
    int bestDir = -1;
    int bestValue = -999999;

    int dirs[4][2] = { {0,1}, {1,0}, {0,-1}, {-1,0} };
    int masks[4] = { 0x01, 0x02, 0x04, 0x08 };
    int absoluteDirs[4] = { UP, RIGHT, DOWN, LEFT };

    for (int i = 0; i < 4; i++) {
        int nx = GmcMouse.cX + dirs[i][0];
        int ny = GmcMouse.cY + dirs[i][1];

        if (nx >= 0 && nx < MAZETYPE && ny >= 0 && ny < MAZETYPE) {
            if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] & masks[i]) {
                int value = 0;

                // 基础价值：洪水值
                if (GucMapStep[nx][ny] < 255) {
                    value -= GucMapStep[nx][ny] * 5;
                }
                else {
                    value -= 1000;
                }

                // 高奖励：如果目标未探索
                if (GucMapGet[nx][ny] == 0) {
                    value += 100;

                    MAZECOOR temp = { nx, ny };
                    int tempScore = calculateCriticalScore(nx, ny);
                    value += tempScore / 2;

                    // 特别奖励：如果是当前前进方向
                    if (absoluteDirs[i] == GucMouseDir) {
                        value += 80;  // 大幅奖励保持方向
                    }
                }
                else {
                    // 已探索区域，但如果可以直行，适当奖励
                    if (absoluteDirs[i] == GucMouseDir) {
                        value += 30;  // 奖励保持直线前进
                    }
                }

                // 惩罚：如果会导致重复探索
                if (isRecentlyVisited(nx, ny)) {
                    value -= 50;
                }

                // 转向惩罚
                int turnAngle = (absoluteDirs[i] - GucMouseDir + 4) % 4;
                if (turnAngle == 2) value -= 30;  // 增加掉头惩罚
                else if (turnAngle == 1 || turnAngle == 3) value -= 15;  // 增加转向惩罚

                if (value > bestValue) {
                    bestValue = value;
                    bestDir = i;
                }
            }
        }
    }

    // 执行移动
    if (bestDir != -1) {
        int turnAngle = (absoluteDirs[bestDir] - GucMouseDir + 4) % 4;
        switch (turnAngle) {
        case 1: StepTurnRight(); break;
        case 2: TurnBack(); break;
        case 3: StepTurnLeft(); break;
        default: break;
        }
        MoveOneBlock();
        addToVisitedHistory(GmcMouse.cX, GmcMouse.cY);
    }
    else {
        floodFillMethod();
    }
}

// 辅助函数：计算方向
int calculateDirection(int fromX, int fromY, int toX, int toY) {
    if (toX == fromX) {
        return (toY > fromY) ? UP : DOWN;
    }
    else {
        return (toX > fromX) ? RIGHT : LEFT;
    }
}

// 辅助函数：记录访问历史
void addToVisitedHistory(int x, int y) {
    visitedHistory[visitedHistoryIndex].cX = x;
    visitedHistory[visitedHistoryIndex].cY = y;
    visitedHistoryIndex = (visitedHistoryIndex + 1) % 20;
}

int isRecentlyVisited(int x, int y) {
    for (int i = 0; i < 20; i++) {
        if (visitedHistory[i].cX == x && visitedHistory[i].cY == y) {
            return 1;
        }
    }
    return 0;
}

int shouldUseAStar(void) {
    MAZECOOR target = findNearestUnexplored();
    if (target.cX == -1) return 0;

    int distance = abs(target.cX - GmcMouse.cX) + abs(target.cY - GmcMouse.cY);

    // 继续搜索阶段：更积极地使用A*
    if (GoalGet) {
        // 在继续搜索阶段，放宽A*使用条件
        return (distance <= 12 && !isBlockedByWall(GmcMouse.cX, GmcMouse.cY, target.cX, target.cY));
    }

    // 正常搜索阶段
    if (distance <= 6 &&
        !isBlockedByWall(GmcMouse.cX, GmcMouse.cY, target.cX, target.cY) &&
        crosswayCheck(GmcMouse.cX, GmcMouse.cY) <= 2) {
        return 1;
    }

    return 0;
}

MazeFeatures G_mazeFeatures = { 0 };

void analyzeMazeFeatures(void) {
    int totalCells = MAZETYPE * MAZETYPE;
    int exploredCells = 0;
    int deadEnds = 0;
    int longCorridors = 0;
    int connectivityScore = 0;

    // 遍历迷宫，分析特征
    for (int i = 0; i < MAZETYPE; i++) {
        for (int j = 0; j < MAZETYPE; j++) {
            if (GucMapGet[i][j]) exploredCells++;

            // 计算每个格子的连通方向数
            int openDirections = 0;
            if (GucMapBlock[i][j] & 0x01) openDirections++; // 上
            if (GucMapBlock[i][j] & 0x02) openDirections++; // 右
            if (GucMapBlock[i][j] & 0x04) openDirections++; // 下
            if (GucMapBlock[i][j] & 0x08) openDirections++; // 左

            // 死胡同检测（只有1个开放方向）
            if (openDirections == 1) deadEnds++;

            // 长直道检测（只有2个开放方向且方向相对）
            if (openDirections == 2) {
                // 检查是否为直线（上下或左右）
                int isStraight = 0;
                if ((GucMapBlock[i][j] & 0x01) && (GucMapBlock[i][j] & 0x04)) isStraight = 1; // 上下
                if ((GucMapBlock[i][j] & 0x02) && (GucMapBlock[i][j] & 0x08)) isStraight = 1; // 左右
                if (isStraight) longCorridors++;
            }

            connectivityScore += openDirections;
        }
    }

    // 计算特征值
    G_mazeFeatures.coverage = (float)exploredCells / totalCells;
    G_mazeFeatures.deadEndCount = deadEnds;
    G_mazeFeatures.longCorridorCount = longCorridors;
    G_mazeFeatures.connectivity = (float)connectivityScore / totalCells / 4.0f; // 归一化到0-1

    // 计算动态阈值（基于迷宫复杂度）
    G_mazeFeatures.decisionThreshold = calculateDynamicThreshold();

    printf("迷宫特征分析: 死胡同=%d, 长直道=%d, 连通性=%.2f, 动态阈值=%d%%\n",
        deadEnds, longCorridors, G_mazeFeatures.connectivity,
        G_mazeFeatures.decisionThreshold);
}

int calculateDynamicThreshold(void) {
    int baseThreshold = 80; 

    // 调整因素1：连通性越高，阈值可以越低（简单迷宫）
    // 连通性在0.3-0.7之间变化，对阈值影响±10%
    float connectivityFactor = (G_mazeFeatures.connectivity - 0.5f) * 20.0f;
    baseThreshold -= (int)connectivityFactor;

    // 调整因素2：死胡同越多，阈值越高（复杂迷宫需要更多搜索）
    // 每个死胡同增加0.2%的阈值，最多+10%
    int deadEndFactor = G_mazeFeatures.deadEndCount * 0.2f;
    if (deadEndFactor > 10) deadEndFactor = 10;
    baseThreshold += deadEndFactor;

    // 调整因素3：长直道越多，阈值可以越低（简单结构）
    // 每个长直道减少0.1%的阈值，最多-5%
    int corridorFactor = G_mazeFeatures.longCorridorCount * 0.1f;
    if (corridorFactor > 5) corridorFactor = 5;
    baseThreshold -= corridorFactor;

    // 边界限制
    if (baseThreshold < 60) baseThreshold = 60;   // 最低60%
    if (baseThreshold > 95) baseThreshold = 95;   // 最高95%

    return baseThreshold;
}

int shouldStartContinueSearch(void) {
    float coverage = calculateCoverage();

    // 因素1：绝对覆盖率太低必须继续
    if (coverage < 0.70f) return 1; // 低于70%必须继续

    // 因素2：动态阈值决策
    analyzeMazeFeatures();
    if (coverage * 100 < G_mazeFeatures.decisionThreshold) return 1;

    // 因素3：关键路径不完整
    if (!isCriticalPathExplored()) return 1;

    return 0;
}

int isCriticalPathExplored(void) {
    // 简单实现：检查从当前位置到起点和到终点的路径
    MAZECOOR path1[MAX_PATH_LENGTH], path2[MAX_PATH_LENGTH];
    int len1 = 0, len2 = 0;

    // 检查到起点的路径
    if (aStarSearch(GmcMouse.cX, GmcMouse.cY, GucXStart, GucYStart, path1, &len1)) {
        for (int i = 0; i < len1; i++) {
            if (!GucMapGet[path1[i].cX][path1[i].cY]) {
                printf("到起点的路径上有未探索格子\n");
                return 0;
            }
        }
    }

    // 检查到终点的路径（已经是终点，但还是检查）
    if (aStarSearch(GmcMouse.cX, GmcMouse.cY, GoalX, GoalY, path2, &len2)) {
        for (int i = 0; i < len2; i++) {
            if (!GucMapGet[path2[i].cX][path2[i].cY]) {
                printf("到终点的路径上有未探索格子\n");
                return 0;
            }
        }
    }

    return 1;
}