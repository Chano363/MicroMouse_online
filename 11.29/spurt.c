#include "spurt.h"
#include "aStar.h"
#include <stdio.h>

// 声明外部变量和函数
extern int GucMouseTask;
extern MAZECOOR GmcMouse;
extern int GucMouseDir;
extern int GucXStart, GucYStart, GoalX, GoalY;

extern void StepTurnRight(void);
extern void StepTurnLeft(void);
extern void TurnBack(void);
extern void MoveOneBlock(void);

// 动作规划结构
typedef struct {
    int straightMoves;  // 连续直线移动次数
    int turnType;       // 0:无转向, 1:右转, 2:掉头, 3:左转
    int totalSteps;     // 总步数
} ActionPlan;

void Spurt(void) {
    static int sprintPhase = 0;
    static MAZECOOR optimizedPath[MAX_PATH_LENGTH];
    static int pathLength = 0;
    static int currentPathIndex = 0;
    static MAZECOOR path1[MAX_PATH_LENGTH], path2[MAX_PATH_LENGTH];
    static int len1 = 0, len2 = 0;
    static int segmentStage = 0;
    static MAZECOOR segmentPath[MAX_PATH_LENGTH];
    static int segmentLength = 0;
    static int segmentIndex = 0;

    switch (sprintPhase) {
    case 0:  // 预计算完整往返路径
        printf("SPURT: 预计算完整往返路径\n");

        // 计算两段路径：当前位置→起点，起点→终点
        len1 = 0; len2 = 0;

        if (aStarSearch(GmcMouse.cX, GmcMouse.cY, GucXStart, GucYStart, path1, &len1) &&
            aStarSearch(GucXStart, GucYStart, GoalX, GoalY, path2, &len2)) {

            // 合并路径（去掉重复的起点）
            pathLength = len1 + len2 - 1;
            for (int i = 0; i < len1; i++) {
                optimizedPath[i] = path1[i];
            }
            for (int i = 1; i < len2; i++) {
                optimizedPath[len1 + i - 1] = path2[i];
            }

            printf("SPURT: 完整路径计算完成，总长度=%d\n", pathLength);
            sprintPhase = 1;
            currentPathIndex = 1; // 从第一个移动开始
        }
        else {
            // 计算失败，回退到分段执行
            printf("SPURT: 路径计算失败，使用分段模式\n");
            sprintPhase = 10; // 跳到分段模式
        }
        break;

    case 1:  // 执行优化路径
        if (currentPathIndex < pathLength) {
            executeUltraOptimizedStep(optimizedPath, pathLength, &currentPathIndex);

            // 检查是否到达终点
            if (GmcMouse.cX == GoalX && GmcMouse.cY == GoalY) {
                printf("SPURT: 到达终点，任务完成\n");
                GucMouseTask = END;
            }
        }
        else {
            printf("SPURT: 路径执行完成但未到达终点\n");
            GucMouseTask = END;
        }
        break;

    case 10:  // 分段执行备用方案
        switch (segmentStage) {
        case 0:  // 返回起点段
            if (segmentLength == 0) {
                if (aStarSearch(GmcMouse.cX, GmcMouse.cY, GucXStart, GucYStart, segmentPath, &segmentLength)) {
                    segmentIndex = 1;
                }
                else {
                    printf("SPURT: 分段模式也失败\n");
                    GucMouseTask = END;
                }
            }
            if (segmentIndex < segmentLength) {
                executeUltraOptimizedStep(segmentPath, segmentLength, &segmentIndex);
            }
            if (GmcMouse.cX == GucXStart && GmcMouse.cY == GucYStart) {
                printf("SPURT: 到达起点，转向准备冲刺\n");
                TurnBack();
                segmentStage = 1;
                segmentLength = 0;
                segmentIndex = 0;
            }
            break;

        case 1:  // 冲刺终点段
            if (segmentLength == 0) {
                if (aStarSearch(GmcMouse.cX, GmcMouse.cY, GoalX, GoalY, segmentPath, &segmentLength)) {
                    segmentIndex = 1;
                }
                else {
                    printf("SPURT: 冲刺路径计算失败\n");
                    GucMouseTask = END;
                }
            }
            if (segmentIndex < segmentLength) {
                executeUltraOptimizedStep(segmentPath, segmentLength, &segmentIndex);
            }
            if (GmcMouse.cX == GoalX && GmcMouse.cY == GoalY) {
                printf("SPURT: 到达终点，任务完成\n");
                GucMouseTask = END;
            }
            break;
        }
        break;
    }
}

// 计算移动方向
int calculateMoveDirection(int fromX, int fromY, int toX, int toY) {
    if (toX == fromX) {
        return (toY > fromY) ? UP : DOWN;
    }
    else {
        return (toX > fromX) ? RIGHT : LEFT;
    }
}

// 分析连续动作序列
ActionPlan analyzeContinuousActions(MAZECOOR* path, int totalLength, int startIndex,
    int currentX, int currentY, int currentDir) {
    ActionPlan plan = { 0, 0, 0 };

    if (startIndex >= totalLength) return plan;

    // 计算第一步的方向
    int firstStepDir = calculateMoveDirection(currentX, currentY,
        path[startIndex].cX, path[startIndex].cY);

    // 计算转向需求
    plan.turnType = (firstStepDir - currentDir + 4) % 4;

    // 计算连续直线移动次数
    plan.straightMoves = 1;
    for (int i = startIndex + 1; i < totalLength; i++) {
        int nextDir = calculateMoveDirection(path[i - 1].cX, path[i - 1].cY,
            path[i].cX, path[i].cY);
        if (nextDir == firstStepDir) {
            plan.straightMoves++;
        }
        else {
            break;
        }
    }

    plan.totalSteps = plan.straightMoves;
    return plan;
}

// 超优化路径执行 - 预测并合并连续动作
void executeUltraOptimizedStep(MAZECOOR* path, int totalLength, int* currentIndex) {
    int startIndex = *currentIndex;
    int currentX = GmcMouse.cX;
    int currentY = GmcMouse.cY;
    int currentDir = GucMouseDir;

    // 分析接下来的连续动作
    ActionPlan plan = analyzeContinuousActions(path, totalLength, startIndex, currentX, currentY, currentDir);

    printf("执行优化步骤: 连续移动=%d, 转向=%d\n", plan.straightMoves, plan.turnType);

    // 执行转向（如果需要）
    if (plan.turnType != 0) {
        switch (plan.turnType) {
        case 1: StepTurnRight(); break;
        case 2: TurnBack(); break;
        case 3: StepTurnLeft(); break;
        }
    }

    // 执行连续直线移动
    for (int i = 0; i < plan.straightMoves; i++) {
        MoveOneBlock();
        (*currentIndex)++;
    }
}