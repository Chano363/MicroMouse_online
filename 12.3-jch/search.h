#pragma once
#include "user.h"

// 添加迷宫特征分析结构
typedef struct {
    float coverage;           // 当前覆盖率
    float explorationEfficiency; // 探索效率（单位步数的覆盖率提升）
    int deadEndCount;         // 死胡同数量
    int longCorridorCount;    // 长直道数量
    float connectivity;       // 迷宫连通性评分
    int decisionThreshold;    // 动态决策阈值（0-100）
} MazeFeatures;

extern MazeFeatures G_mazeFeatures;

extern int GucContinueSearchCount;
extern int GucContinueSearchSteps;
extern float GucSearchCoverage;
extern MAZECOOR visitedHistory[20];
extern int visitedHistoryIndex;
extern float GucStartSearchCoverage;  // 开始继续搜索时的覆盖率

float calculateCoverage(void);
int shouldContinueSearch(void);
int shouldStartContinueSearch(void);
void continueSearchMethod(void);
void analyzeMazeFeatures(void);
int use_tw(void);