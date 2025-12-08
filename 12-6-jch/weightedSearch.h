#pragma once
#include "user.h"

#define MAX_PATH_LENGTH (MAZETYPE * MAZETYPE)

struct wsNode {
    int cX, cY;
    int f, g, h;
    int parentX, parentY;
    int closed;
};
typedef struct wsNode wsNode;

// 函数声明
int weightedSearch(int startX, int startY, int goalX, int goalY, MAZECOOR* path, int* pathLength);
void ws_Method(void);
int Manhattan(int x1, int y1, int x2, int y2);
void getNeighbors(int x, int y, MAZECOOR neighbors[], int* count);
void ws_Path(MAZECOOR* path, int* pathLength);

// 外部函数声明（在user.c中定义）
extern MAZECOOR findNearestUnexplored(void);
extern void floodFillMethod(void);