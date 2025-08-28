#ifndef FLOODFILL_H
#define FLOODFILL_H

#define N 16
#ifdef __cplusplus
extern "C" {
#endif

int hasWall(int x, int y, int dir);
void initWalls();
 void senseWalls();
 void floodFill();
 void stepToLowestNeighbor();

#ifdef __cplusplus
}
#endif

#endif