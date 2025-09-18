#ifndef FLOODFILL_H
#define FLOODFILL_H

#define N 8
#ifdef __cplusplus
extern "C" {
#endif

int hasWall(int x, int y, int dir);
void initWalls();
 void senseWalls();
 void floodFill();
 void stepToLowestNeighbor();
 void stepToLowestNeighbor_old();
 void exploreMaze() ;
 void stepToLowestNeighborStatic();
  void stepToLowestNeighbor_fewer_turns();

#ifdef __cplusplus
}
#endif

#endif