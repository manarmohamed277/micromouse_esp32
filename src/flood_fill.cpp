#include <Arduino.h>
#include"flood_fill.h"
#include"sensor.h"
#include"queue.h"
#include"move.h"


//////////////////////////////flood fill/////////////////////////////////


// directions: 0=N, 1=E, 2=S, 3=W
int dx[4] = {0,1,0,-1};
int dy[4] = {1,0,-1,0};

int curr_x = 0, curr_y = 0;
int Direction = 0;

byte visited[N][N] = {0};
byte walls[N][N] = {0};
int bestDir;

byte flood[N][N] ={0};
       /* {
                {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14},
                {13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
                {12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
                {11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
                {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
                {9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
                {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
                {7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
                {7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
                {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
                {9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
                {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
                {11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
                {12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
                {13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
                {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14}
        };*/
/************************see if there is a wall***********************/
int hasWall(int x, int y, int dir) {
    return walls[x][y] & (1 << dir);
} 

          /**********************************************************************/
          /***********************set bourders***********************************/
          void initWalls() {
              for (int i=0; i<N; i++)
                  for (int j=0; j<N; j++)
                      walls[i][j] = 0;

              for (int i=0; i<N; i++) {
                  walls[i][0]     |= 4;
                  walls[i][N-1]   |= 1;
                  walls[0][i]     |= 8;
                  walls[N-1][i]   |= 2;
              }
          }
          /***********************************************************************/
          /*********************see & add walls***********************************/
          void senseWalls() {

              if (readFrontSensor()) {
                  int nx = curr_x + dx[Direction];
                  int ny = curr_y + dy[Direction];
                  walls[curr_x][curr_y] |= (1 << Direction);
                 
                  //add walls in the other cells
                  if (nx>=0 && ny>=0 && nx<N && ny<N) {
                      int opp = (Direction + 2) % 4;
                      walls[nx][ny] |= (1 << opp);
                  }
              }


              if (readLeftSensor()) {
                  int dir = (Direction+3)%4;
                  int nx = curr_x + dx[dir];
                  int ny = curr_y + dy[dir];
                  walls[curr_x][curr_y] |= (1 << dir);
                
                  if (nx>=0 && ny>=0 && nx<N && ny<N) {
                      int opp = (dir + 2) % 4;
                      walls[nx][ny] |= (1 << opp);
                  }
              }


              if (readRightSensor()) {
                  int dir = (Direction+1)%4;
                  int nx = curr_x + dx[dir];
                  int ny = curr_y + dy[dir];
                  walls[curr_x][curr_y] |= (1 << dir);
            
                  if (nx>=0 && ny>=0 && nx<N && ny<N) {
                      int opp = (dir + 2) % 4;
                      walls[nx][ny] |= (1 << opp);

                  }
              }
          }

/***********************************************************************/
/*****************************flood fill********************************/
          void floodFill() {
              queue q;
              creatQueue(&q);

              for (int i=0;i<N;i++)
                  for (int j=0;j<N;j++)
                      visited[i][j] = 0;

            //API_setColor(curr_x,curr_y,'G');
              // mark goal (center 4 cells)
              visited[7][7]=1; visited[7][8]=1; visited[8][7]=1; visited[8][8]=1;

              queueEntry e1={7,7}; append(e1,&q);
              queueEntry e2={7,8}; append(e2,&q);
              queueEntry e3={8,7}; append(e3,&q);
              queueEntry e4={8,8}; append(e4,&q);

              while (!queueEmpty(&q)) {
                  queueEntry curr; serve(&curr,&q);
                  int cx=curr.x, cy=curr.y;
                  int val=flood[cx][cy];

                  for (int d=0; d<4; d++) {
                      int nx=cx+dx[d], ny=cy+dy[d];
                      if (nx>=0 && ny>=0 && nx<N && ny<N) {
                          if (!hasWall(cx,cy,d) && !visited[nx][ny]) {
                              flood[nx][ny]=val+1;
                              /**************************/

                              visited[nx][ny]=1;
                              queueEntry neigh={nx,ny};
                              append(neigh,&q);
                          }
                      }
                  }
              }
          }
          /******************************************************************************/

          /*******************************take an action*********************************/

          void stepToLowestNeighbor_old() {
               bestDir=-1; int bestVal=999;

              for (int d=0; d<4; d++) {
                  int nx=curr_x+dx[d], ny=curr_y+dy[d];
                  if (nx>=0 && ny>=0 && nx<N && ny<N) {
                      if (!hasWall(curr_x,curr_y,d)) {
                          if (flood[nx][ny] < bestVal) {
                              bestVal=flood[nx][ny];
                              bestDir=d;
                             //
                             // printf("best direction %d",d);
                          }
                      }
                  }
              }



              // directions: 0=N, 1=E, 2=S, 3=W
              if(Direction==0&&bestDir==1)
                  turnRight();
              else if(Direction==0&&bestDir==3)
                  turnLeft();

              else if(Direction==0&&bestDir==2)
              {
                  turnLeft();
                  turnLeft();
              }

              //////////////////////////////////////////////
              else if(Direction==1&&bestDir==0)
                  turnLeft();

              else if(Direction==1&&bestDir==3) {
                  turnLeft();
                  turnLeft();
              }
              else if(Direction==1&&bestDir==2)
                  turnRight();
              //////////////////////////////////////////////
              else if(Direction==3&&bestDir==0)
                  turnRight();
              else if(Direction==3&&bestDir==1){
                  turnRight();
                  turnRight();}
              else if(Direction==3&&bestDir==2)
                  turnLeft();

              /////////////////////////////////////////////
              else if(Direction==2&&bestDir==0){
                  turnLeft();
                  turnLeft();}
              else if(Direction==2&&bestDir==1)
                  turnLeft();
              else if(Direction==2&&bestDir==3)
                  turnRight();

            ////////////////////////////////////////////////
            moveForward(.14);
          }
          //////////////////////////////////////////////////////////////
           void exploreMaze() {
              int total = N * N;
              int visited_count = 0;

              // reset visited
              for (int i=0;i<N;i++)
                  for (int j=0;j<N;j++)
                      visited[i][j]=0;

              // أول خلية
              senseWalls();
              visited[curr_x][curr_y] = 1;
              visited_count++;

              while (visited_count < total) {
                  // دور على جار جديد
                  int found = 0;
                  for (int d=0; d<4; d++) {
                      int nx = curr_x + dx[d];
                      int ny = curr_y + dy[d];
                      if (nx<0 || ny<0 || nx>=N || ny>=N) continue;


                      if (!hasWall(curr_x,curr_y,d) && !visited[nx][ny]) {
                          
                          // لف بالـ ifs
                          if(Direction==0&&d==1) turnRight();
                          else if(Direction==0&&d==3) turnLeft();
                          else if(Direction==0&&d==2){ turnLeft(); turnLeft(); }

                          else if(Direction==1&&d==0) turnLeft();
                          else if(Direction==1&&d==3){ turnLeft(); turnLeft(); }
                          else if(Direction==1&&d==2) turnRight();

                          else if(Direction==2&&d==0){ turnLeft(); turnLeft(); }
                          else if(Direction==2&&d==1) turnLeft();
                          else if(Direction==2&&d==3) turnRight();

                          else if(Direction==3&&d==0) turnRight();
                          else if(Direction==3&&d==1){ turnRight(); turnRight(); }
                          else if(Direction==3&&d==2) turnLeft();

                          Direction = d;

                          moveForward(.14);
                          curr_x = nx; curr_y = ny;

                          senseWalls(); // مهم جداً: أول ما أوصل لخلية أسجل الحيطان
                          visited[curr_x][curr_y] ++;
                          visited_count++;
                         
                          found = 1;
                          break;
                      }
                  }

                  if (!found) {
                      

                      bestDir=-1; int bestVal=999;

                      for (int d=0; d<4; d++) {
                          int nx=curr_x+dx[d], ny=curr_y+dy[d];
                          if (nx>=0 && ny>=0 && nx<N && ny<N) {
                              if (!hasWall(curr_x,curr_y,d)) {
                                  if (visited[nx][ny] < bestVal) {
                                      bestVal=visited[nx][ny];
                                      bestDir=d;
                                      //
                                      // printf("best direction %d",d);
                                  }
                              }
                          }
                      }
                      

                      // directions: 0=N, 1=E, 2=S, 3=W
                      if((Direction==0)&&(bestDir==1))
                          turnRight();
                      else if(Direction==0&&bestDir==3)
                          turnLeft();

                      else if(Direction==0&&bestDir==2)
                      {
                          turnLeft();
                          turnLeft();
                      }

                          //////////////////////////////////////////////
                      else if(Direction==1&&bestDir==0)
                          turnLeft();

                      else if(Direction==1&&bestDir==3) {
                          turnLeft();
                          turnLeft();
                      }
                      else if(Direction==1&&bestDir==2)
                          turnRight();
                          //////////////////////////////////////////////
                      else if(Direction==3&&bestDir==0)
                          turnRight();
                      else if(Direction==3&&bestDir==1){
                          turnRight();
                          turnRight();}
                      else if(Direction==3&&bestDir==2)
                          turnLeft();

                          /////////////////////////////////////////////
                      else if(Direction==2&&bestDir==0){
                          turnLeft();
                          turnLeft();}
                      else if(Direction==2&&bestDir==1)
                          turnLeft();
                      else if(Direction==2&&bestDir==3)
                          turnRight();
                      else if (bestDir == -1) {
                          
                          return;
                      }

                      ////////////////////////////////////////////////
                      moveForward(.14);
                      visited[curr_x][curr_y]++;
                      // مفيش جار جديد: ارجع لورا
                   /*   if(Direction==0){ turnLeft(); turnLeft(); Direction=2; }
                      else if(Direction==1){ turnLeft(); turnLeft(); Direction=3; }
                      else if(Direction==2){ turnLeft(); turnLeft(); Direction=0; }
                      else if(Direction==3){ turnLeft(); turnLeft(); Direction=1; }

                      if (!hasWall(curr_x,curr_y,Direction)) {
                          moveForward();

                         // curr_x += dx[Direction];
                          //curr_y += dy[Direction];
                      }*/} /*else {
                          log("Maze fully explored or stuck!");
                          break;
                      }*/
                  }




             
          }

/*********************less turns more distans*****************************/
        void stepToLowestNeighbor() {



               bestDir=-1; int bestVal=999;
              int found = 0;
              for (int d=0; d<4; d++) {
                  int nx=curr_x+dx[d], ny=curr_y+dy[d];
                  if (nx>=0 && ny>=0 && nx<N && ny<N) {
                     if ((!hasWall(curr_x,curr_y,d))&&(!visited[nx][ny])) {
                         found=1;
                          if (flood[nx][ny] < bestVal) {
                              bestVal=flood[nx][ny];
                              bestDir=d;
                             //
                             // printf("best direction %d",d);
                          }
                      }
                  }
              }
             

              // directions: 0=N, 1=E, 2=S, 3=W
              if((Direction==0)&&(bestDir==1))
                  turnRight();
              else if(Direction==0&&bestDir==3)
                  turnLeft();

              else if(Direction==0&&bestDir==2)
              {
                  turnLeft();
                  turnLeft();
              }

              //////////////////////////////////////////////
              else if(Direction==1&&bestDir==0)
                  turnLeft();

              else if(Direction==1&&bestDir==3) {
                  turnLeft();
                  turnLeft();
              }
              else if(Direction==1&&bestDir==2)
                  turnRight();
              //////////////////////////////////////////////
              else if(Direction==3&&bestDir==0)
                  turnRight();
              else if(Direction==3&&bestDir==1){
                  turnRight();
                  turnRight();}
              else if(Direction==3&&bestDir==2)
                  turnLeft();

              /////////////////////////////////////////////
              else if(Direction==2&&bestDir==0){
                  turnLeft();
                  turnLeft();}
              else if(Direction==2&&bestDir==1)
                  turnLeft();
              else if(Direction==2&&bestDir==3)
                  turnRight();
              else if (bestDir == -1) {
                 // log("No move possible!");
                  bestDir=-1; int bestVal=999;

                  for (int d=0; d<4; d++) {
                      int nx=curr_x+dx[d], ny=curr_y+dy[d];
                      if (nx>=0 && ny>=0 && nx<N && ny<N) {
                          if (!hasWall(curr_x,curr_y,d)) {
                              if (visited[nx][ny] < bestVal) {
                                  bestVal=visited[nx][ny];
                                  bestDir=d;
                                  //
                                  // printf("best direction %d",d);
                              }
                          }
                      }
                  }
                 

                  // directions: 0=N, 1=E, 2=S, 3=W
                  if((Direction==0)&&(bestDir==1))
                      turnRight();
                  else if(Direction==0&&bestDir==3)
                      turnLeft();

                  else if(Direction==0&&bestDir==2)
                  {
                      turnLeft();
                      turnLeft();
                  }

                      //////////////////////////////////////////////
                  else if(Direction==1&&bestDir==0)
                      turnLeft();

                  else if(Direction==1&&bestDir==3) {
                      turnLeft();
                      turnLeft();
                  }
                  else if(Direction==1&&bestDir==2)
                      turnRight();
                      //////////////////////////////////////////////
                  else if(Direction==3&&bestDir==0)
                      turnRight();
                  else if(Direction==3&&bestDir==1){
                      turnRight();
                      turnRight();}
                  else if(Direction==3&&bestDir==2)
                      turnLeft();

                      /////////////////////////////////////////////
                  else if(Direction==2&&bestDir==0){
                      turnLeft();
                      turnLeft();}
                  else if(Direction==2&&bestDir==1)
                      turnLeft();
                  else if(Direction==2&&bestDir==3)
                      turnRight();
                  else if (bestDir == -1) {
                     
                      return;
                  }

                  ////////////////////////////////////////////////
                  moveForward(.14);
                  visited[curr_x][curr_y]++;

                  return;
              }

            ////////////////////////////////////////////////
            moveForward(.14);
              visited[curr_x][curr_y]=1;

          }

void stepToLowestNeighborStatic() {

        int bestDir = -1;
        int bestVal = 999;

        // الأول: دور على أقل قيمة
        for (int d = 0; d < 4; d++) {
            int nx = curr_x + dx[d], ny = curr_y + dy[d];
            if (nx >= 0 && ny >= 0 && nx < N && ny < N) {
                if (!hasWall(curr_x, curr_y, d)) {
                    if (flood[nx][ny] < bestVal) {
                        bestVal = flood[nx][ny];
                        bestDir = d;
                    }
                }
            }
        }

        // لو لقى أكتر من جار بنفس القيمة (tie-breaking)
        if (bestDir != -1) {
            for (int d = 0; d < 4; d++) {
                int nx = curr_x + dx[d], ny = curr_y + dy[d];
                if (nx >= 0 && ny >= 0 && nx < N && ny < N) {
                    if (!hasWall(curr_x, curr_y, d)) {
                        if (flood[nx][ny] == bestVal) {
                            // الأولوية للاتجاه الحالي
                            if (d == Direction) {
                                bestDir = d;
                                break;
                            }
                            // أولوية ثانوية: N=0, E=1, S=2, W=3
                            if (bestDir != Direction && d < bestDir) {
                                bestDir = d;
                            }
                        }
                    }
                }
            }
        }

        

        if (bestDir == -1) {
           
            return;
        }

        // نفس كود الدوران بتاعك بالظبط
        if ((Direction == 0) && (bestDir == 1))
            turnRight();
        else if (Direction == 0 && bestDir == 3)
            turnLeft();
        else if (Direction == 0 && bestDir == 2) {
            turnLeft();
            turnLeft();
        }
        else if (Direction == 1 && bestDir == 0)
            turnLeft();
        else if (Direction == 1 && bestDir == 3) {
            turnLeft();
            turnLeft();
        }
        else if (Direction == 1 && bestDir == 2)
            turnRight();
        else if (Direction == 3 && bestDir == 0)
            turnRight();
        else if (Direction == 3 && bestDir == 1) {
            turnRight();
            turnRight();
        }
        else if (Direction == 3 && bestDir == 2)
            turnLeft();
        else if (Direction == 2 && bestDir == 0) {
            turnLeft();
            turnLeft();
        }
        else if (Direction == 2 && bestDir == 1)
            turnLeft();
        else if (Direction == 2 && bestDir == 3)
            turnRight();

        moveForward(.14);
    }




/*********************less turns more distans*****************************/
          void stepToLowestNeighbor_fewer_turns() {
              bestDir = -1;
              int bestVal = 999;

              for (int d = 0; d < 4; d++) {
                  int nx = curr_x + dx[d];
                  int ny = curr_y + dy[d];
                  if (nx >= 0 && ny >= 0 && nx < N && ny < N) {
                      if (!hasWall(curr_x, curr_y, d)) {
                          int cost = flood[nx][ny];


                          if (d != Direction) {
                              cost += 1;
                          }


                          if (d == Direction && cost <= bestVal) {
                              bestVal = cost;
                              bestDir = d;
                          }
                          else if (cost < bestVal) {
                              bestVal = cost;
                              bestDir = d;
                          }
                      }
                  }
              }


              if (bestDir == -1) {
                  
                  return;
              }


              if (Direction == 0 && bestDir == 1) turnRight();
              else if (Direction == 0 && bestDir == 3) turnLeft();
              else if (Direction == 0 && bestDir == 2) { turnLeft(); turnLeft(); }

              else if (Direction == 1 && bestDir == 0) turnLeft();
              else if (Direction == 1 && bestDir == 3) { turnLeft(); turnLeft(); }
              else if (Direction == 1 && bestDir == 2) turnRight();

              else if (Direction == 3 && bestDir == 0) turnRight();
              else if (Direction == 3 && bestDir == 1) { turnRight(); turnRight(); }
              else if (Direction == 3 && bestDir == 2) turnLeft();

              else if (Direction == 2 && bestDir == 0) { turnLeft(); turnLeft(); }
              else if (Direction == 2 && bestDir == 1) turnLeft();
              else if (Direction == 2 && bestDir == 3) turnRight();


              moveForward(.14);
          }
          /*********************************************************************************/
          /*********************************************************************************/

/////////////////////////////////////////////////////////////////////////