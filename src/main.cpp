#include <Arduino.h>
#include"queue.h"
#include"move.h"
#include"sensor.h"
#include"flood_fill.h"
#define LED 2


// put function declarations here:
//int myFunction(int, int);

///////////////////////////////////////////////////////////////
extern int curr_x ;
 extern int curr_y ;

void setup() {
  // put your setup code here, to run once:
  //int result = myFunction(2, 3);
  //pinMode(LED,OUTPUT);
  pinMode(SENSOR_FRONT,INPUT);
  pinMode(SENSOR_LEFT,INPUT);
  pinMode(SENSOR_RIGHT,INPUT);
  ///////////////////////////////////
       initWalls();
       floodFill();
}

void loop() {
  if(flag==0){
     senseWalls();
     floodFill();

     stepToLowestNeighbor();


    if ((curr_x==7 && curr_y==7) || (curr_x==7 && curr_y==8) || (curr_x==8 && curr_y==7) || (curr_x==8 && curr_y==8)) {
                     

                     stopMotors();
                  }}
                  else{
                    
                  }
              
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
