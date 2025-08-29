#include <Arduino.h>
#include"queue.h"
#include"move.h"
#include"sensor.h"
#include"flood_fill.h"
#include <EEPROM.h>
//#define LED 2

//switch
#define switch_pin 2
//////////////////////////
#define EEPROM_SIZE 1024  
extern int flood[16][16];
/////////////////////////////
// put function declarations here:

void saveFloodToEEPROM() ;
  
void loadFloodFromEEPROM();

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
  pinMode(switch_pin,INPUT);
  //////////////////////////////////
  EEPROM.begin(EEPROM_SIZE);
  //////////////////////////////////
       initWalls();
       floodFill();
}

void loop() {
    int switchState = digitalRead(switch_pin);
  if(switchState==LOW){
     senseWalls();
     floodFill();

     stepToLowestNeighbor();


    if ((curr_x==7 && curr_y==7) || (curr_x==7 && curr_y==8) || (curr_x==8 && curr_y==7) || (curr_x==8 && curr_y==8)) {
                     

                     stopMotors();
                     saveFloodToEEPROM();

                  }
                 while (digitalRead(switch_pin) == LOW);  //stop here
                }        

    else if (switchState==HIGH)
                  
   {
       loadFloodFromEEPROM();
       stepToLowestNeighbor();


        if ((curr_x==7 && curr_y==7) || (curr_x==7 && curr_y==8) || (curr_x==8 && curr_y==7) || (curr_x==8 && curr_y==8)) {
                     

                     stopMotors();
                  }
                   while (digitalRead(switch_pin) == HIGH);
                  }
              
}

// put function definitions here:

void saveFloodToEEPROM() {
  int addr = 0;
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      int value = flood[i][j];
      EEPROM.put(addr, value);
      addr += sizeof(int);
    }
  }
  EEPROM.commit();
}

void loadFloodFromEEPROM() {
  int addr = 0;
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      int value;
      EEPROM.get(addr, value);
      flood[i][j] = value;
      addr += sizeof(int);
    }
  }
}
