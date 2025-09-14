#include <Arduino.h>
#include "queue.h"
#include "move.h"
#include "sensor.h"
#include "flood_fill.h"
#include <EEPROM.h>
#include <DFRobot_BMI160.h>

//switch
#define switch_pin 20
#define switch_pin_go 19
//////////////////////////
#define EEPROM_SIZE 512   // flood + walls
extern byte flood[N][N];
extern byte walls[N][N];  
/////////////////////////////
// put function declarations here:

void saveDataToEEPROM();
void loadDataFromEEPROM();
void savewallsToEEPROM();
void savefloodToEEPROM();
void loadwallsFromEEPROM();
void loadfloodFromEEPROM();

///////////////////////////////////////////////////////////////
extern int curr_x;
extern int curr_y;
//extern DFRobot_BMI160 imu; 
void setup() {
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  ///////////////////////////////////
  pinMode(switch_pin, INPUT);
  //////////////////////////////////
  EEPROM.begin(EEPROM_SIZE);
  //////////////////////////////////
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_LEFT_PWM, PWM_CHANNEL_LEFT);
  ledcAttachPin(MOTOR_RIGHT_PWM, PWM_CHANNEL_RIGHT);

  // Encoders
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);

  pinMode(MOTOR_LEFT_LOW, OUTPUT);
pinMode(MOTOR_RIGHT_LOW, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRightISR, RISING);
  /////////////////////////////////
  
  

  ////////////////////////////////
  initWalls();
  floodFill();
}

void loop() {
/*  int switchState = digitalRead(switch_pin);

  if (switchState == LOW) {
    senseWalls();
    floodFill();
    stepToLowestNeighbor();

    if ((curr_x == 7 && curr_y == 7) || (curr_x == 7 && curr_y == 8) || 
        (curr_x == 8 && curr_y == 7) || (curr_x == 8 && curr_y == 8)) {
      stopMotors();
      saveDataToEEPROM();
    }
    while (digitalRead(switch_pin) == LOW);  // stop here
  } 
  else if (switchState == HIGH) {
    loadDataFromEEPROM();
    stepToLowestNeighbor();

    if ((curr_x == 7 && curr_y == 7) || (curr_x == 7 && curr_y == 8) || 
        (curr_x == 8 && curr_y == 7) || (curr_x == 8 && curr_y == 8)) {
      stopMotors();
    }
    while (digitalRead(switch_pin) == HIGH);
  }*/

  //////////////////////////////////////////////////////////////////////

  int switchState = digitalRead(switch_pin);
  int switchState_go = digitalRead(switch_pin_go);

  if (switchState == HIGH) {
   exploreMaze();
    if ((curr_x == 7 && curr_y == 7) || (curr_x == 7 && curr_y == 8) || 
        (curr_x == 8 && curr_y == 7) || (curr_x == 8 && curr_y == 8)) {
      stopMotors();
      savewallsToEEPROM();
    }
    while (digitalRead(switch_pin) == HIGH);  // stop here
  } 
  else if (switchState_go == HIGH) {
    loadwallsFromEEPROM();
    while (switch_pin)
    {
    floodFill();
    stepToLowestNeighborStatic();

    /*if ((curr_x == 7 && curr_y == 7) || (curr_x == 7 && curr_y == 8) || 
        (curr_x == 8 && curr_y == 7) || (curr_x == 8 && curr_y == 8)) {*/
        if(curr_x==(N/2)&&curr_y==(N/2)){
      stopMotors();
      savefloodToEEPROM();
    
    while (digitalRead(switch_pin) == HIGH);
        }
      }
  }
  ////////////////////////////
  else  {
    loadfloodFromEEPROM();
    while (1)
    {
      stepToLowestNeighbor();
    if ((curr_x == 7 && curr_y == 7) || (curr_x == 7 && curr_y == 8) || 
        (curr_x == 8 && curr_y == 7) || (curr_x == 8 && curr_y == 8)) {
      stopMotors();
    }
   
  }

}
}

// put function definitions here:
int addr = 0;

void savefloodToEEPROM() {
  addr=256;

  // save flood
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      int value = flood[i][j];
      EEPROM.put(addr, value);
      addr += sizeof(byte);
    }
  }
  EEPROM.commit();
}
void savewallsToEEPROM() {
  addr=0;
  // save walls
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      int value = walls[i][j];
      EEPROM.put(addr, value);
      addr += sizeof(byte);
    }
  }

  EEPROM.commit();
}

void loadwallsFromEEPROM() {
   addr = 0;

  // get walls
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      int value;
      EEPROM.get(addr, value);
      walls[i][j] = value;
      addr += sizeof(byte);
    }
  }}
  void loadfloodFromEEPROM() {
 addr = 256;
  // get flood
  for (int i = 0; i < 16; i++) {
    for (int j = 0; j < 16; j++) {
      byte value;
      EEPROM.get(addr, value);
      flood[i][j] = value;
      addr += sizeof(byte);
    }
  }
}
