#include <Arduino.h>
#include "queue.h"
#include "move.h"
#include "sensor.h"
#include "flood_fill.h"
#include <EEPROM.h>
#include "ESP32Encoder.h"
#include <Wire.h>
#include "DFRobot_BMI160.h"

//switch
#define switch_pin 35
#define switch_pin_go 34
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
extern int freq ;       // 1 kHz
extern int resolution ;    // 
extern float angleZ ;
extern ESP32Encoder rightEncoder;
extern ESP32Encoder leftEncoder;

extern DFRobot_BMI160 bmi160;
extern  int8_t i2c_addr ;
void setup() {
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  ///////////////////////////////////
  pinMode(switch_pin, INPUT);
  //////////////////////////////////
  //EEPROM.begin(EEPROM_SIZE);
  //////////////////////////////////
   pinMode(14,OUTPUT);
  digitalWrite(14,LOW);

  pinMode(27,OUTPUT);
  digitalWrite(27,LOW);

//I2C begin
  Wire.begin(S_DA, S_CLK); 

  Serial.begin(115200);
 
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  ledcSetup(2, freq, resolution);
  ledcSetup(3, freq, resolution);

  ledcAttachPin(IN1, 0);
  ledcAttachPin(IN2, 1);
  ledcAttachPin(IN3, 2);
  ledcAttachPin(IN4, 3);

  rightEncoder.attachFullQuad(32, 33);
  leftEncoder.attachFullQuad(18, 19);
  
  rightEncoder.clearCount();
  leftEncoder.clearCount();

  /////////////////////////////////
  if (bmi160.softReset() != BMI160_OK) {
   // Serial.println("BMI160 reset no");
   
    while (1);
  }

  if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
    
    while (1);
  }

  delay(5000);
  

  ////////////////////////////////
//digitalWrite(switch_pin,LOW);
  ////////////////////////////////
  initWalls();
  floodFill();
}

void loop() {

  //int switchState = digitalRead(switch_pin);

  //if (switchState == LOW) {
    senseWalls();
//delay(4000);
    delay(200);
    floodFill();
    stepToLowestNeighbor_old();

    if ((curr_x == 7 && curr_y == 7) || (curr_x == 7 && curr_y == 8) || 
        (curr_x == 8 && curr_y == 7) || (curr_x == 8 && curr_y == 8)) {
     
      stopMotors();
      while (1);

      
    }
  }
  /*if ((curr_x == 1 && curr_y == 1) || (curr_x == 1 && curr_y == 2) || 
        (curr_x == 2 && curr_y == 1) || (curr_x == 2 && curr_y == 2)) {*/
      // if(curr_x==2&&curr_y==2){
     // stopMotors();
      //while(1);
      //saveDataToEEPROM();
    
   /* while (digitalRead(switch_pin) == LOW);  // stop here
  } 
  else if (switchState == HIGH) {
    loadDataFromEEPROM();
    stepToLowestNeighbor();*/
   // while (digitalRead(switch_pin) == HIGH);
  //}

  //////////////////////////////////////////////////////////////////////

  /*int switchState = digitalRead(switch_pin);
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
        (curr_x == 8 && curr_y == 7) || (curr_x == 8 && curr_y == 8)) {
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
}*/

// put function definitions here:
/*int addr = 0;

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
  }*/

