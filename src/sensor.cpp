#include <Arduino.h>
#include"sensor.h"

/////////////////////////////////SEE WALLS////////////////////////////

int readFrontSensor() {
  return digitalRead(SENSOR_FRONT);
}

int readLeftSensor() {
  return  digitalRead(SENSOR_LEFT);
}

int readRightSensor() {
  return  digitalRead(SENSOR_RIGHT);
}
/////////////////////////////////////////////////////////////////////////

