#ifndef SENSORS_H
#define SENSORS_H
#define SENSOR_FRONT 15
#define SENSOR_LEFT  4
#define SENSOR_RIGHT 12


#ifdef __cplusplus
extern "C" {
#endif

int readFrontSensor();
int readLeftSensor();
int readRightSensor();

#ifdef __cplusplus
}
#endif

#endif