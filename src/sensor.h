#ifndef SENSORS_H
#define SENSORS_H
#define SENSOR_FRONT 34
#define SENSOR_LEFT  35
#define SENSOR_RIGHT 32


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