#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C" {
#endif
#define IN1  26  // Right Motor +
#define IN2  25  // Right Motor -
#define IN3  27  // Left Motor +
#define IN4  14  // Left Motor -

//ENCODER
#define S_DA 21
#define S_CLK 22

void moveForward( );
void turnLeft();
void turnRight();
void stopMotors();




#ifdef __cplusplus
}
#endif

#endif
