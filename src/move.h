#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C" {
#endif
#define MOTOR_RIGHT_LOW  25
#define  MOTOR_RIGHT_PWM  26
#define MOTOR_LEFT_LOW   27
#define  MOTOR_LEFT_PWM   14

#define ENCODER_LEFT_A   18
#define ENCODER_RIGHT_A  32

#define PWM_CHANNEL_LEFT   0
#define PWM_CHANNEL_RIGHT  1
#define PWM_FREQ           1000   
#define PWM_RESOLUTION     8  


void moveForward(float distance_m);
void turnLeft();
void turnRight();
void stopMotors();

void IRAM_ATTR encoderLeftISR()  ;
void IRAM_ATTR encoderRightISR() ;


#ifdef __cplusplus
}
#endif

#endif
