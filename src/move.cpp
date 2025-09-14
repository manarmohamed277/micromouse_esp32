#include <Arduino.h>
#include"move.h"




volatile long encoderLeftTicks = 0;
volatile long encoderRightTicks = 0;

const int TICKS_PER_REV = 360;      // عدد البالس في لفة كاملة
const float WHEEL_DIAMETER = 0.06;  // قطر العجلة (متر)
const float WHEEL_CIRC = PI * WHEEL_DIAMETER;

// ---- PID ----
float Kp = 2.0, Ki = 0.2, Kd = 0.5;
float integral = 0, lastError = 0;



void IRAM_ATTR encoderLeftISR()  { encoderLeftTicks++; }
void IRAM_ATTR encoderRightISR() { encoderRightTicks++; }

void moveForward(float distance_m) {
  long targetTicks = (long)((distance_m / WHEEL_CIRC) * TICKS_PER_REV);

  long startLeft  = encoderLeftTicks;
  long startRight = encoderRightTicks;

  integral = 0;
  lastError = 0;

  while (true) {
    long movedLeft  = encoderLeftTicks - startLeft;
    long movedRight = encoderRightTicks - startRight;
    long movedAvg   = (movedLeft + movedRight) / 2;

    long error = targetTicks - movedAvg;
    if (error <= 0) break;

    integral += error;
    float derivative = error - lastError;
    float output = Kp*error + Ki*integral + Kd*derivative;
    lastError = error;

    int pwmVal = constrain((int)output, 0, 255);

    int balance = movedLeft - movedRight;
    int pwmLeft  = constrain(pwmVal - balance, 0, 255);
    int pwmRight = constrain(pwmVal + balance, 0, 255);

    ledcWrite(PWM_CHANNEL_LEFT, pwmLeft);
    ledcWrite(PWM_CHANNEL_RIGHT, pwmRight);

    Serial.print("Target: "); Serial.print(targetTicks);
    Serial.print("  Moved: "); Serial.print(movedAvg);
    Serial.print("  PWM L/R: "); Serial.print(pwmLeft);
    Serial.print(" / "); Serial.println(pwmRight);

    delay(10);
  }

  ledcWrite(PWM_CHANNEL_LEFT, 0);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
}












void stopMotors() {
ledcWrite(PWM_CHANNEL_LEFT, 0);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
}




void turnRight() {
  long targetTicks = 100;  // عدد ticks المطلوب للتدوير 90° (اضبطه تجريبيًا)
  long startLeft = encoderLeftTicks;
  long startRight = encoderRightTicks;

  while ((encoderLeftTicks - startLeft) < targetTicks) {
    // Left forward
    digitalWrite(MOTOR_LEFT_LOW, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, 150);

    // Right backward
    digitalWrite(MOTOR_RIGHT_LOW, HIGH);
    ledcWrite(PWM_CHANNEL_RIGHT, 150);

    delay(5);
  }

  stopMotors();
}

// ---- لف شمال 90° ----
void turnLeft() {
  long targetTicks = 100;  // نفس الشيء، اضبطه تجريبيًا
  long startLeft = encoderLeftTicks;
  long startRight = encoderRightTicks;

  while ((encoderRightTicks - startRight) < targetTicks) {
    // Left backward
    digitalWrite(MOTOR_LEFT_LOW, HIGH);
    ledcWrite(PWM_CHANNEL_LEFT, 150);

    // Right forward
    digitalWrite(MOTOR_RIGHT_LOW, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, 150);

    delay(5);
  }

  stopMotors();
}





