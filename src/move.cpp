#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_BMI160.h"
//#include "BluetoothSerial.h"
#include "ESP32Encoder.h"
#include "sensor.h"
#include "move.h"
  // بلوتوث

DFRobot_BMI160 bmi160;
 int8_t i2c_addr = 0x68;   // لو SDIO متوصل GND يبقى 0x68 , لو VCC يبقى 0x69
float gyroZ_offset = 0;
// BluetoothSerial SerialBT;

// ---- PWM settings ----
int freq = 2000;       // 1 kHz
int resolution = 8;    // 
float angleZ = 0;
unsigned long prevTime = 0;
///////////////encoder//////////
ESP32Encoder rightEncoder;
ESP32Encoder leftEncoder;

void rightMotorForward(int speed) {
  ledcWrite(0, speed);
  ledcWrite(1, 0);
}

void leftMotorForward(int speed) {
  ledcWrite(2, speed);
  ledcWrite(3, 0);
}

void leftMotorBackward(int speed) {
  ledcWrite(3, speed);
  ledcWrite(2, 0);
}

void rightMotorBackward(int speed) {
  ledcWrite(1, speed);
  ledcWrite(0, 0);
}

void stopMotors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

void rotate90Right(int speed) {
  angleZ = 0;
  prevTime = millis();

  // لف مكانك: يمين قدام + شمال ورا
  rightMotorForward(speed);
  leftMotorBackward(speed);

  while (abs(angleZ) < 90.0) {
    unsigned long currTime = millis();
    float dt = (currTime - prevTime) / 1000.0;
    prevTime = currTime;

    int16_t accelGyro[6] = {0};
    if (bmi160.getAccelGyroData(accelGyro) == 0) {
      float gz = accelGyro[2] / 16.4;  // rad/s
     // gz = gz - gyroZ_offset;                  // اطرح الـ offset
      angleZ += gz  * dt;      // اجمع بالدرجات
    }
    if (angleZ >= 90.0 || angleZ <= -90.0) {
    stopMotors();
    break;
}
  }

  stopMotors();
// SerialBT.println("Rotation 90 done!");
}
///////////////////////////////////////PID///////////////////////////////////
 float Kp = 2.0;    // proportional
 float Ki = 0.6;    // integral
 float Kd = 0.04;   // derivative
// PID rotate function
// targetDeg: الزاوية المطلوبة بالدرجات (مثلاً +90 ليمين، -90 لليسار)
// maxPWM: أقصى PWM تحب تسمح بيه (0..255)
// timeoutMs: مهلة أمان بالملّي ثانية
// returns true إذا وصل، false إذا timeout
bool rotateByAnglePID(float targetDeg, int maxPWM = 255, unsigned long timeoutMs = 6000) {
  // PID gains - ابدأ بالقيم دي وجرب تضبطها
  

  // anti-windup limits للـ integral term
  const float integratorMax = 100.0;
  const float integratorMin = -100.0;

  // conversion factor: RAW -> deg/s (لـ BMI160 ±2000dps)
  const float RAW_TO_DPS = 1.0f / 16.4f; // accelGyro raw /16.4 = deg/s

  float integrator = 0.0;
  float prevError = 0.0;
  float currentAngle = 0.0;    // درجة، ممكن تكون موجبة أو سالبة
  unsigned long prevMicros = micros();
  unsigned long startMs = millis();

  // safety: لو عايز تعمل معايرة قبل الحركة خليها بره الدالة
  // شغّل المواتير في البداية بصفر (تأمين)
  stopMotors();
  delay(10);

  //SerialBT.print("PID rotate start target=");
  //SerialBT.println(targetDeg);

  while (true) {
    // timeout safety
    if (millis() - startMs > timeoutMs) {
      stopMotors();
     // SerialBT.println("PID rotate TIMEOUT");
      return false;
    }

    unsigned long nowMicros = micros();
    float dt = (nowMicros - prevMicros) / 1000000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    prevMicros = nowMicros;

    // read gyro raw
    int16_t raw[6] = {0};
    if (bmi160.getAccelGyroData(raw) != 0) {
      // قراءة فشلت: استمر لكن برinted error
     // SerialBT.println("gyro read err");
      delay(2);
      continue;
    }

    // تحويل raw -> deg/s
    float gz_dps = raw[2] * RAW_TO_DPS; // deg/s (قد تكون موجبة/سالبة حسب اتجاه الدوران)

    // لو عندك offset مخزن حط طرحيه هنا (مثال)
    // gz_dps -= gyroZ_offset;

    // تكامل للحصول على الزاوية المقطوعة (بالدرجات)
    currentAngle += gz_dps * dt; // deg/s * s = deg

    // احسب الخطأ (target - current)
    float error = targetDeg - currentAngle;

    // PID terms
    integrator += error * dt;
    // anti-windup
    if (integrator > integratorMax) integrator = integratorMax;
    if (integrator < integratorMin) integrator = integratorMin;

    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = Kp * error + Ki * integrator + Kd * derivative;
    // output بوحدة درجات/ث او مجرد إشارة تحكم -> نحولها الى PWM magnitude
    // نأخذ القيمة المطلقة لاختيار السرعة، إشارة الإشارة تحدد الاتجاه
    float pwmCmd = fabs(output);

    // خريطة بسيطة: نحد pwmCmd بحيث لا يتعدى maxPWM
    if (pwmCmd > maxPWM) pwmCmd = maxPWM;

    // حد أدنى ليتغلب على احتكاك الموتور (deadzone)
    const int pwmMin = 30;
    if (pwmCmd < pwmMin) pwmCmd = pwmMin;

    // اختيار الاتجاه حسب إشارة output (لو output >0 -> لفة للاتجاه الموجب، وإلا العكس)
    // هنا هنفترض أن output >0 معناها نحتاج لفة "مع عقارب الساعة" (تقدر تعكس لو لازم)
    if (output > 0.0f) {
      // مثال: لتعريف اتجاه، نفرض:
      // positive -> right motor backwards, left motor forwards  (لف مع عقارب الساعة)
      rightMotorBackward((int)pwmCmd);
      leftMotorForward((int)pwmCmd);
    } else {
      // negative -> لف عكس عقارب الساعة
      rightMotorForward((int)pwmCmd);
      leftMotorBackward((int)pwmCmd);
    }

    // طباعة ديباغ خفيفة عالبلوتوث (تقدر تطفيها بعد التجربة)
   /* SerialBT.print("err="); SerialBT.print(error, 2);
    SerialBT.print(" angle="); SerialBT.print(currentAngle, 2);
    SerialBT.print(" gz="); SerialBT.print(gz_dps, 2);
    SerialBT.print(" out="); SerialBT.println(output, 2);*/

    // شرط النهاية: لما نكون وصلنا تقريباً (tolerance)
    if (fabs(error) < 2.09) { // لو الخطأ اقل من 1 درجة نعتبر انتهينا
      stopMotors();
     // SerialBT.println("PID rotate DONE");
       return true; 
    }

    // حلقات صغيرة جداً عشان I2C مايتحمّش
    delay(5);
  }
  stopMotors();
  ledcWrite(0,0);
ledcWrite(1,0);
ledcWrite(2,0);
ledcWrite(3,0);
delay(10);
return false; 
}

void turnLeft(){
  rotateByAnglePID(90,255,7000);
}

void turnRight(){
  rotateByAnglePID(-90,255,7000);
}

//////////////////////////PID///////////////////////////////////////

///////////////////////////motion_correct///////////////////////////
/*void moveStraightPIDBT() {
    // استقبل الأمر من البلوتوث: "distance,pwm,Kp"
    if (SerialBT.available()) {
        String data = SerialBT.readStringUntil('\n');
        data.trim();
        int firstComma = data.indexOf(',');
        int secondComma = data.indexOf(',', firstComma + 1);

        if (firstComma == -1 || secondComma == -1) return; // تأكد من صحة البيانات

        float distance_cm = data.substring(0, firstComma).toFloat();
        int targetPWM = data.substring(firstComma + 1, secondComma).toInt();
        float Kp_correction = data.substring(secondComma + 1).toFloat();

        SerialBT.print("Moving straight: ");
        SerialBT.print(distance_cm); SerialBT.print("cm, PWM="); SerialBT.print(targetPWM);
        SerialBT.print(", Kp_correction="); SerialBT.println(Kp_correction);

        float wheelCircumference = 12.566; // مثال قطر العجلة
        float revs = distance_cm / wheelCircumference;
        long targetCounts = (long)(revs * 8400);

        int16_t accelGyro[6] = {0};
        float initialAngleZ = 0;
        float currentAngleZ = 0;

        rightEncoder.clearCount();
        leftEncoder.clearCount();

        // زاوية البداية
        if (bmi160.getAccelGyroData(accelGyro) == 0) {
            initialAngleZ = accelGyro[2] / 16.4; // deg/s
        }

        unsigned long prevTime = millis();

        while (true) {
            long rightCount = rightEncoder.getCount();
            long leftCount = leftEncoder.getCount();
            long avgCount = (rightCount + leftCount) / 2;

            if (avgCount >= targetCounts) break;

            unsigned long currTime = millis();
            float dt = (currTime - prevTime) / 1000.0;
            prevTime = currTime;

            if (bmi160.getAccelGyroData(accelGyro) == 0) {
                float gz = accelGyro[2] / 16.4; // deg/s
                currentAngleZ += gz * dt;
            }

            float error = currentAngleZ - initialAngleZ;

            int correction = (int)(Kp_correction * error);

            int rightPWM, leftPWM;
            if (error > 0) {
                rightPWM = constrain(targetPWM - correction, 0, 255);
                leftPWM  = constrain(targetPWM + correction, 0, 255);
            } else {
                rightPWM = constrain(targetPWM + correction, 0, 255);
                leftPWM  = constrain(targetPWM - correction, 0, 255);
            }

            rightMotorForward(rightPWM);
            leftMotorForward(leftPWM);

            delay(5);
        }

        stopMotors();
        SerialBT.println("Done moving straight");
    }
}*/

///////////////////////////////////////////////////////////////////

void moveStraightPID(int basePWM, float distance_cm) {

    float wheelCircumference = 12.566; // قطر العجلة * π
    float revs = distance_cm / wheelCircumference;
    long targetCounts = (long)(revs * 8400);

    rightEncoder.clearCount();
    leftEncoder.clearCount();

    int16_t accelGyro[6] = {0};
    float initialAngleZ = 0;
    float currentAngleZ = 0;
    float Kp_correction = 2.0; 

    if (bmi160.getAccelGyroData(accelGyro) == 0) {
        initialAngleZ = accelGyro[2] / 16.4;
    }

    unsigned long prevTime = millis();

    while (true) {
        long rightCount = rightEncoder.getCount();
        long leftCount = leftEncoder.getCount();
        long avgCount = (rightCount + leftCount) / 2;
        if (avgCount >= targetCounts) break;

        unsigned long currTime = millis();
        float dt = (currTime - prevTime) / 1000.0;
        prevTime = currTime;

        if (bmi160.getAccelGyroData(accelGyro) == 0) {
            float gz = accelGyro[2] / 16.4; 
            currentAngleZ += gz * dt;
        }

        float error = currentAngleZ - initialAngleZ;
        int correction = (int)(Kp_correction * error);

        // اقفل correction في حدود PWM معقول
        correction = constrain(correction, -basePWM, basePWM);

        int rightPWM = basePWM - correction;
        int leftPWM  = basePWM + correction;

        // تأكد ان PWM داخل 0-255
        rightPWM = constrain(rightPWM, 0, 255);
        leftPWM  = constrain(leftPWM, 0, 255);

        rightMotorForward(rightPWM);
        leftMotorForward(leftPWM);

        delay(5);
    }

    // توقف نهائي للموتور للتخلص من أي اهتزاز
    stopMotors();
    delay(50);
    stopMotors();
}




void moveForward(  ) {
  float wheelCircumference = 12.566; 
  float revs = 24.0 / wheelCircumference;

 
  long targetCounts = (long)(revs * 8400); 

  rightEncoder.clearCount();
  leftEncoder.clearCount();

  rightMotorForward(255);
  leftMotorForward(255);

  while (true) {
    long right = rightEncoder.getCount();
    long left = -leftEncoder.getCount();
    long avgCount = (right + left) / 2;

    if (avgCount >= targetCounts) {
      break;
    }
    delay(1);
  }

  stopMotors();
}