#include "Motors.h"
#include "Magnetometer.h"  
#include <math.h>
#include "Magnetometer.h"
#include <Wire.h>


#define NUM_SENSORS 3
#define CALIBRATION_TIME 4000 
#define BUZZER_PIN 6
Magnetometer_c magnetometer;
Motors_c motors;
LIS3MDL mag;

float magMin[NUM_SENSORS], magMax[NUM_SENSORS], magOffset[NUM_SENSORS], magScaling[NUM_SENSORS];

// Calculate the magnetic field strength as a function
float calculateMagneticFieldStrength() {
    float x_c = (magnetometer.readings[0] - magOffset[0]) * magScaling[0];
    float y_c = (magnetometer.readings[1] - magOffset[1]) * magScaling[1];
    float z_c = (magnetometer.readings[2] - magOffset[2]) * magScaling[2];
    return sqrt(x_c * x_c + y_c * y_c + z_c * z_c);  // Calculated magnetic field strength
}

// Calibrate the magnetometer and verify minimum and maximum values
void calibrateMagnetometer() {
    unsigned long calibrationStartTime = millis();  // 记录校准开始时间
    Serial.println("开始校准...");

    // 初始化磁力计的最小和最大值
    for (int i = 0; i < 3; i++) {
        magMin[i] = 9999.9;
        magMax[i] = -9999.9;
    }

    // 旋转小车并进行磁力计校准
    motors.setPWM(20, -21);  // 让小车旋转
    while (millis() - calibrationStartTime < CALIBRATION_TIME) {
        // 读取磁力计
        magnetometer.getReadings();
        for (int i = 0; i < 3; i++) {
            // 更新磁力计的最小和最大值
            if (magnetometer.readings[i] < magMin[i]) {
                magMin[i] = magnetometer.readings[i];
                Serial.print("Axis ");
                Serial.print(i);
                Serial.print(" new min: ");
                Serial.println(magMin[i]);
            }
            if (magnetometer.readings[i] > magMax[i]) {
                magMax[i] = magnetometer.readings[i];
                Serial.print("Axis ");
                Serial.print(i);
                Serial.print(" new max: ");
                Serial.println(magMax[i]);
            }
        }
        delay(10);  // 确保稳定读取
    }

    motors.setPWM(0, 0);  // 停止旋转

    // 校准结束，验证最小值和最大值的合理性
    Serial.println("磁力计校准结束，结果如下：");
    for (int i = 0; i < 3; i++) {
        float range = magMax[i] - magMin[i];
        
        // 检查范围是否合理
        if (range < 0.5) {  // 自定义阈值，确保范围不太小
            Serial.print("Warning: Axis ");
            Serial.print(i);
            Serial.println(" range is too small, recalibration may be required.");
        }

        // 计算偏移量和缩放因子
        magOffset[i] = (magMin[i] + magMax[i]) / 2.0;
        magScaling[i] = 1000.0 / range;  // 归一化到 [-1, 1]

        // 检查缩放因子是否合理
        if (magScaling[i] > 10 || magScaling[i] < 0.1) {  // 合理范围可以根据需求调整
            Serial.print("Warning: Axis ");
            Serial.print(i);
            Serial.println(" scaling factor is abnormal.");
        }

        // 输出每个轴的校准结果
        Serial.print("Axis ");
        Serial.print(i);
        Serial.print(": min=");
        Serial.print(magMin[i]);
        Serial.print(", max=");
        Serial.print(magMax[i]);
        Serial.print(", offset=");
        Serial.print(magOffset[i]);
        Serial.print(", scaling=");
        Serial.println(magScaling[i]);
    }

    // 校准结束后发出提示音
    analogWrite(BUZZER_PIN, 120);  // 打开蜂鸣器
    delay(500);  // 提示音持续500毫秒
    analogWrite(BUZZER_PIN, 0);  // 关闭蜂鸣器
}

void setup() {

     Wire.begin();

  // Serial for debug output
  Serial.begin(9600);
  Serial.println("***RESET***");
  delay(1000);


  // If you have a problem with your magnetometer, your code
  // will get stuck here and print the below message.
  if (!mag.init() ) {  // no..? :(

    // Since we failed to communicate with the
    // magnetometer, we put the robot into an infinite
    // while loop and report the error.
    while(1) {
      Serial.println("Failed to detect and initialize magnetometer!");
      delay(1000);
    }
  }

  // Initialisation was ok.  Set magnetometer to default settings.
  mag.enableDefault();

    // 初始化电机和磁力计
    motors.initialise();  // 假设电机初始化

    // 校准磁力计
    calibrateMagnetometer();
}

void loop() {
   
    // Calling this function from our instance of the LIS3MDL class
  // gets the latest readings from the magnetometer sensor.
  mag.read();

  // Print the readings over serial. m.x = x axis, etc.
  Serial.print(mag.m.x);
  Serial.print(",");
  Serial.print(mag.m.y);
  Serial.print(",");
  Serial.print(mag.m.z);

  // Remember to finish comma seperated values with a newline (\n)
  Serial.print("\n");

  // Short delay to keep things slow enough to observe on the
  // Serial Plotter.
  // There is a limit to how fast you
  // can make i2c readings.
  delay(100);
}
