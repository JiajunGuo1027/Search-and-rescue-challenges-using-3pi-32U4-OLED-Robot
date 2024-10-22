# include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
# define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

#include "Encoders.h"
#include "LineSensors.h"
#include "Motors.h"
#include "PID.h"
#include "Magnetometer.h"
#include <math.h>
#include <Wire.h>


#define BUZZER_PIN 6
#define EXPLORATION_TIME 120000 // 2 minutes in milliseconds
#define THRESHOLD 0.7  // Calibration threshold for detecting black line (between 0.0 and 1.0)
#define CALIBRATION_TIME 4000  // 校准阶段持续4秒
#define SNAKE_PERIOD 500  // 蛇形运动左右切换的周期，单位：毫秒
#define DETECT_THRESHOLD 1000  // 磁场检测阈值

Motors_c motors;
PID_c pid_left, pid_right;
LineSensors_c line_sensors;
Magnetometer_c my_mag;
unsigned long startTime;

// 磁力计校准数据
float magMin[3] = {9999.9, 9999.9, 9999.9};
float magMax[3] = {-9999.9, -9999.9, -9999.9};
float magOffset[3];
float magScaling[3];
//
//// 计算磁场强度的函数
//float calculateMagneticFieldStrength() {
//    float x_c = (my_mag.readings[0] - magOffset[0]) * magScaling[0];
//    float y_c = (my_mag.readings[1] - magOffset[1]) * magScaling[1];
//    float z_c = (my_mag.readings[2] - magOffset[2]) * magScaling[2];
//    return sqrt(x_c * x_c + y_c * y_c + z_c * z_c);  // 计算磁场强度
//}

// 计算磁场强度的函数
float calculateFieldStrength(float x, float y, float z) {
    float validX = !isnan(x) ? x : 0;  // 如果x为nan，则将其设为0
    float validY = !isnan(y) ? y : 0;  // 如果y为nan，则将其设为0
    float validZ = !isnan(z) ? z : 0;  // 如果z为nan，则将其设为0

    float fieldStrength = sqrt(validX * validX + validY * validY + validZ * validZ);
    if( SERIAL_ACTIVE ) {

  Serial.print("Field Strength: ");
    Serial.println(fieldStrength);
}
    
    return fieldStrength;
}

// 同时校准线性传感器和磁力计，并通过串口输出校准信息
void calibrateSensors() {
    unsigned long calibrationStartTime = millis();  // 记录校准开始时间

  Serial.println("开始校准...");

    // 初始化线性传感器和磁力计的最小和最大值
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        line_sensors.minimum[sensor] = 1023;
        line_sensors.maximum[sensor] = 0;
    }
    for (int i = 0; i < 3; i++) {
        magMin[i] = 9999.9;
        magMax[i] = -9999.9;
    }

    // 同时旋转小车，进行线性传感器和磁力计的校准
    motors.setPWM(24, -25);  // 以25的速度旋转
    while (millis() - calibrationStartTime < CALIBRATION_TIME) {
        // 读取线性传感器
        line_sensors.readSensorsADC();
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            // 更新线性传感器的最小和最大值
            if (line_sensors.readings[sensor] < line_sensors.minimum[sensor]) {
                line_sensors.minimum[sensor] = line_sensors.readings[sensor];
            }
            if (line_sensors.readings[sensor] > line_sensors.maximum[sensor]) {
                line_sensors.maximum[sensor] = line_sensors.readings[sensor];
            }
        }

        // 读取磁力计
        my_mag.getReadings();
        for (int i = 0; i < 3; i++) {
            // 更新磁力计的最小和最大值
            if (my_mag.readings[i] < magMin[i]) {
                magMin[i] = my_mag.readings[i];
            }
            if (my_mag.readings[i] > magMax[i]) {
                magMax[i] = my_mag.readings[i];
            }
        }
        delay(10);  // 确保稳定读取
    }

    motors.setPWM(0, 0);  // 停止旋转

    // 计算磁力计的校准偏移量和缩放因子，并通过串口输出校准结果


  Serial.println("磁力计校准结束，结果如下：");

   
    
    for (int i = 0; i < 3; i++) {
        magOffset[i] = (magMin[i] + magMax[i]) / 2.0;
        magScaling[i] = 1000.0 / (magMax[i] - magMin[i]);  // 归一化至[-1, 1]

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


// 检查传感器是否检测到黑线
bool isOnLine() {
    line_sensors.calcCalibratedADC();  // 获取校准后的传感器读数
    // 如果任何一个传感器的校准值低于阈值（表示检测到黑线），返回true
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        if (line_sensors.calibrated[sensor] > THRESHOLD) {
            return true;
        }
    }
    return false;
}

// 蛇形运动函数
void snakeMove() {
    unsigned long currentTime = millis();
    // 根据当前时间决定是向左转还是向右转
    if ((currentTime / SNAKE_PERIOD) % 2 == 0) {
        motors.setPWM(16, 8);  // 向右转
    } else {
        motors.setPWM(8, 16);  // 向左转
    }
}


// 在探索过程中，通过串口输出磁力计的读数和磁场强度
void exploreGrid() {
    startTime = millis();
    motors.setPWM(17, 17);
    delay(3500);

    while (millis() - startTime < EXPLORATION_TIME) {
        // 检查传感器是否检测到黑线
        if (isOnLine()) {
            // 检测到黑线，进行转弯避障 
            motors.setPWM(0, 0);
            delay(100);
            motors.setPWM(-18, -18);
            delay(100);
            motors.setPWM(16, -16);  // 原地转弯避开黑线
            delay(500);  // 转弯500毫秒，时间可以根据需要调整
        } else {
            // 读取磁力计并输出归一化读数和磁场强度
            my_mag.getReadings();
//            float magneticFieldStrength = calculateMagneticFieldStrength();

            // 使用新的 calculateFieldStrength 计算磁场强度
            float magneticFieldStrength = calculateFieldStrength(
                (my_mag.readings[0] - magOffset[0]) * magScaling[0],
                (my_mag.readings[1] - magOffset[1]) * magScaling[1],
                (my_mag.readings[2] - magOffset[2]) * magScaling[2]
            );
            
             // 输出磁力计实时读数和计算的磁场强度
             if( SERIAL_ACTIVE ) {

            Serial.print("Magnetometer readings (normalized): X=");
            Serial.print((my_mag.readings[0] - magOffset[0]) * magScaling[0]);
            Serial.print(", Y=");
            Serial.print((my_mag.readings[1] - magOffset[1]) * magScaling[1]);
            Serial.print(", Z=");
            Serial.print((my_mag.readings[2] - magOffset[2]) * magScaling[2]);
            Serial.print(", Field Strength=");
            Serial.println(magneticFieldStrength);
}
            

           // 检测磁场强度超过阈值，检测到磁铁
            if (magneticFieldStrength > DETECT_THRESHOLD) {
                // 检测到磁铁，停止小车并发出蜂鸣声
                motors.setPWM(0, 0);
                analogWrite(BUZZER_PIN, 120);  // 打开蜂鸣器
                delay(1000);  // 蜂鸣器提示1秒
                analogWrite(BUZZER_PIN, 0);  // 关闭蜂鸣器
                return;  // 退出探索函数
            }
            // 没有检测到磁铁，继续蛇形运动
            snakeMove();
        }
        delay(1000);  // 调整延迟以确保平稳移动
    }

    // 探索结束后返回起始位置
    motors.setPWM(-17, -17);  // 后退回到起始位置
    delay(5000);  // 移动5秒后停止
    motors.setPWM(0, 0);  // 停止
}

void setup() {
    // 串口监视器用于调试输出
    Serial.begin(9600);
    if( SERIAL_ACTIVE ) {

 Serial.println("***RESET***");
}
    
    delay(1000);
    
    // 等待2秒以便你可以调整小车位置
    delay(2000);
    
    // 设置蜂鸣器引脚
    pinMode(BUZZER_PIN, OUTPUT);
    
    // 初始化组件
    motors.initialise();  // 初始化电机
    line_sensors.initialiseForADC();  // 初始化线形传感器
    pid_left.initialise(0.9, 0.4, 0.1);  // 初始化PID
    pid_right.initialise(1.2, 0.6, 0.1);  // 初始化PID
    my_mag.initialise();

    // 同时校准线性传感器和磁力计
    calibrateSensors();
}


void loop() {
  // 从 Magnetometer_c 实例中获取最新的磁力计读数
    my_mag.getReadings();

    // 打印磁力计的 X, Y, Z 轴读数
    if( SERIAL_ACTIVE ) {

 Serial.print(my_mag.readings[0]);
    Serial.print(",");
    Serial.print(my_mag.readings[1]);
    Serial.print(",");
    Serial.println(my_mag.readings[2]);
}

    // 适当的延迟，保持数据读取速度可以观察
    delay(500);
    exploreGrid();
    // 停止探索
    motors.setPWM(0, 0);
    while (1);  // 停止程序
}
