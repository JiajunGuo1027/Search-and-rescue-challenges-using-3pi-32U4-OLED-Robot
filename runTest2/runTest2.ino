#include "Encoders.h"
#include "LineSensors.h"
#include "Motors.h"
#include "PID.h"
#include "Magnetometer.h"

#define BUZZER_PIN 6
#define EXPLORATION_TIME 120000 // 2 minutes in milliseconds
#define THRESHOLD 0.7  // Calibration threshold for detecting black line (between 0.0 and 1.0)
#define CALIBRATION_TIME 4000  // 校准阶段持续4秒
#define SNAKE_PERIOD 1000  // 蛇形运动左右切换的周期，单位：毫秒
#define DETECT_THRESHOLD 120  // 磁场检测阈值

Motors_c motors;
PID_c pid_left, pid_right;
LineSensors_c line_sensors;
Magnetometer_c magnetometer;
unsigned long startTime;

// 磁力计校准数据
float magMin[3] = {9999.9, 9999.9, 9999.9};
float magMax[3] = {-9999.9, -9999.9, -9999.9};
float magOffset[3];
float magScaling[3];

// 计算磁场强度的函数
float calculateMagneticFieldStrength() {
    float x_c = (magnetometer.readings[0] - magOffset[0]) * magScaling[0];
    float y_c = (magnetometer.readings[1] - magOffset[1]) * magScaling[1];
    float z_c = (magnetometer.readings[2] - magOffset[2]) * magScaling[2];
    return sqrt(x_c * x_c + y_c * y_c + z_c * z_c);  // 计算磁场强度
}

// 校准函数：小车原地旋转，记录每个传感器的最小和最大值，校准时间为3秒
void calibrateLineSensors() {
    unsigned long calibrationStartTime = millis();  // 记录校准开始的时间

    // 初始化最小和最大值
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        line_sensors.minimum[sensor] = 1023;
        line_sensors.maximum[sensor] = 0;
    }

    // 旋转小车，并记录每个传感器的最大值和最小值，持续3秒
    motors.setPWM(25, -25);  // 以25的速度旋转
    while (millis() - calibrationStartTime < CALIBRATION_TIME) {
        line_sensors.readSensorsADC();  // 获取传感器读数
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            // 更新最小值
            if (line_sensors.readings[sensor] < line_sensors.minimum[sensor]) {
                line_sensors.minimum[sensor] = line_sensors.readings[sensor];
            }
            // 更新最大值
            if (line_sensors.readings[sensor] > line_sensors.maximum[sensor]) {
                line_sensors.maximum[sensor] = line_sensors.readings[sensor];
            }
        }
        delay(10);  // 确保平稳读取
    }
    motors.setPWM(0, 0);  // 停止旋转

    // 校准结束后发出提示音
    analogWrite(BUZZER_PIN, 120);  // 打开蜂鸣器
    delay(500);  // 提示音持续500毫秒
    analogWrite(BUZZER_PIN, 0);  // 关闭蜂鸣器
}

// 磁力计校准函数：记录每个轴的最小值和最大值
void calibrateMagnetometer() {
    unsigned long calibrationStartTime = millis();  // 记录校准开始时间

    // 初始化最小和最大值
    for (int i = 0; i < 3; i++) {
        magMin[i] = 9999.9;
        magMax[i] = -9999.9;
    }

    // 旋转小车，并记录磁力计的最大值和最小值，持续3秒
    motors.setPWM(25, -25);  // 以25的速度旋转
    while (millis() - calibrationStartTime < CALIBRATION_TIME) {
        magnetometer.getReadings();  // 获取磁力计读数
        for (int i = 0; i < 3; i++) {
            // 更新最小值
            if (magnetometer.readings[i] < magMin[i]) {
                magMin[i] = magnetometer.readings[i];
            }
            // 更新最大值
            if (magnetometer.readings[i] > magMax[i]) {
                magMax[i] = magnetometer.readings[i];
            }
        }
        delay(10);  // 确保平稳读取
    }

    motors.setPWM(0, 0);  // 停止旋转

    // 计算偏移量和缩放因子
    for (int i = 0; i < 3; i++) {
        magOffset[i] = (magMin[i] + magMax[i]) / 2.0;
        magScaling[i] = 2.0 / (magMax[i] - magMin[i]);  // 归一化至[-1, 1]
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
        motors.setPWM(18, 10);  // 向右转
    } else {
        motors.setPWM(10, 18);  // 向左转
    }
}

// 避障探索函数
void exploreGrid() {
    startTime = millis();
    motors.setPWM(18, 18);
    delay(4000);
    while (millis() - startTime < EXPLORATION_TIME) {
        // 检查传感器是否检测到黑线
        if (isOnLine()) {
            // 检测到黑线，进行转弯避障
            motors.setPWM(0, 0);
            delay(100);
            motors.setPWM(-18, -18);
            delay(100);
            motors.setPWM(18, -18);  // 原地转弯避开黑线
            delay(500);  // 转弯500毫秒，时间可以根据需要调整
        } else {
            // 检查磁场强度，判断是否检测到磁铁
            magnetometer.getReadings();  // 获取最新的磁力计读数
            float magneticFieldStrength = calculateMagneticFieldStrength();
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
    motors.setPWM(-18, -18);  // 后退回到起始位置
    delay(5000);  // 移动5秒后停止
    motors.setPWM(0, 0);  // 停止
}

void setup() {
    // 等待2秒以便你可以调整小车位置
    delay(2000);
    // 设置蜂鸣器引脚
    pinMode(BUZZER_PIN, OUTPUT);
    
    // 初始化组件
    motors.initialise();  // 初始化电机
    line_sensors.initialiseForADC();  // 初始化线形传感器
    pid_left.initialise(1.0, 0.5, 0.1);  // 初始化PID
    pid_right.initialise(1.0, 0.5, 0.1);  // 初始化PID
    magnetometer.initialise();  // 初始化磁力计

    // 校准线传感器
    calibrateLineSensors();
    // 校准磁力计
    calibrateMagnetometer();
}

void loop() {
    exploreGrid();
    // 停止探索
    motors.setPWM(0, 0);
    while (1);  // 停止程序
}
