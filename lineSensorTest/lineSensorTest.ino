#include "LineSensors.h"
#include "Motors.h"

LineSensors_c line_sensors;
Motors_c motors;

bool calibrating = true;

void setup() {
    // 初始化串口通信
    Serial.begin(9600);
    
    // 初始化线传感器
    line_sensors.initialiseForADC();

    // 初始化电机
     motors.initialise();

    // 设置初始的最大最小值
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        line_sensors.maximum[sensor] = 0;  // 初始最大值
        line_sensors.minimum[sensor] = 1023;  // 初始最小值
    }

    // 让小车开始旋转
    motors.setPWM(15, -15); // 设置电机反向旋转
}

void loop() {
    if (calibrating) {
        // 读取传感器的原始值
        line_sensors.readSensorsADC();

        // 更新每个传感器的最大值和最小值
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            if (line_sensors.readings[sensor] > line_sensors.maximum[sensor]) {
                line_sensors.maximum[sensor] = line_sensors.readings[sensor];  // 更新最大值
            }
            if (line_sensors.readings[sensor] < line_sensors.minimum[sensor]) {
                line_sensors.minimum[sensor] = line_sensors.readings[sensor];  // 更新最小值
            }
        }

        // 让机器人旋转一段时间后停止校准
        delay(10);  // 给每次读取之间留出间隔
        if (millis() > 5000) {  // 校准5秒
            motors.setPWM(0, 0); // 停止电机旋转
            calibrating = false;
            Serial.println("Calibration completed!");

            // 输出校准后的最大值和最小值
            for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
                Serial.print("Sensor ");
                Serial.print(sensor);
                Serial.print(" Min: ");
                Serial.print(line_sensors.minimum[sensor]);
                Serial.print(" Max: ");
                Serial.println(line_sensors.maximum[sensor]);
            }
        }
    } else {
        // 校准完成后使用校准值读取传感器数据
        line_sensors.calcCalibratedADC();
        
        // 打印校准后的数据
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            Serial.print("Calibrated sensor ");
            Serial.print(sensor);
            Serial.print(": ");
            Serial.println(line_sensors.calibrated[sensor], 4);  // 输出四位小数
        }

        delay(500);  // 每500毫秒输出一次
    }
}
