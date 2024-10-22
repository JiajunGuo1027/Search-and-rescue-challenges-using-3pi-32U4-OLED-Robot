// 使用 Magnetometer.h 文件中的 Magnetometer_c 类
#include "Magnetometer.h"

// 创建 Magnetometer_c 类的实例
Magnetometer_c my_mag;

void setup() {
    // 串口监视器用于调试输出
    Serial.begin(9600);
    Serial.println("***RESET***");
    delay(1000);
    my_mag.initialise();

}

void loop() {
    // 从 Magnetometer_c 实例中获取最新的磁力计读数
    my_mag.getReadings();

    // 打印磁力计的 X, Y, Z 轴读数
    Serial.print(my_mag.readings[0]);
    Serial.print(",");
    Serial.print(my_mag.readings[1]);
    Serial.print(",");
    Serial.print(my_mag.readings[2]);

    // 打印新行
    Serial.print("\n");

    // 适当的延迟，保持数据读取速度可以观察
    delay(100);
}
