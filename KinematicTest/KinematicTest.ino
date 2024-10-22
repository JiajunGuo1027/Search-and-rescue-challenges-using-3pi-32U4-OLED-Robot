#include "Encoders.h"   // 自动编码器计数
#include "Kinematics.h" // 包含运动学逻辑

// 创建运动学类的全局实例
Kinematics_c pose;

void setup() {
  // 初始化串口通信，便于校准时查看位置和角度
  Serial.begin(9600);

  // 初始化编码器以记录轮子转动的脉冲
  setupEncoder0();
  setupEncoder1();

  // 设置机器人初始位置和角度为 (0,0,0)
  pose.initialise(0, 0, 0);
}

void loop() {
  // 定期更新机器人的位置和角度（每次调用更新编码器值）
  pose.update();

  // 输出当前的 X, Y 坐标和角度 Theta
  Serial.print("X: ");
  Serial.print(pose.x, 4); // 输出精确到小数点后4位
  Serial.print(" Y: ");
  Serial.print(pose.y, 4);
  Serial.print(" Theta: ");
  Serial.println(pose.theta, 4);

  // 每20毫秒更新一次，避免占用过多CPU资源
  delay(20);
}
