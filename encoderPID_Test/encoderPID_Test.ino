#include "Encoders.h"
#include "PID.h"
#include "Motors.h"

// Create instances
Motors_c motors;
PID_c pid_left;
PID_c pid_right;

// Target speed for the robot (40 as an example)
const float target_speed = 40;

// Last encoder counts
long last_left_count = 0;
long last_right_count = 0;

void setup() {
    // Initialize motors and encoders
    motors.initialise();
    setupEncoder0();
    setupEncoder1();

    // Initialize PID for both motors with tuned gains
    pid_left.initialise(1.0, 0.5, 0.1);  // 左电机的 PID 参数
    pid_right.initialise(1.0, 0.5, 0.1);  // 右电机的 PID 参数
      pid_left.reset();
  pid_right.reset();

    // Start the Serial monitor for debugging
    Serial.begin(9600);
}

void loop() {
    static unsigned long last_update_time = 0;
    unsigned long current_time = millis();

    // Ensure PID update happens at a regular interval (e.g., 100 ms)
    if (current_time - last_update_time >= 100) {
        last_update_time = current_time;

        // 使用 PID 控制器来调节速度
        forward();
    }

    stop();
    delay(4000); // 停止 4 秒

    // Control the robot movements
    forward();
    delay(9000); // 前进 9 秒
    stop();
    delay(1000); // 停止 1 秒
}

// 修改后的 forward() 函数，通过 PID 控制来调节速度
void forward() {
    // Get the encoder counts for both motors
    long left_count_delta = count_e0 - last_left_count;
    long right_count_delta = count_e1 - last_right_count;

    last_left_count = count_e0;
    last_right_count = count_e1;

    // 使用 PID 控制器根据编码器反馈计算电机的速度
    float left_speed = pid_left.update(target_speed, left_count_delta);
    float right_speed = pid_right.update(target_speed, right_count_delta);

    // 将计算得到的速度应用到电机
    motors.setPWM(left_speed, right_speed);

    // 打印调试信息，显示实际速度和目标速度
    Serial.print("Left speed: ");
    Serial.print(left_speed);
    Serial.print(", Right speed: ");
    Serial.println(right_speed);
}

// Function to move backward (not using PID control in this example)
void backward() {
    motors.setPWM(-15.0, -15.0); // Move both motors backward
}

// Function to rotate in place (not using PID control in this example)
void rotate() {
    motors.setPWM(15.0, -15.0); // One motor forward, the other backward
}

// Function to stop the robot
void stop() {
    motors.setPWM(0.0, 0.0); // Stop both motors
}
