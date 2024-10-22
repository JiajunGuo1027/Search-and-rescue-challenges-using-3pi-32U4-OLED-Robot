#include "Kinematics.h"   // Include the kinematics code
#include "Motors.h"       // Include motor control
#include "Encoders.h"     // Include encoder code

Motors_c motors;          // Motor control instance
Kinematics_c pose;        // Position tracking instance

// Setup function for motor and encoder initialization
void setup() {
    Serial.begin(9600);   // Start serial communication
    motors.initialise();  // Initialize motors
    setupEncoder0();      // Initialize encoder for motor 0
    setupEncoder1();      // Initialize encoder for motor 1
    
    pose.initialise(0, 0, 0);   // Initialize kinematics (start at X = 0, Y = 0, θ = 0)
    
    delay(1000);                // Delay to allow everything to initialize
}
void poseUpdate() {
    // 更新小车的位置信息和角度
    pose.update();
    
    // 归一化角度
    normalizeTheta();
}
void normalizeTheta() {
    while (pose.theta > PI) {
        pose.theta -= 2 * PI;
    }
    while (pose.theta < -PI) {
        pose.theta += 2 * PI;
    }
}
// This function allows you to update the kinematics and output the X, Y, θ
void updatePosition() {
     poseUpdate();  // Update robot pose based on encoder feedback

    // Print out the current position and orientation
    Serial.print("X: ");
    Serial.print(pose.x);
    Serial.print(" mm, Y: ");
    Serial.print(pose.y);
    Serial.print(" mm, Theta: ");
    Serial.println(pose.theta);
}

// Loop function for continuous updating
void loop() {
    updatePosition();  // Update and print the current position
    delay(500);        // Update every 500 ms (adjust as needed)
}
