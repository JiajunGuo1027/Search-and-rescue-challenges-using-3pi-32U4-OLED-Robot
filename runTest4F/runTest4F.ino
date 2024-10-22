#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
# define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)
#include "Encoders.h"
#include "LineSensors.h"
#include "Motors.h"
#include "PID.h"
#include "Magnetometer.h"
#include "Kinematics.h"
#include <math.h>
#include <Wire.h>

#define BUZZER_PIN 6
#define NUM_SENSORS 3
#define CALIBRATION_TIME 3200  // Calibration phase
#define EXPLORATION_TIME 120000  // 2 minutes in milliseconds
#define THRESHOLD 0.6  // Calibration threshold for detecting black line (between 0.0 and 1.0)
#define SNAKE_PERIOD 1000  // The period of the serpentine movement between left and right, in milliseconds
#define DETECT_THRESHOLD 480  // Magnetic field detection threshold
#define EXPLORATION_LIMIT_Y -190  // Maximum allowed y coordinate in mm
#define EXPLORATION_LIMIT_X 600  // The maximum allowed x coordinate, in mm

Motors_c motors;
PID_c pid_left, pid_right,pi_turn;
LineSensors_c line_sensors;
Magnetometer_c my_mag;
Kinematics_c pose;
unsigned long startTime;

unsigned long test_ts;
# define TEST_MS 2000
// Last encoder counts
long last_left_count = 0;
long last_right_count = 0;
float targetSpeedLeft = 0;
float targetSpeedRight = 0;
float target_theta = 0;  // target ignition angle
float smoothed_speed_left = 0.0;
float smoothed_speed_right = 0.0;
const float alpha = 0.8;  // Smoothing factor
float lastOutOfBoundsX = 0;
float lastOutOfBoundsY = 0;
const float BOUNDARY_DISTANCE_THRESHOLD = 70; // 100mm distance from the last boundary check

// For magnetometer calibration
float magMin[3] = {9999.9, 9999.9, 9999.9};
float magMax[3] = {-9999.9, -9999.9, -9999.9};
float magOffset[3];
float magScaling[3];

// Recording magnet position
float magnet_x = 0;
float magnet_y = 0;

bool magnet_found = false;  // Used to signal if a magnet has been found

bool isOutOfBounds() {
    // Calculate the distance the robot has moved since it was last out of bounds
    float distanceFromLastBoundary = sqrt(pow(pose.x - lastOutOfBoundsX, 2) + pow(pose.y - lastOutOfBoundsY, 2));
    
    if (distanceFromLastBoundary < BOUNDARY_DISTANCE_THRESHOLD) {
        // Still within the safe distance, don't check for boundaries
        return false;
    }

    // Normal boundary checking
    if (pose.x > EXPLORATION_LIMIT_X || pose.y > EXPLORATION_LIMIT_Y) {
        // Store the current position as the last detected boundary position
        lastOutOfBoundsX = pose.x;
        lastOutOfBoundsY = pose.y;
        return true;  // Out of bounds
    }
    return false;  // Within bounds
}

// Check whether the sensor detects a black line
bool isOnLine() {
    line_sensors.calcCalibratedADC();  // Get calibrated sensor readings
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        if (line_sensors.calibrated[sensor] > THRESHOLD) {
            return true;
        }
    }
    return false;
}

// Function to apply low-pass filter
float applyLowPassFilter(float current_speed, float previous_smoothed) {
    return (alpha * current_speed) + ((1.0 - alpha) * previous_smoothed);
}

// Calculate the magnetic field strength as a function
float calculateMagneticFieldStrength() {
    float x_c = (my_mag.readings[0] - magOffset[0]) * magScaling[0];
    float y_c = (my_mag.readings[1] - magOffset[1]) * magScaling[1];
    float z_c = (my_mag.readings[2] - magOffset[2]) * magScaling[2];
    return sqrt(x_c * x_c + y_c * y_c + z_c * z_c);  // Calculated magnetic field strength
}

// The Angle error is calculated and the rotation speed requirement is calculated through the PI controller
float calculateTurnSpeed(float target_angle) {
    float theta_error = target_angle - pose.theta;

    // Normalize the Angle difference, making sure the Angle difference is between [-PI, PI]
    if (theta_error > PI) {
        theta_error -= 2 * PI;
    } else if (theta_error < -PI) {
        theta_error += 2 * PI;
    }

    // The PI controller is used to calculate speed requirements
    float turn_speed = pi_turn.update(0, theta_error); 
    return turn_speed;
}

// Initialize the PID and PI controllers
void setupPID() {
    pid_left.initialise(0.5, 0.1, 0.1);
    pid_right.initialise(0.5, 0.1, 0.1);
    pi_turn.initialise(0.5, 0.1, 0.1);  //The PI controller is used for rotation
 //     test_ts = millis();

  pid_left.reset();
  pid_right.reset();
  pi_turn.reset();
}

void setMotorsSpeed(float targetSpeedLeft, float targetSpeedRight) {
//    if (millis() - test_ts > TEST_MS) {
//        test_ts = millis();
        
        // Get the encoder counts for both motors
        long left_count_delta = count_e0 - last_left_count;
        long right_count_delta = count_e1 - last_right_count;

        last_left_count = count_e0;
        last_right_count = count_e1;

        // Use the PID controller to calculate motor speeds based on encoder feedback
        float left_speed = pid_left.update(targetSpeedLeft, left_count_delta);
        float right_speed = pid_right.update(targetSpeedRight, right_count_delta);

        // Apply low-pass filter to smooth the speed measurements
        smoothed_speed_left = applyLowPassFilter(left_speed, smoothed_speed_left);
        smoothed_speed_right = applyLowPassFilter(right_speed, smoothed_speed_right);

        // Apply the smoothed speeds to the motors
        motors.setPWM(smoothed_speed_left, smoothed_speed_right);
//    }
}

// Calibrate the linear sensor and magnetometer simultaneously and output the calibration information through the serial port
void calibrateSensors() {
    unsigned long calibrationStartTime = millis();  // Record the start time of calibration

    // Initializes minimum and maximum values for linear sensors and magnetometers
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        line_sensors.minimum[sensor] = 1023;
        line_sensors.maximum[sensor] = 0;
    }
    for (int i = 0; i < 3; i++) {
        magMin[i] = 9999.9;
        magMax[i] = -9999.9;
    }

    motors.setPWM(-27, 27);
    while (millis() - calibrationStartTime < CALIBRATION_TIME) {
        // Read linear sensor
        line_sensors.readSensorsADC();
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            // Update minimum and maximum values of linear sensors
            if (line_sensors.readings[sensor] < line_sensors.minimum[sensor]) {
                line_sensors.minimum[sensor] = line_sensors.readings[sensor];
            }
            if (line_sensors.readings[sensor] > line_sensors.maximum[sensor]) {
                line_sensors.maximum[sensor] = line_sensors.readings[sensor];
            }
        }

        // Read the magnetometer
        my_mag.getReadings();
        
        for (int i = 0; i < 3; i++) {
            // Update the minimum and maximum values of the magnetometer
            if (my_mag.readings[i] < magMin[i]) {
                magMin[i] = my_mag.readings[i];
            }
            if (my_mag.readings[i] > magMax[i]) {
                magMax[i] = my_mag.readings[i];
            }
        }
    }

    motors.setPWM(0, 0); 

    for (int i = 0; i < 3; i++) {
        magOffset[i] = (magMin[i] + magMax[i]) / 2.0;
        if (magMax[i] <= magMin[i]) {
            magScaling[i] = 0.5;  
        } else {
            magScaling[i] = 800.0 / (magMax[i] - magMin[i]);  // Normalized to [-1, 1]
        }

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

    //A prompt tone is sounded after calibration is complete
    analogWrite(BUZZER_PIN, 120);  
    delay(500);  
    analogWrite(BUZZER_PIN, 0); 

     // After the calibration is complete, adjust the car orientation back to the original direction (theta = 0).
//    poseUpdate();  
//    setTurn(0); 
//    while (fabs(pose.theta) > 0.1) {  
//    setTurn(0); 
//    poseUpdate();  
//    delay(100);  
//}
    motors.setPWM(0, 0);           
    delay(500);                 
}

//Serpentine motion function
void snakeMove() {
    unsigned long currentTime = millis();
    if ((currentTime / SNAKE_PERIOD) % 2 == 0) {
        motors.setPWM(17, 11);  // turn right
                delay(800);
    } else {
        motors.setPWM(11, 17);  // turn left
                delay(800);
    }
}

// exploration stage
void exploreGrid() {
    startTime = millis();
    
    motors.setPWM(17, 17);
    delay(3900);
    unsigned long lastSensorReadTime = 0;

    while (millis() - startTime < EXPLORATION_TIME) {
        poseUpdate();

        if (isOnLine()) {
            if (SERIAL_ACTIVE) {
                Serial.println("Line detected");
            }
            motors.setPWM(0, 0);
            delay(100);
            motors.setPWM(-16, 16);
            delay(1500);
            motors.setPWM(15, 15);
            delay(700);
            continue;
        }
        
        if (isOutOfBounds()) {
            if (SERIAL_ACTIVE) {
                Serial.println("Out of bounds");
            }
//            motors.setPWM(-16, -16);
//            delay(500);
            motors.setPWM(0, 0);
            delay(100);
            motors.setPWM(-16, 16);
            delay(1900);
            motors.setPWM(15, 15);
            delay(1000);
            poseUpdate();
            continue;
        }
       
        if (millis() - lastSensorReadTime > 10) {
            lastSensorReadTime = millis();
            my_mag.getReadings();
            float magneticFieldStrength = calculateMagneticFieldStrength();
            
            if (magneticFieldStrength > DETECT_THRESHOLD) {
                motors.setPWM(0, 0);
                poseUpdate();
                magnet_x = pose.x;
                magnet_y = pose.y;
                magnet_found = true;  

                analogWrite(BUZZER_PIN, 120);
                delay(300);
                analogWrite(BUZZER_PIN, 0);
                pose.update();
                returnToStart();
                analogWrite(BUZZER_PIN, 120);
                delay(300);
                analogWrite(BUZZER_PIN, 0);
                pose.update();
                goToMagnet();
                analogWrite(BUZZER_PIN, 120);
                delay(300);
                analogWrite(BUZZER_PIN, 0);
                break;
            }
        snakeMove(); // No magnet detected. Continue serpentine motion
        }
    }
    analogWrite(BUZZER_PIN, 120);
    delay(1000);
    analogWrite(BUZZER_PIN, 0);
   // motors.setPWM(0, 0);
    pose.update();
    returnToStart();
}

void poseUpdate() {
    //Update car position information and Angle
    pose.update();
    
    // Normalized Angle
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

void setTurn(float target_angle) {
    poseUpdate();
    float turn_speed = calculateTurnSpeed(target_angle);
    float angle_difference = target_angle - pose.theta;

    //Make sure the Angle difference is between -PI and PI
    if (angle_difference > PI) {
        angle_difference -= 2 * PI;
    } else if (angle_difference < -PI) {
        angle_difference += 2 * PI;
    }

    // Introduce a dead zone to prevent constant minor adjustments
    if (fabs(angle_difference) > 0.05) {  // Adjusted from 0.05 to 0.1 radians
        // Turn right if the angle difference is positive
        if (angle_difference > 0.05) {
            motors.setPWM(-15, 15);
        }
        // Turn left if the angle difference is negative
        else if (angle_difference < -0.05) {
            motors.setPWM(15, -15);
        }
    } else {
        motors.setPWM(0, 0);  // Stop turning if within dead zone
    }
}

// Move to target
void moveToTarget(float target_x, float target_y) {
    poseUpdate();
    float distance = sqrt(pow(target_x - pose.x, 2) + pow(target_y - pose.y, 2));
    if (distance < 10) {
        motors.setPWM(0, 0);
        return;
    }

    float target_angle = atan2(target_y - pose.y, target_x - pose.x);
    setTurn(target_angle);
    motors.setPWM(17, 17);
}

void returnToStart() {
    float target_angle = atan2(0 - pose.y, 0 - pose.x);
    setTurn(target_angle);
    moveToTarget(0, 0);
}

void goToMagnet() {
    float target_angle = atan2(magnet_y - pose.y, magnet_x - pose.x);
    setTurn(target_angle);
    moveToTarget(magnet_x, magnet_y);
}

void setup() {
    Serial.begin(9600);
    delay(1000);
    pinMode(BUZZER_PIN, OUTPUT);

    motors.initialise();
    setupEncoder0();
    setupEncoder1();
    setupPID();  
    line_sensors.initialiseForADC();
    my_mag.initialise(); 
    smoothed_speed_left = 0.0;
    smoothed_speed_right = 0.0;
    pose.initialise(0, 0, 0); // Initialization start
    delay(1000);

//    if (!my_mag.initialise()) {
//        if (SERIAL_ACTIVE) {
//            Serial.println("Magnetometer initialization failed!");
//        }
//        while (1);
//    } else {
//        if (SERIAL_ACTIVE) {
//            Serial.println("Magnetometer initialized successfully.");
//        }
//    }    

    analogWrite(BUZZER_PIN, 120);
    delay(500);
    analogWrite(BUZZER_PIN, 0);

    calibrateSensors();
    analogWrite(BUZZER_PIN, 120);
    delay(500);
    analogWrite(BUZZER_PIN, 0);

//    //poseUpdate();  
//    setTurn(0);  
//    delay(500); 
//    analogWrite(BUZZER_PIN, 120);
//    delay(500);
//    analogWrite(BUZZER_PIN, 0);

    //motors.setPWM(0, 0);
    //float currentTheta = pose.theta; 
//    poseUpdate(); 
//    setTurn(-PI/2);             
//    delay(500);                    
}

void loop() { 
    poseUpdate();  // update location

    // Exploratory phase
    exploreGrid();
    
    // go back
    returnToStart();

    // If the magnet is found, return to the magnet position
    if (magnet_found) {
        goToMagnet();
    }

    // Whether the magnet is found or not, make a beep to signal the end
    analogWrite(BUZZER_PIN, 120);
    delay(1000);
    analogWrite(BUZZER_PIN, 0);

    motors.setPWM(0, 0);  // stop
    while (1); 
}
