#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#define NUM_SENSORS 5
const int sensor_pins[NUM_SENSORS] = {A11, A0, A2, A3, A4};  // Define sensor pins
#define EMIT_PIN 11  // Define the infrared emitter pin

class LineSensors_c {
public:
    float readings[NUM_SENSORS];     // Store raw sensor readings
    float minimum[NUM_SENSORS];      // Store minimum calibration values
    float maximum[NUM_SENSORS];      // Store maximum calibration values
    float scaling[NUM_SENSORS];      // Store calibration scaling factors
    float calibrated[NUM_SENSORS];   // Store calibrated sensor values
    float threshold = 0.5;           // Threshold for black line detection (calibrated value)

    LineSensors_c() {
        // Constructor (empty)
    }

    // Initialize the sensors for ADC readings
    void initialiseForADC() {
        // Turn on the infrared emitter
        pinMode(EMIT_PIN, OUTPUT);
        digitalWrite(EMIT_PIN, HIGH);

        // Initialize sensor pins
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            pinMode(sensor_pins[sensor], INPUT_PULLUP);
        }
    }

    // Read the raw ADC values from the sensors
    void readSensorsADC() {
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            readings[sensor] = analogRead(sensor_pins[sensor]);
        }
    }

    // Calibrate the sensors by finding their minimum and maximum values
    void calibrateSensors() {
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            minimum[sensor] = 1023;  // Initialize to max ADC value
            maximum[sensor] = 0;     // Initialize to min ADC value
        }

        // Simulate calibration process by rotating over a black-and-white surface
        for (int i = 0; i < 100; i++) {
            readSensorsADC();
            for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
                if (readings[sensor] > maximum[sensor]) {
                    maximum[sensor] = readings[sensor];  // Record the maximum value
                }
                if (readings[sensor] < minimum[sensor]) {
                    minimum[sensor] = readings[sensor];  // Record the minimum value
                }
            }
            delay(10);  // Slight delay during rotation
        }

        // Calculate scaling factors for each sensor
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            scaling[sensor] = maximum[sensor] - minimum[sensor];
            if (scaling[sensor] == 0) scaling[sensor] = 1;  // Avoid division by 0
        }
    }

    // Calculate the calibrated sensor values (scaled between 0 and 1)
    void calcCalibratedADC() {
        readSensorsADC();  // First, read the latest sensor values

        // Normalize sensor readings to the range [0, 1]
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            calibrated[sensor] = (readings[sensor] - minimum[sensor]) / scaling[sensor];
            if (calibrated[sensor] < 0) calibrated[sensor] = 0;
            if (calibrated[sensor] > 1) calibrated[sensor] = 1;
        }
    }

    // Check if any of the sensors detect a black line (below the threshold)
    bool isOnLine() {
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            if (calibrated[sensor] < threshold) {
                return true;  // A black line is detected
            }
        }
        return false;  // No black line detected
    }
};

#endif
