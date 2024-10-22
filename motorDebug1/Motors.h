#ifndef _MOTORS_H
#define _MOTORS_H

// Correct Pin definitions
#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

// Maximum PWM value
#define MAX_PWM 180.0

// Class to operate the motors.
class Motors_c {

  public:

    // Constructor, must exist.
    Motors_c() {
      // Leave empty. Ensure initialise() is called instead.
    }

    // Initialize motor pins
    void initialise() {
      // Set motor pins as outputs
      pinMode(L_PWM, OUTPUT);
      pinMode(L_DIR, OUTPUT);
      pinMode(R_PWM, OUTPUT);
      pinMode(R_DIR, OUTPUT);

      // Set initial direction
      digitalWrite(L_DIR, LOW);
      digitalWrite(R_DIR, LOW);
    }

    // Set PWM power for the motors
    void setPWM(float left_pwr, float right_pwr) {
      // Control left motor direction
      if (left_pwr < 0) {
        digitalWrite(L_DIR, HIGH);  // Reverse
        left_pwr = -left_pwr;       // Convert to positive for PWM
      } else {
        digitalWrite(L_DIR, LOW);   // Forward
      }

      // Control right motor direction
      if (right_pwr < 0) {
        digitalWrite(R_DIR, HIGH);  // Reverse
        right_pwr = -right_pwr;     // Convert to positive for PWM
      } else {
        digitalWrite(R_DIR, LOW);   // Forward
      }

      // Ensure PWM values are within range [0, MAX_PWM]
      left_pwr = constrain(left_pwr, 0, MAX_PWM);
      right_pwr = constrain(right_pwr, 0, MAX_PWM);

      // Write PWM to the motors
      analogWrite(L_PWM, (int)left_pwr);
      analogWrite(R_PWM, (int)right_pwr);
    }
};

#endif
