#include "Motors.h"

Motors_c motors;

void setup() {
  // Initialize motors
  motors.initialise();
}

void loop() {
  // Move forward for 2 seconds
  motors.setPWM(50, 50);
  delay(2000);

  // Move backward for 2 seconds
  motors.setPWM(-50, -50);
  delay(2000);

  // Turn left for 1 second
  motors.setPWM(0, 50);
  delay(1000);

  // Turn right for 1 second
  motors.setPWM(50, 0);
  delay(1000);

  // Spin in place for 1 second
  motors.setPWM(50, -50);
  delay(1000);
  
  // Stop
  motors.setPWM(0, 0);
  delay(2000);
}
