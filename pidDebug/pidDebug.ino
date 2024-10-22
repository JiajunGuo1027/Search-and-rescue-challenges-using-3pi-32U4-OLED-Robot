#include "Motors.h"
#include "PID.h"

Motors_c motors; 
PID_c left_pid; // Let's just work with one motor to start

float demand;   // Demand wheel speed.

// We will use a timestamp and millis to change ourdemand periodically, to test the response of the PID
unsigned long test_ts;
# define TEST_MS 2000

void setup() {

  // Initialise all gains to zero for now.
  left_pid.initialise( 0.0, 0.0, 0.0 );

  // Prepare our testing timestamp
  test_ts = millis();

  // Remember to reset your PID if you have
  // used any delay()
  left_pid.reset();
}

void loop() {

  // This flips the demand to be a wheel
  // speed in the other direction every
  // TEST_MS.  
  if( millis() - test_ts > TEST_MS ) {
    test_ts = millis();

    demand = demand * -1.0;
  }

  // You will need to ensure you are calculating
  // an estimate of wheel speed.
  // ...
  // ...


  // You should encapsulate the following line within
  // a task scheduling millis() block, similar to the
  // above. You should schedule your pid update to be
  // at a frequency slower than your speed estimate is
  // updated.  Try 50ms.
  // replace "measurement" with your wheel speed
  // measurement variable.
  float l_pwm = left_pid.update( demand, 15 );

  // Just test the left motor
  motors.setPWM( l_pwm, 0 );

  // Serial prints to help us debug and tune
  Serial.print( demand );
  Serial.print( "," );
  Serial.print( 15 );
  Serial.print( "\n" );

}
