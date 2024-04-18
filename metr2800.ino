#include <Servo.h>

Servo servo1;
Servo servo2;

unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long interval = 5000; // 5 seconds interval

unsigned long actuatorActivationTime = 1000; // 1 second activation time for linear actuators
unsigned long actuatorPreviousTime1 = 0;
unsigned long actuatorPreviousTime2 = 0;

int state = 0; // State variable to track the current state

#define ACTUATOR_PIN_1 8 // Define the pin for linear actuator 1
#define ACTUATOR_PIN_2 7 // Define the pin for linear actuator 2

void setup() {
  servo1.attach(9);  // Attach servo1 to pin 9
  servo2.attach(10); // Attach servo2 to pin 10
  
  pinMode(ACTUATOR_PIN_1, OUTPUT); // Set linear actuator 1 pin as output
  pinMode(ACTUATOR_PIN_2, OUTPUT); // Set linear actuator 2 pin as output
  
  // Initialize servos to initial position
  servo1.write(0);
  servo2.write(0);
}

void loop() {
  currentTime = millis(); // Update current time
  
  // State machine to control servo movements and linear actuator activations
  switch(state) {
    case 0:
      if (currentTime - previousTime >= interval) {
        servo1.write(135);
        servo2.write(45);
        previousTime = currentTime;
        state++;
      }
      break;
    case 1:
      if (currentTime - previousTime >= interval) {
        servo1.write(90);
        servo2.write(90);
        previousTime = currentTime;
        state++;
      }
      break;
    case 2:
      if (currentTime - previousTime >= interval) {
        servo1.write(45);
        servo2.write(135);
        previousTime = currentTime;
        state++;
      }
      break;
    case 3:
      // Activate linear actuator 1 for 1 second during the 5-second interval
      if (currentTime - previousTime <= actuatorActivationTime) {
        digitalWrite(ACTUATOR_PIN_1, HIGH);
      } else {
        digitalWrite(ACTUATOR_PIN_1, LOW);
        actuatorPreviousTime1 = currentTime; // Reset previous time for next iteration
        state++;
      }
      break;
    case 4:
      // Activate linear actuator 2 for 1 second during the 5-second interval
      if (currentTime - actuatorPreviousTime1 <= actuatorActivationTime) {
        digitalWrite(ACTUATOR_PIN_2, HIGH);
      } else {
        digitalWrite(ACTUATOR_PIN_2, LOW);
        state = 0; // Reset state machine after completing the sequence
        previousTime = currentTime; // Reset previous time for next iteration
      }
      break;
  }
}
