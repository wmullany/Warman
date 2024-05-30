#include <Servo.h>
#include <math.h>

int leftAngle = 0;    //Current left angle
int rightAngle = 165;  //Current right angle

int leftHeight = 15;
int rightHeight = 15;

Servo leftSpool;
Servo rightSpool;

Servo leftRotate;
Servo rightRotate;

void myDelay(int ms) {
    int end = millis() + ms;
    while (millis() < end) {}; // do nothing
}

/**
 * Move right a servo smoothly from a start position to an end position.
 *
 * @param start Starting position of the servo in degrees.
 * @param end Ending position of the servo in degrees.
 */
void moveRightServoSmooth(int start, int end) {

  int pos = start;  // Initialize position to start
  unsigned long previousMillis = millis();
  ;
  unsigned long delayMillis = 15;  // Delay between servo movements in milliseconds

  // Determine direction of movement
  int increment = (end > start) ? 1 : -1;

  // Move servo towards the end position
  while (pos != end) {

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= delayMillis) {

      pos += increment;        // Increment position
      rightRotate.write(pos);  // Move servo to new position

      if (abs(pos - start) <35 || abs(pos - end) < 35) {  // If near start or end, go slower.

        delayMillis = 45;  // Adjust delay to control speed

      } else {

        delayMillis = 15;  // Adjust delay to control speed
      }

      previousMillis = currentMillis;
    }
  }

  rightAngle = end;
}

/**
 * Move left a servo smoothly from a start position to an end position.
 *
 * @param start Starting position of the servo in degrees.
 * @param end Ending position of the servo in degrees.
 */
void moveLeftServoSmooth(int start, int end) {

  int pos = start;  // Initialize position to start
  unsigned long previousMillis = millis();
  unsigned long delayMillis = 15;  // Delay between servo movements in milliseconds

  // Determine direction of movement
  int increment = (end > start) ? 1 : -1;

  // Move servo towards the end position
  while (pos != end) {

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= delayMillis) {

      pos += increment;       // Increment position
      leftRotate.write(pos);  // Move servo to new position

      if (abs(pos - start) < 40 || abs(pos - end) < 40) {  // If near start or end, go slower.

        delayMillis = 40;  // Adjust delay to control speed

      } else {

        delayMillis = 30;  // Adjust delay to control speed
      }

      previousMillis = currentMillis;
    }
  }

  leftAngle = end;
}
/*
case LEFT_UP:
      Serial.print("Left Up\n");
      digitalWrite(13,HIGH);
      digitalWrite(12,LOW);
      break;
    case LEFT_DOWN:
      Serial.print("Left Down\n");
      digitalWrite(13,LOW);
      digitalWrite(12,HIGH);
      break;
*/
void setLeftActuatorHeight(int start, int end) {

  float strokeLength = 50;  //50mm stroke length
  int speed = 15;           //15mm/s speed
  float totalTimeFrom0to100 = strokeLength / speed;

  float duration = 1000 * totalTimeFrom0to100 * abs(((float)start - (float)end) / 100);
  int delay = round(duration);

  Serial.println(totalTimeFrom0to100);
  Serial.println(duration);

  // Go up or go down
  if (start < end) {

    digitalWrite(13, HIGH);
    digitalWrite(12, LOW);

  } else if (start > end) {

    digitalWrite(13, LOW);
    digitalWrite(12, HIGH);
  }

  myDelay(duration);

  digitalWrite(13, LOW);
  digitalWrite(12, LOW);

  leftHeight = end; 
}

/*
// Lower Right Arm
  digitalWrite(8,LOW);
  digitalWrite(4,HIGH);
  delay(1500);

  // Stop Height Change
  digitalWrite(8,LOW);
  digitalWrite(4,LOW);
*/

void setRightActuatorHeight(int start, int end) {

  float strokeLength = 50;  //50mm stroke length
  int speed = 5;           //15mm/s speed
  float totalTimeFrom0to100 = strokeLength / speed;

  float duration = 1000 * totalTimeFrom0to100 * abs(((float)start - (float)end) / 100);
  int delay = round(duration);

  Serial.println(totalTimeFrom0to100);
  Serial.println(duration);

  // Go up or go down
  if (start < end) {

    digitalWrite(8, HIGH);
    digitalWrite(4, LOW);

  } else if (start > end) {

    digitalWrite(8, LOW);
    digitalWrite(4, HIGH);
  }

  myDelay(duration);

  digitalWrite(8, LOW);
  digitalWrite(4, LOW);

  rightHeight = end; 
}

void leftArmExtend(unsigned long duration) {

  leftSpool.write(0);
  myDelay(duration);
  leftSpool.write(90);

}

void rightArmExtend(unsigned long duration) {

  rightSpool.write(0);
  myDelay(duration);
}

void leftArmRetract(unsigned long duration) {

  leftSpool.write(180);
  myDelay(duration);
  leftSpool.write(90);
}

void rightArmRetract(unsigned long duration) {

  rightSpool.write(180);
  myDelay(duration);  
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);

  pinMode(8, OUTPUT);
  pinMode(4, OUTPUT);

  leftRotate.attach(10);
  rightRotate.attach(9);

  leftSpool.attach(6);
  rightSpool.attach(5);

  leftRotate.write(0);
  rightRotate.write(rightAngle);
}

int i = 0; 

void loop() {
  // put your main code here, to run repeatedly:

  //Initial angles get updated by the functions

  /*
    |-------------------------------------------------|      
    |    1                  O                     4   |  
    |                     |---|                       |   
    |     2               |   |                  5    |   
    |                     | R |                       |   
    |    3                |---|                   6   |   
    |                                                 | 
    |-------------------------------------------------|      
  

  
  
  //Collect First+Fourth Ball
  moveLeftServoSmooth(leftAngle, 45);
  moveRightServoSmooth(rightAngle, 120);

  //Extend Arms
  rightArmExtend(1000); 
  leftArmExtend(1000);

  //Collect Ball and Lift Arms
  delay(1000); 
  setLeftActuatorHeight(leftHeight, leftHeight + 20); 
  setRightActuatorHeight(rightHeight, rightHeight + 20); 

  //Retract Arms
  rightArmRetract(1000); 
  leftArmRetract(1000);
  
  //Collect Second+Fifth Ball
  moveLeftServoSmooth(leftAngle, 90);
  moveRightServoSmooth(leftAngle, 90);

  //Extend Arms
  rightArmExtend(1000); 
  leftArmExtend(1000);

  //Collect Ball and Lift Arms
  delay(1000); 
  setLeftActuatorHeight(leftHeight, leftHeight + 20); 
  setRightActuatorHeight(rightHeight, rightHeight + 20); 

  //Retract Arms
  rightArmRetract(1000); 
  leftArmRetract(1000);
  
  //Collect Third+Sixth Ball
  moveLeftServoSmooth(leftAngle, 180);
  moveRightServoSmooth(leftAngle, 90);

  //Collect Ball and Lift Arms
  delay(1000); 
  setLeftActuatorHeight(leftHeight, leftHeight + 20); 
  setRightActuatorHeight(rightHeight, rightHeight + 20); 

  //Retract Arms
  rightArmRetract(1000); 
  leftArmRetract(1000);
  
  
  delay(3000);
  moveLeftServoSmooth(leftAngle, 0); 
  delay(1000);
  moveLeftServoSmooth(leftAngle, 45); 
  delay(1000); 
  moveLeftServoSmooth(leftAngle, 90); 
  delay(1000);
  moveLeftServoSmooth(leftAngle, 135); 
  delay(1000); 
  moveLeftServoSmooth(leftAngle, 180); 
  delay(1000);
  moveLeftServoSmooth(leftAngle, 90); 
  delay(1000);
  moveLeftServoSmooth(leftAngle, 45); 
  delay(1000);
  moveLeftServoSmooth(leftAngle, 0); 
  delay(1000);
  
  
  leftRotate.write(0);
  delay(1000);
  leftRotate.write(10);
  delay(1000);
  leftRotate.write(20);
  delay(1000);
  */
  

  if (i == 0) {  


    // Move linear actuator to the bottom of its range
    setLeftActuatorHeight(100, 0);
    setLeftActuatorHeight(100, 0);

    // Set Rotation Servo to 0
    moveLeftServoSmooth(leftAngle, 0); 
    delay(1000);

    // Move to the first ball and collect
    moveLeftServoSmooth(leftAngle, 70); 
    delay(1000);
    leftArmExtend(5500);
    leftSpool.write(90); // Stop extending 
    setLeftActuatorHeight(0, 43); // Pickup
    delay(1000); 
    leftArmRetract(2500); //Dont need this much

    // Move to second ball
    moveLeftServoSmooth(leftAngle, 109); 
    delay(1000); 
    setLeftActuatorHeight(43, 83);
    delay(1000);

    //Extend and pickup second ball
    leftArmExtend(3000);
    leftSpool.write(90);
    setLeftActuatorHeight(43, 73);
    delay(1000); 
    leftArmRetract(3000);

    i += 1; 
  }
}
