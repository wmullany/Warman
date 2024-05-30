#include <Servo.h>
#include <math.h>

int leftAngle = 6; // Current left angle
int rightAngle = 175; // Current right angle
int leftHeight = 15;
int rightHeight = 15;

Servo leftSpool;
Servo rightSpool;
Servo leftRotate;
Servo rightRotate;

void myDelay(unsigned long ms) {
    unsigned long end = millis() + ms;
    while (millis() < end) {
    }
    // do nothing
}

/**
 * Move right a servo smoothly from a start position to an end position.
 *
 * @param start Starting position of the servo in degrees.
 * @param end Ending position of the servo in degrees.
 */
void moveRightServoSmooth(int start, int end) {
    int pos = start; // Initialize position to start
    unsigned long previousMillis = millis();
    unsigned long delayMillis = 15; // Delay between servo movements in milliseconds

    // Determine direction of movement
    int increment = (end > start) ? 1 : -1;

    // Move servo towards the end position
    while (pos != end) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= delayMillis) {
            pos += increment; // Increment position
            rightRotate.write(pos); // Move servo to new position
            if (abs(pos - start) < 20 || abs(pos - end) < 20) {
                // If near start or end, go slower.
                delayMillis = 30; // Adjust delay to control speed
            } else {
                delayMillis = 10; // Adjust delay to control speed
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
    int pos = start; // Initialize position to start
    unsigned long previousMillis = millis();
    unsigned long delayMillis = 15; // Delay between servo movements in milliseconds

    // Determine direction of movement
    int increment = (end > start) ? 1 : -1;

    // Move servo towards the end position
    while (pos != end) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= delayMillis) {
            pos += increment; // Increment position
            leftRotate.write(pos); // Move servo to new position
            if (abs(pos - start) < 20 || abs(pos - end) < 20) {
                // If near start or end, go slower.
                delayMillis = 30; // Adjust delay to control speed
            } else {
                delayMillis = 10; // Adjust delay to control speed
            }
            previousMillis = currentMillis;
        }
    }
    leftAngle = end;
}

void setLeftActuatorHeight(int start, int end) {
    float strokeLength = 50; // 50mm stroke length
    int speed = 5; // 15mm/s speed
    float totalTimeFrom0to100 = strokeLength / speed;
    float duration = 1000 * totalTimeFrom0to100 * abs(((float)start - (float)end)) / 100;
    int delay = (unsigned long)duration;

    // Go up or go down
    if (start < end) {
        digitalWrite(13, HIGH);
        digitalWrite(12, LOW);
    } else if (start > end) {
        digitalWrite(13, LOW);
        digitalWrite(12, HIGH);
    }
    myDelay(delay);
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    leftHeight = end;
}


void setRightActuatorHeight(int start, int end) {
    float strokeLength = 50; // 50mm stroke length
    int speed = 5; // 15mm/s speed
    float totalTimeFrom0to100 = strokeLength / speed;
    float duration = 1000 * totalTimeFrom0to100 * abs(((float)start - (float)end)) / 100;
    Serial.print("Duration ");
    Serial.println(duration);
    int delay = (unsigned long)duration;
    Serial.print("Delay ");
    Serial.println(delay);

    // Go up or go down
    if (start < end) {
        digitalWrite(8, HIGH);
        digitalWrite(4, LOW);
    } else if (start > end) {
        digitalWrite(8, LOW);
        digitalWrite(4, HIGH);
    }
    myDelay(delay);
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
    rightSpool.write(180);
    myDelay(duration);
    rightSpool.write(92);
}

void leftArmRetract(unsigned long duration) {
    leftSpool.write(180);
    myDelay(duration);
    leftSpool.write(90);
}

void rightArmRetract(unsigned long duration) {
    rightSpool.write(0);
    myDelay(duration);
    rightSpool.write(92);
}

void STARTLeftLinearAct(int start, int end) { 
    // Go up or go down
    if (start < end) {
        digitalWrite(13, HIGH);
        digitalWrite(12, LOW);
    } else if (start > end) {
        digitalWrite(13, LOW);
        digitalWrite(12, HIGH);
    }
}

void STOPLeftLinearAct(void) { 
  digitalWrite(13, LOW);
  digitalWrite(12, LOW);
}

void STARTRightLinearAct(int start, int end) { 
    // Go up or go down
    if (start < end) {
        digitalWrite(8, HIGH);
        digitalWrite(4, LOW);
    } else if (start > end) {
        digitalWrite(8, LOW);
        digitalWrite(4, HIGH);
    }
}

void STOPRightLinearAct(void) { 
    digitalWrite(8, LOW);
    digitalWrite(4, LOW);
}

void left_Extend_START() {
    leftSpool.write(0);
}

void left_Retract_START() {
    leftSpool.write(180);
}

void left_SPOOL_STOP() {
    leftSpool.write(90);
}

void right_Extend_START() {
    rightSpool.write(180);
}

void right_Retract_START() {
    rightSpool.write(0);
}

void right_SPOOL_STOP() {
    rightSpool.write(92);
}

void moveBothServosSmooth(int startLeft, int endLeft, int startRight, int endRight) {
    int posRight = startRight; // Initialize position for the right servo
    int posLeft = startLeft; // Initialize position for the left servo
    unsigned long previousMillisRight = millis();
    unsigned long previousMillisLeft = millis();
    unsigned long delayMillisRight = 15; // Initial delay between right servo movements in milliseconds
    unsigned long delayMillisLeft = 15; // Initial delay between left servo movements in milliseconds

    // Determine direction of movement for each servo
    int incrementRight = (endRight > startRight) ? 1 : -1;
    int incrementLeft = (endLeft > startLeft) ? 1 : -1;

    // Move servos towards their end positions
    while (posRight != endRight || posLeft != endLeft) {
        unsigned long currentMillis = millis();

        // Move right servo if it hasn't reached its end position
        if (posRight != endRight && currentMillis - previousMillisRight >= delayMillisRight) {
            posRight += incrementRight; // Increment position
            rightRotate.write(posRight); // Move right servo to new position

            // Adjust delay to control speed near start or end positions
            if (endRight < 70) { 

                delayMillisRight = 40; 
            
            } else if (abs(posRight - startRight) < 20) {
                delayMillisRight = 30 - abs(posRight - startRight) + 5;

            } else if (abs(posRight - endRight) < 20) {
                delayMillisRight = 10 + abs(endRight - posRight) + 5;

            } else {
                delayMillisRight = 15;
            }

            previousMillisRight = currentMillis;
        }

        // Move left servo if it hasn't reached its end position
        if (posLeft != endLeft && currentMillis - previousMillisLeft >= delayMillisLeft) {
            posLeft += incrementLeft; // Increment position
            leftRotate.write(posLeft); // Move left servo to new position

            // Adjust delay to control speed near start or end positions
            if (endLeft < 100 && endLeft > 96) { 
                delayMillisLeft = 40; 

            } else if (abs(posLeft - startLeft) < 20) {
                delayMillisLeft = 30 - abs(posLeft - startLeft) + 5;

            } else if (abs(posLeft - endLeft) < 20) {
                delayMillisLeft = 10 + abs(endLeft - posLeft) + 5;

            } else {
                delayMillisLeft = 15;
            }

            previousMillisLeft = currentMillis;
        }
    }

    // Update the global angles
    rightAngle = endRight;
    leftAngle = endLeft;
}


void firstAndFourth(void) { 

  //Extend Arms 
  right_Extend_START(); //3300
  left_Extend_START();  //3360
  myDelay(3300); 
  right_SPOOL_STOP(); 
  myDelay(800); 
  left_SPOOL_STOP();

  //Rotate Both Arms moveBothServosSmooth(int startLeft, int endLeft, int startRight, int endRight)
  int endLeft = 65; 
  int endRight = 124; 

  moveBothServosSmooth(leftAngle, endLeft, rightAngle, endRight); 

  //Lower Both Arms
  STARTRightLinearAct(65, 0);   //Lower
  STARTLeftLinearAct(60, 0);   //Lower
  myDelay(6000); 
  STOPLeftLinearAct();  
  myDelay(500); 
  STOPRightLinearAct(); 


  //Extend and Raise Both Arms to pick up balls
  right_Extend_START(); //3150
  left_Extend_START(); //1800
  myDelay(1800); 
  left_SPOOL_STOP(); 
  STARTLeftLinearAct(0, 40);  //Raise
  myDelay(1350); 
  right_SPOOL_STOP();
  STARTRightLinearAct(0, 75);  //Raise
  myDelay(2650); 
  STOPLeftLinearAct();  

  //Retract to deposit 
  myDelay(2410); 
  left_SPOOL_STOP();


  STARTLeftLinearAct(0, 110);  //Raise
  myDelay(2440);
  STOPRightLinearAct();
  myDelay(7960); 
  left_Retract_START(); //600
  myDelay(1000); 
  left_SPOOL_STOP(); 
  STOPLeftLinearAct(); 

  
  right_Retract_START(); //3210
  STARTLeftLinearAct(85, 0);   //Lower
  left_Extend_START(); //600
  myDelay(150); 
  left_SPOOL_STOP();
  myDelay(2460); 
  right_SPOOL_STOP();
  myDelay(5290); 
  STOPLeftLinearAct();
  left_Retract_START(); //2900
  myDelay(2900); 
  left_SPOOL_STOP();

}

/**/
void thirdAndFifth(void) { 

  //Rotate early to stop hitting with other arm
  moveLeftServoSmooth(leftAngle, 90); 

  int endLeft = 132; 
  int endRight = 86; 
  moveBothServosSmooth(leftAngle, endLeft, rightAngle, endRight); 

  left_Extend_START(); //2500
  right_Extend_START(); //2280
  myDelay(1180); 
  left_SPOOL_STOP();
  myDelay(300);
  right_SPOOL_STOP();


  STARTLeftLinearAct(70, 0);
  STARTRightLinearAct(0, 90);
  myDelay(7000); 
  STOPLeftLinearAct();
  left_Extend_START(); //2770
  myDelay(2500); 
  STOPRightLinearAct();
  myDelay(270); 
  left_SPOOL_STOP();

  STARTLeftLinearAct(0, 80);
  right_Retract_START(); //2000 
  myDelay(2000); 
  right_SPOOL_STOP();
  STARTRightLinearAct(90, 0);
  myDelay(6000); 
  left_Retract_START(); //2180
  myDelay(2180); 
  left_SPOOL_STOP();
  STARTLeftLinearAct(0, 70);
  myDelay(4760); 
  STOPRightLinearAct();
  myDelay(1640);
  left_Retract_START(); //600
  myDelay(600); 
  STOPLeftLinearAct();
  myDelay(1000); 
  left_SPOOL_STOP();
  
  STARTLeftLinearAct(95, 0);
  myDelay(9500); 
  STOPLeftLinearAct();

}

/*Picks up the second and sixth ball (last)*/
void secondAndSixth(void) { 
  int endLeft = 98; 
  int endRight = 62; 

  moveBothServosSmooth(leftAngle, endLeft, rightAngle, endRight); 

  left_Extend_START(); //1380
  myDelay(1200); 
  right_Extend_START();
  myDelay(1300);
  left_SPOOL_STOP();

  STARTLeftLinearAct(0, 45);
  myDelay(550); 
  right_SPOOL_STOP();

  STARTRightLinearAct(20, 0);
  myDelay(2000); 
  STOPRightLinearAct();

  right_Extend_START(); //1200
  myDelay(1200); 
  right_SPOOL_STOP();
  STARTRightLinearAct(0, 80);
  myDelay(750); 
  STOPLeftLinearAct();
  myDelay(7250); 
  STOPRightLinearAct();

  //moveLeftServoSmooth(leftAngle, 130); 

  STARTLeftLinearAct(0, 90);
  right_Retract_START();  //2000
  myDelay(1400); 
  right_SPOOL_STOP();
  STARTRightLinearAct(0, 70);
  myDelay(7000); 
  STOPRightLinearAct();
  STOPLeftLinearAct();

  left_Retract_START();  //300
    //1000

  myDelay(600); 

  left_SPOOL_STOP();
  right_Retract_START();

  myDelay(1200);  //Potentially reduce 

  //this is shit code
  //cheers
  right_SPOOL_STOP();

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
    leftRotate.write(leftAngle);
    rightRotate.write(rightAngle);
}

bool executed = false;

void loop() {
    
    if (!executed) {

        /* Starting Sequence */
        rightArmExtend(1200);
        // delay(50);
        setLeftActuatorHeight(9, 0);
        rightSpool.write(93);
        moveRightServoSmooth(rightAngle, 168);

        firstAndFourth(); 

        thirdAndFifth(); 

        secondAndSixth(); 


        ///End of demo
        setRightActuatorHeight(90, 0);
        setLeftActuatorHeight(70, 0);
        leftSpool.write(94);

        /* Right Arm Closing Sequence */
        delay(1000);
        moveLeftServoSmooth(leftAngle, 4);
        moveRightServoSmooth(rightAngle, 175);
        rightSpool.write(89);
        leftSpool.write(96);
      
        ///Keep this one
        executed = true;
    }
}

