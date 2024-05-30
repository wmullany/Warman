/*
 * ================================================================
 * Project: Warman Challenge Robot
 * Description:
 * Designed to control a robot within a grid environment, 
 * as required by the Warman Challenge. The robot has two 
 * servo-controlled arms and linear actuators for to pick 
 * up pods within the grid.
 * 
 * Dependencies:
 * - Servo library for controlling servos.
 * - Math library for mathematical calculations.
 *
 * Author: Will Mullany
 * Date: 1st April 2024
 * ================================================================
 */

#include <Servo.h>
#include <math.h>

// Initial angles for the left and right servos
int leftAngle = 6; // Current angle for the left servo
int rightAngle = 175; // Current angle for the right servo

// Initial heights for the left and right actuators
int leftHeight = 15; // Current height for the left actuator
int rightHeight = 15; // Current height for the right actuator

// Servo objects for controlling the spools and rotation
Servo leftSpool; // Servo controlling the left spool
Servo rightSpool; // Servo controlling the right spool
Servo leftRotate; // Servo controlling the left rotation
Servo rightRotate; // Servo controlling the right rotation

/**
 * Delays the execution for a specified amount of milliseconds.
 *
 * @param ms The number of milliseconds to delay.
 */
void myDelay(unsigned long ms) {
    unsigned long end = millis() + ms; // Calculate the end time
    while (millis() < end) {
        // Wait until the specified time has passed
    }
}

/**
 * Move the right servo smoothly from a start position to an end position.
 *
 * @param start Starting position of the servo in degrees.
 * @param end Ending position of the servo in degrees.
 */
void moveRightServoSmooth(int start, int end) {
    int pos = start; // Initialize position to start
    unsigned long previousMillis = millis(); // Store the current time
    unsigned long delayMillis = 15; // Delay between servo movements in milliseconds

    // Determine the direction of movement
    int increment = (end > start) ? 1 : -1;

    // Move the servo towards the end position
    while (pos != end) {
        unsigned long currentMillis = millis(); // Get the current time
        if (currentMillis - previousMillis >= delayMillis) {
            pos += increment; // Increment position
            rightRotate.write(pos); // Move the servo to the new position
            // Adjust delay to control speed near the start or end position
            if (abs(pos - start) < 20 || abs(pos - end) < 20) {
                delayMillis = 30;
            } else {
                delayMillis = 10;
            }
            previousMillis = currentMillis; // Update the previous time
        }
    }
    rightAngle = end; // Update the global right angle
}

/**
 * Move the left servo smoothly from a start position to an end position.
 *
 * @param start Starting position of the servo in degrees.
 * @param end Ending position of the servo in degrees.
 */
void moveLeftServoSmooth(int start, int end) {
    int pos = start; // Initialize position to start
    unsigned long previousMillis = millis(); // Store the current time
    unsigned long delayMillis = 15; // Delay between servo movements in milliseconds

    // Determine the direction of movement
    int increment = (end > start) ? 1 : -1;

    // Move the servo towards the end position
    while (pos != end) {
        unsigned long currentMillis = millis(); // Get the current time
        if (currentMillis - previousMillis >= delayMillis) {
            pos += increment; // Increment position
            leftRotate.write(pos); // Move the servo to the new position
            // Adjust delay to control speed near the start or end position
            if (abs(pos - start) < 20 || abs(pos - end) < 20) {
                delayMillis = 30;
            } else {
                delayMillis = 10;
            }
            previousMillis = currentMillis; // Update the previous time
        }
    }
    leftAngle = end; // Update the global left angle
}

/**
 * Set the height of the left linear actuator.
 *
 * @param start The starting height percentage (0 to 100).
 * @param end The ending height percentage (0 to 100).
 */
void setLeftActuatorHeight(int start, int end) {
    float strokeLength = 50; // Stroke length in mm
    int speed = 5; // Speed in mm/s
    float totalTimeFrom0to100 = strokeLength / speed; // Total time to move from 0% to 100%
    float duration = 1000 * totalTimeFrom0to100 * abs(((float)start - (float)end)) / 100; // Duration in milliseconds
    int delay = (unsigned long)duration; // Delay in milliseconds

    // Set the direction of the actuator
    if (start < end) {
        digitalWrite(13, HIGH);
        digitalWrite(12, LOW);
    } else if (start > end) {
        digitalWrite(13, LOW);
        digitalWrite(12, HIGH);
    }
    myDelay(delay); // Wait for the specified duration
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    leftHeight = end; // Update the global left height
}

/**
 * Set the height of the right linear actuator.
 *
 * @param start The starting height percentage (0 to 100).
 * @param end The ending height percentage (0 to 100).
 */
void setRightActuatorHeight(int start, int end) {
    float strokeLength = 50; // Stroke length in mm
    int speed = 5; // Speed in mm/s
    float totalTimeFrom0to100 = strokeLength / speed; // Total time to move from 0% to 100%
    float duration = 1000 * totalTimeFrom0to100 * abs(((float)start - (float)end)) / 100; // Duration in milliseconds
    Serial.print("Duration ");
    Serial.println(duration);
    int delay = (unsigned long)duration; // Delay in milliseconds
    Serial.print("Delay ");
    Serial.println(delay);

    // Set the direction of the actuator
    if (start < end) {
        digitalWrite(8, HIGH);
        digitalWrite(4, LOW);
    } else if (start > end) {
        digitalWrite(8, LOW);
        digitalWrite(4, HIGH);
    }
    myDelay(delay); // Wait for the specified duration
    digitalWrite(8, LOW);
    digitalWrite(4, LOW);
    rightHeight = end; // Update the global right height
}

/**
 * Extend the left arm for a specified duration.
 *
 * @param duration The duration to extend the arm in milliseconds.
 */
void leftArmExtend(unsigned long duration) {
    leftSpool.write(0); // Extend the left arm
    myDelay(duration); // Wait for the specified duration
    leftSpool.write(90); // Stop the left arm
}

/**
 * Extend the right arm for a specified duration.
 *
 * @param duration The duration to extend the arm in milliseconds.
 */
void rightArmExtend(unsigned long duration) {
    rightSpool.write(180); // Extend the right arm
    myDelay(duration); // Wait for the specified duration
    rightSpool.write(92); // Stop the right arm
}

/**
 * Retract the left arm for a specified duration.
 *
 * @param duration The duration to retract the arm in milliseconds.
 */
void leftArmRetract(unsigned long duration) {
    leftSpool.write(180); // Retract the left arm
    myDelay(duration); // Wait for the specified duration
    leftSpool.write(90); // Stop the left arm
}

/**
 * Retract the right arm for a specified duration.
 *
 * @param duration The duration to retract the arm in milliseconds.
 */
void rightArmRetract(unsigned long duration) {
    rightSpool.write(0); // Retract the right arm
    myDelay(duration); // Wait for the specified duration
    rightSpool.write(92); // Stop the right arm
}

/**
 * Start moving the left linear actuator from start height to end height.
 *
 * @param start The starting height percentage (0 to 100).
 * @param end The ending height percentage (0 to 100).
 */
void STARTLeftLinearAct(int start, int end) {
    // Set the direction of the actuator
    if (start < end) {
        digitalWrite(13, HIGH);
        digitalWrite(12, LOW);
    } else if (start > end) {
        digitalWrite(13, LOW);
        digitalWrite(12, HIGH);
    }
}

/**
 * Stop the left linear actuator.
 */
void STOPLeftLinearAct(void) {
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
}

/**
 * Start moving the right linear actuator from start height to end height.
 *
 * @param start The starting height percentage (0 to 100).
 * @param end The ending height percentage (0 to 100).
 */
void STARTRightLinearAct(int start, int end) {
    // Set the direction of the actuator
    if (start < end) {
        digitalWrite(8, HIGH);
        digitalWrite(4, LOW);
    } else if (start > end) {
        digitalWrite(8, LOW);
        digitalWrite(4, HIGH);
    }
}

/**
 * Stop the right linear actuator.
 */
void STOPRightLinearAct(void) {
    digitalWrite(8, LOW);
    digitalWrite(4, LOW);
}

/**
 * Start extending the left spool.
 */
void left_Extend_START() {
    leftSpool.write(0); // Command the left spool to extend
}

/**
 * Start retracting the left spool.
 */
void left_Retract_START() {
    leftSpool.write(180); // Command the left spool to retract
}

/**
 * Stop the left spool.
 */
void left_SPOOL_STOP() {
    leftSpool.write(90); // Command the left spool to stop
}

/**
 * Start extending the right spool.
 */
void right_Extend_START() {
    rightSpool.write(180); // Command the right spool to extend
}

/**
 * Start retracting the right spool.
 */
void right_Retract_START() {
    rightSpool.write(0); // Command the right spool to retract
}

/**
 * Stop the right spool.
 */
void right_SPOOL_STOP() {
    rightSpool.write(92); // Command the right spool to stop
}

/*
* Function: moveBothServosSmooth
* Description: Moves two servos smoothly from their start positions to end positions.
* Parameters:
*   - startLeft: The initial position of the left servo.
*   - endLeft: The final position of the left servo.
*   - startRight: The initial position of the right servo.
*   - endRight: The final position of the right servo.
*/
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

/*
 * Function: firstAndFourth
 * Description: Performs a series of movements to manipulate robotic arms in a specific sequence.
 * This function controls extension, rotation, lowering, raising, and retraction of the arms.
 */
void firstAndFourth(void) { 

  // Extend Arms 
  right_Extend_START(); // Extend right arm
  left_Extend_START();  // Extend left arm
  myDelay(3300);  // Delay for extension
  right_SPOOL_STOP();  // Stop spooling of right arm
  myDelay(800);  // Delay for stabilization
  left_SPOOL_STOP();  // Stop spooling of left arm

  // Rotate Both Arms
  // moveBothServosSmooth(int startLeft, int endLeft, int startRight, int endRight)
  int endLeft = 65;  // Set end angle for left arm rotation
  int endRight = 124;  // Set end angle for right arm rotation
  moveBothServosSmooth(leftAngle, endLeft, rightAngle, endRight);  // Move both arms smoothly to specified angles

  // Lower Both Arms
  STARTRightLinearAct(65, 0);   // Lower right arm
  STARTLeftLinearAct(60, 0);   // Lower left arm
  myDelay(6000);  // Delay for lowering
  STOPLeftLinearAct();   // Stop lowering of left arm
  myDelay(500);  // Delay for stabilization
  STOPRightLinearAct();  // Stop lowering of right arm

  // Extend and Raise Both Arms to pick up balls
  right_Extend_START(); // Extend right arm
  left_Extend_START(); // Extend left arm
  myDelay(1800);  // Delay for extension
  left_SPOOL_STOP();  // Stop spooling of left arm
  STARTLeftLinearAct(0, 40);  // Raise left arm
  myDelay(1350);  // Delay for raising
  right_SPOOL_STOP();  // Stop spooling of right arm
  STARTRightLinearAct(0, 75);  // Raise right arm
  myDelay(2650);  // Delay for raising
  STOPLeftLinearAct();  // Stop raising of left arm

  // Retract to deposit 
  myDelay(2410);  // Delay for preparation
  left_SPOOL_STOP();  // Stop spooling of left arm
  STARTLeftLinearAct(0, 110);  // Raise left arm
  myDelay(2440);  // Delay for raising
  STOPRightLinearAct();  // Stop raising of right arm
  myDelay(7960);  // Delay for stabilization
  left_Retract_START(); // Retract left arm
  myDelay(1000);  // Delay for retraction
  left_SPOOL_STOP();  // Stop spooling of left arm
  STOPLeftLinearAct();  // Stop retracting of left arm
  
  right_Retract_START(); // Retract right arm
  STARTLeftLinearAct(85, 0);   // Lower left arm
  left_Extend_START(); // Extend left arm
  myDelay(150);  // Delay for extension
  left_SPOOL_STOP();  // Stop spooling of left arm
  myDelay(2460);  // Delay for stabilization
  right_SPOOL_STOP();  // Stop spooling of right arm
  myDelay(5290);  // Delay for stabilization
  STOPLeftLinearAct();  // Stop lowering of left arm
  left_Retract_START(); // Retract left arm
  myDelay(2900);  // Delay for retraction
  left_SPOOL_STOP();  // Stop spooling of left arm
}

/*
 * Function: thirdAndFifth
 * Description: Performs a series of movements to manipulate robotic arms in a specific sequence.
 * This function controls rotation, extension, lowering, raising, and retraction of the arms.
 */
void thirdAndFifth(void) { 

  // Rotate early to stop hitting with other arm
  moveLeftServoSmooth(leftAngle, 90);  // Move left arm smoothly to 90 degrees

  int endLeft = 132;  // Set end angle for left arm rotation
  int endRight = 86;  // Set end angle for right arm rotation
  moveBothServosSmooth(leftAngle, endLeft, rightAngle, endRight);  // Move both arms smoothly to specified angles

  left_Extend_START(); // Extend left arm
  right_Extend_START(); // Extend right arm
  myDelay(1180);  // Delay for extension
  left_SPOOL_STOP();  // Stop spooling of left arm
  myDelay(300);  // Delay for stabilization
  right_SPOOL_STOP();  // Stop spooling of right arm


  STARTLeftLinearAct(70, 0);  // Lower left arm
  STARTRightLinearAct(0, 90);  // Lower right arm
  myDelay(7000);  // Delay for lowering
  STOPLeftLinearAct();  // Stop lowering of left arm
  left_Extend_START(); // Extend left arm
  myDelay(2500);  // Delay for extension
  STOPRightLinearAct();  // Stop lowering of right arm
  myDelay(270);  // Delay for stabilization
  left_SPOOL_STOP();  // Stop spooling of left arm

  STARTLeftLinearAct(0, 80);  // Raise left arm
  right_Retract_START(); // Retract right arm
  myDelay(2000);  // Delay for retraction
  right_SPOOL_STOP();  // Stop spooling of right arm
  STARTRightLinearAct(90, 0);  // Raise right arm
  myDelay(6000);  // Delay for raising
  left_Retract_START(); // Retract left arm
  myDelay(2180);  // Delay for retraction
  left_SPOOL_STOP();  // Stop spooling of left arm
  STARTLeftLinearAct(0, 70);  // Lower left arm
  myDelay(4760);  // Delay for lowering
  STOPRightLinearAct();  // Stop lowering of right arm
  myDelay(1640);  // Delay for stabilization
  left_Retract_START(); // Retract left arm
  myDelay(600);  // Delay for retraction
  STOPLeftLinearAct();  // Stop retracting of left arm
  myDelay(1000);  // Delay for stabilization
  left_SPOOL_STOP();  // Stop spooling of left arm
  
  STARTLeftLinearAct(95, 0);  // Lower left arm
  myDelay(9500);  // Delay for lowering
  STOPLeftLinearAct();  // Stop lowering of left arm

}

/*
 * Function: secondAndSixth
 * Description: Picks up the second and sixth ball (last) using robotic arms.
 */
void secondAndSixth(void) { 
  int endLeft = 98;  // Set end angle for left arm rotation
  int endRight = 62;  // Set end angle for right arm rotation

  moveBothServosSmooth(leftAngle, endLeft, rightAngle, endRight);  // Move both arms smoothly to specified angles

  left_Extend_START(); // Extend left arm
  myDelay(1200);  // Delay for extension
  right_Extend_START(); // Extend right arm
  myDelay(1300);  // Delay for extension
  left_SPOOL_STOP();  // Stop spooling of left arm

  STARTLeftLinearAct(0, 45);  // Raise left arm
  myDelay(550);  // Delay for raising
  right_SPOOL_STOP();  // Stop spooling of right arm

  STARTRightLinearAct(20, 0);  // Lower right arm
  myDelay(2000);  // Delay for lowering
  STOPRightLinearAct();  // Stop lowering of right arm

  right_Extend_START(); // Extend right arm
  myDelay(1200);  // Delay for extension
  right_SPOOL_STOP();  // Stop spooling of right arm
  STARTRightLinearAct(0, 80);  // Raise right arm
  myDelay(750);  // Delay for raising
  STOPLeftLinearAct();  // Stop raising of left arm
  myDelay(7250);  // Delay for stabilization
  STOPRightLinearAct();  // Stop raising of right arm

  STARTLeftLinearAct(0, 90);  // Raise left arm
  right_Retract_START();  // Retract right arm
  myDelay(1400);  // Delay for retraction
  right_SPOOL_STOP();  // Stop spooling of right arm
  STARTRightLinearAct(0, 70);  // Lower right arm
  myDelay(7000);  // Delay for lowering
  STOPRightLinearAct();  // Stop lowering of right arm
  STOPLeftLinearAct();  // Stop raising of left arm

  left_Retract_START();  // Retract left arm
  myDelay(600);  // Delay for retraction
  left_SPOOL_STOP();  // Stop spooling of left arm
  right_Retract_START();  // Retract right arm
  myDelay(1200);  // Delay for retraction

  // This is shit code
  right_SPOOL_STOP();  // Stop spooling of right arm

}

void setup() {
    // Initialize serial communication
    Serial.begin(9600);

    // Set pin modes for controlling servos and other components
    pinMode(13, OUTPUT);  // Pin 13 for output
    pinMode(12, OUTPUT);  // Pin 12 for output
    pinMode(8, OUTPUT);   // Pin 8 for output
    pinMode(4, OUTPUT);   // Pin 4 for output

    // Attach servos to respective pins
    leftRotate.attach(10);   // Attach left rotation servo to pin 10
    rightRotate.attach(9);   // Attach right rotation servo to pin 9
    leftSpool.attach(6);      // Attach left spool servo to pin 6
    rightSpool.attach(5);     // Attach right spool servo to pin 5

    // Set initial positions for servos
    leftRotate.write(leftAngle);   // Set initial position for left rotation servo
    rightRotate.write(rightAngle); // Set initial position for right rotation servo
}

/*
 * Function: loop
 * Description: Executes the main loop of the robotic arm control system.
 * This function contains the sequence of actions to be performed by the robotic arms.
 */
void loop() {
    
    if (!executed) {

        /* Starting Sequence */
        rightArmExtend(1200);  // Extend right arm
        // delay(50);  // Uncomment this line for a delay
        setLeftActuatorHeight(9, 0);  // Set left actuator height
        rightSpool.write(93);  // Set position of right spool
        moveRightServoSmooth(rightAngle, 168);  // Move right servo smoothly to specified angle

        firstAndFourth();  // Execute first and fourth function

        thirdAndFifth();  // Execute third and fifth function

        secondAndSixth();  // Execute second and sixth function


        ///End of demo
        setRightActuatorHeight(90, 0);  // Set right actuator height
        setLeftActuatorHeight(70, 0);   // Set left actuator height
        leftSpool.write(94);  // Set position of left spool

        /* Right Arm Closing Sequence */
        delay(1000);  // Delay for 1 second
        moveLeftServoSmooth(leftAngle, 4);  // Move left servo smoothly to specified angle
        moveRightServoSmooth(rightAngle, 175);  // Move right servo smoothly to specified angle
        rightSpool.write(89);  // Set position of right spool
        leftSpool.write(96);   // Set position of left spool

        ///Keep this one

        executed = true;  // Set executed flag to true
    }
}
