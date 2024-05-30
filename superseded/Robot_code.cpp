/*
Author: Liam McLean
Email: agentvenom131@gmail.com
*/
#include <Servo.h>

Servo Left_Spool;
Servo Right_Spool;

Servo Left_Rotate;
Servo Right_Rotate;

#define LEFT_ANTICLOCKWISE 'a'
#define LEFT_CLOCKWISE 'd'
#define LEFT_UP 'w'
#define LEFT_DOWN 's'
#define LEFT_EXTEND 'q'
#define LEFT_RETRACT 'e'

#define RIGHT_ANTICLOCKWISE 'j'
#define RIGHT_CLOCKWISE 'l'
#define RIGHT_UP 'i'
#define RIGHT_DOWN 'k'
#define RIGHT_EXTEND 'u'
#define RIGHT_RETRACT 'o'
#define STOP ' '

int Left_Angle = 15;
int Right_Angle = 165;

char Userinput(){
  if(Serial.available()> 0)
  {
    return Serial.read();  
  }
  return 0x00;
}

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
      Left_Rotate.write(pos);  // Move servo to new position
      if (abs(pos - start) < 40 || abs(pos - end) < 40) {  // If near start or end, go slower.
        delayMillis = 100;  // Adjust delay to control speed
      } else {
        delayMillis = 45;  // Adjust delay to control speed
      }
      previousMillis = currentMillis;
    }
  }
  //leftAngle = end;
}

void setup(){
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);

  pinMode(8,OUTPUT);
  pinMode(4,OUTPUT);

  Left_Rotate.attach(10);
  Right_Rotate.attach(9);

  Left_Spool.attach(6);
  Right_Spool.attach(5);
}

void loop(){
  char input = Userinput();
  switch(input){
    case LEFT_ANTICLOCKWISE:
      Serial.print("Left AntiClockwise\n");
      //Left_Angle += 3;
      moveLeftServoSmooth(5, 90);
      //Serial.print("Anti\n");
      //Left_Angle = 90;
      //Left_Rotate.write(Left_Angle);
      break;
    case LEFT_CLOCKWISE:
      Serial.print("Left Clockwise\n");
      //Left_Angle -= 3;
      moveLeftServoSmooth(90, 5);
      //Serial.print("Clockwise\n");
      //Left_Angle = 0;
      //Left_Rotate.write(Left_Angle);
      break;
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
    case LEFT_EXTEND:
      Serial.print("Left Extend\n");
      Left_Spool.write(180);
      break;
    case LEFT_RETRACT:
      Serial.print("Left Retract\n");
      Left_Spool.write(0);
      break;
    case RIGHT_ANTICLOCKWISE:
      Serial.print("Right AntiClockwise\n");
      //Right_Angle += 3;
      Right_Angle = 180;
      Right_Rotate.write(Right_Angle);
      break;
    case RIGHT_CLOCKWISE:
      Serial.print("Right Clockwise\n");
      //Right_Angle -= 3;
      Right_Angle = 0;
      Right_Rotate.write(Right_Angle);
      break;
    case RIGHT_UP:
      Serial.print("Right Up\n");
      digitalWrite(8,HIGH);
      digitalWrite(4,LOW);
      break;
    case RIGHT_DOWN:
      Serial.print("Right Down\n");
      digitalWrite(8,LOW);
      digitalWrite(4,HIGH);
      break;
    case RIGHT_EXTEND:
      Serial.print("Right Extend\n");
      Right_Spool.write(0);
      break;
    case RIGHT_RETRACT:
      Serial.print("Right Retract\n");
      Right_Spool.write(180);
      break;
    case STOP:
      Serial.print("Stop All\n");
      //Left_Rotate.write(Left_Angle);
      //Right_Rotate.write(Right_Angle);
      Left_Spool.write(90);
      Right_Spool.write(90);
      digitalWrite(8,LOW);
      digitalWrite(4,LOW);
      digitalWrite(13,LOW);
      digitalWrite(12,LOW);
      break;
    default:
      break;

  }

}
  /*
  // Iniatial Arm states
  Left_Rotate.write(15);
  Right_Rotate.write(165);
  delay(2000);

  // Rotate Out
  Left_Rotate.write(45);
  Right_Rotate.write(135);
  delay(2000);
  // Lower Right Arm

  digitalWrite(8,LOW);
  digitalWrite(4,HIGH);
  delay(1500);

  // Stop Height Change
  digitalWrite(8,LOW);
  digitalWrite(4,LOW);

  // Extend Right Arm
  Right_Spool.write(0);
  delay(4000);

  // Raise Right Arm
  digitalWrite(8,HIGH);
  digitalWrite(4,LOW);
  delay(4000);

  // Retract Right Arm
  Right_Spool.write(180);
  delay(3000);
  */
