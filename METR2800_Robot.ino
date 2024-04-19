include <Servo.h>

Servo leftSpool;
Servo rightSpool;
Servo leftRotate;
Servo rightRotate;


int extend = 180;
int retract = 0;

int left = 180;
int right = 180;

int leftAct = 10;
int rightAct = 11;

void setup() {
  // put your setup code here, to run once:
  leftSpool.attach(3);
  rightSpool.attach(5);
  leftRotate.attach(6);
  rightRotate.attach(9);
  pinMode(leftAct,OUTPUT);
  pinMode(rightAct),OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  //delay timing and position to be changes during testing

  // rotate and extend arms
  leftSpool.write(extend);
  rightSpool.write(extend);
  leftRotate.write(left);
  rightRotate.write(right);
  digitalWrite(leftAct, HIGH);
  digitalWrite(rightAct, HIGH);

  delay(1000);

  // second ball
  leftSpool.write(extend);
  rightSpool.write(extend);
  leftRotate.write(left);
  rightRotate.write(right);
  digitalWrite(leftAct, HIGH);
  digitalWrite(rightAct, HIGH);

  delay(1000);

  // third ball
  leftSpool.write(extend);
  rightSpool.write(extend);
  leftRotate.write(left);
  rightRotate.write(right);
  digitalWrite(leftAct, HIGH);
  digitalWrite(rightAct, HIGH);

  delay(1000);
  
  


}
