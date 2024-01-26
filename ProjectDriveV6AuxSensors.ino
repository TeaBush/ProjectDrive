#include <Servo.h>

int leftIR = A4;               //IR Sensor
int rightIR = A3;             //IR Sensor
float irDistL = 0;      //Stores Left IR distance
float irDistR = 0;       //Stores right IR distance

Servo servo1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);                       //begin serial communication with the computer
  servo1.attach(7);   //set steering servo to pin 7 (change if needed)
  servo1.write(90);  //Center steering
}

void loop() {
  
 //printing values of the sensors to the serial monitor
  Serial.print("Left:");
  Serial.print(analogRead(leftIR));
  Serial.print(",");
  Serial.print("Right:");
  Serial.println(analogRead(rightIR));
  delay(50);
 
  //Wall detected on left
  if(analogRead(leftIR + 170) <= 100) { //Applied a correction factor of 170 because left sensor reads smaller values than right
   //turn right
    servo1.write(150);
    delay(1000);
  } 

//Wall detected on right
   if(analogRead(rightIR) <= 100) {
   //turn left
    servo1.write(30);
    delay(1000);
  } 

  if(analogRead(rightIR) >= 100 && analogRead(leftIR) >= 100) { //if no obstacles detected on sides
   //Center steering
    servo1.write(90);
    delay(1000);
  } 
  
}
