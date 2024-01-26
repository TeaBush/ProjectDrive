/*
  Credit to SparkFun Inventor’s Kit
  Circuit 5C - Autonomous Robot, for original code as well as Linobot v1 by Aarav Garg
  Credit to DIY Builder for some car code, mainly bluetooth controls
  Project Drive, V3, Manual Controls
*/

//-------------------------Variables-----------------------------------------
#include <Servo.h>        //include servo library


//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor


//distance variables
const int trigPin = 6;
const int echoPin = 5;
int leftIR = A2;               //IR Sensor
int rightIR = A3;             //IR Sensor
float distance = 0;            //variable to store the distance measured by the distance sensor

//Steering variables
Servo servo1;
int steeringAngle;

//On/Off variables
int switchPin = 7;             //switch to turn the robot on and off (TODO: replace with button)

//Manual Control Varaibles
char command; 
char turn;

 


/********************************************************************************/
void setup()
{
  pinMode(trigPin, OUTPUT);       //this pin will send ultrasonic pulses out from the distance sensor
  pinMode(echoPin, INPUT);        //this pin will sense when the pulses reflect back to the distance sensor

  pinMode(switchPin, INPUT_PULLUP);   //set this as a pullup to sense whether the switch is flipped

  //Set IR sensors for line detection as inputs
  //pinMode(A2, INPUT);
  //pinMode(A3, INPUT);

  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);


  Serial.begin(9600);                       //begin serial communication with the computer
  Serial.println("Connection established.");
  
 
  //servo1.attach(5);   //TODO: Change steering pin. 5 is in use
  //steering(90);  //turn servo back to normal position. Centering is done by software for now

   rightMotor(0);   //Motors should start stopped
   leftMotor(0);

   //pinMode(5, OUTPUT); //Set automonous light indicator output to pin 5
   pinMode(switchPin, INPUT_PULLUP);
}



/********************************************************************************/


void loop() {
  //DETECT THE DISTANCE READ BY THE DISTANCE SENSOR
  distance = getDistance();
   Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" in");   // print the units
  delay(100);


  //-----------------------------AUTONOMOUS CONTROLS---------------------------------- 
  
    if (distance < 10) {              //if an object is detected
      //Stop until object moves out of path
      Serial.print(" ");
      Serial.print("Waiting for clearance");
      //steering(0);
      rightMotor(0);
      leftMotor(0);
     
    } else {                        //if no obstacle is detected drive forward
      Serial.print(" ");
      Serial.print("Path clear.");
      //steering(0);
      rightMotor(255);  //TODO: Motors always need to be manually jolted to start. Find out why.
      leftMotor(255);
    }
  
}


//------------------------FUNCTIONS---------------------------

void steering(int angle) {  
  servo1.write(angle); // turn servo1 to value of angle
  delay(100);
}

/********************************************************************************/
void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}
//---------------------------------------------------------------------------------------------
//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor

  calculatedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  return calculatedDistance;              //send back the distance that was calculated

}