#include <Servo.h>




#define TOUCHPIN 31
#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12
#define MOTOR_SPEED_LEFT 130
#define MOTOR_SPEED_RIGHT 130


#define leftTrig 13
#define leftEcho 45

// Variables to store the duration of the pulse and the distance in centimeters
long duration;
int distance;

long leftduration;
int leftdistance;

Servo arm;
Servo wrist;
Servo finger;  // Create a servo object

int apos = 0;    // Variable to store the servo position
int wpos = 0;
int fpos = 0;

int parcelPlaced = 0;
int picked = 0;


//Right motor
int enableRightMotor=6;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=5;
int leftMotorPin1=9;
int leftMotorPin2=10;

int initialround = 0;
//int metal = 0;
int permission = 0;



void setup()
{

  Serial.begin(9600);

  // Set trigPin as output and echoPin as input
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);

  pinMode(TOUCHPIN, INPUT);

  //The problem with TT gear motors is that, at very low pwm value it does not even rotate.
  //If we increase the PWM value then it rotates faster and our robot is not controlled in that speed and goes out of line.
  //For that we need to increase the frequency of analogWrite.
  //Below line is important to change the frequency of PWM signal on pin D5 and D6
  //Because of this, motor runs in controlled manner (lower speed) at high PWM value.
  //This sets frequecny as 7812.5 hz.
  TCCR0B = TCCR0B & B11111000 | B00000010 ;
  
  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  rotateMotor(0,0);   

  arm.attach(2);
  wrist.attach(3);
  finger.attach(4);  // Attach the servo to pin 9
  arm.write(120);
}

//----------------------------------------------------------------------------------------------------------------------------

void loop()
{
  int touchValue = digitalRead(TOUCHPIN);
  delay(2000);
  if (touchValue == 1){
    permission = 1;
  }
  
  if (picked == 0 && permission == 1){
    delay(10000);
    pickUp();
    picked = 1;
    initialround++;
    delay(8000);
    

  }
  
  digitalWrite(leftTrig, LOW);
  delayMicroseconds(2);
  
  digitalWrite(leftTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(leftTrig, LOW);

  leftduration = pulseIn(leftEcho, HIGH);

  leftdistance = leftduration * 0.034 / 2;
  
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
    
    

  if (leftdistance <= 6 && parcelPlaced == 0 && permission == 1) //plastic placement
  {
    rotateMotor(0,0);
    parcelPlaced = 1;
    dropItem();
    rotateMotor(MOTOR_SPEED_RIGHT, -MOTOR_SPEED_LEFT);
    //rotateMotor(120, -120);
  
  }

  /*else if (10 <= leftdistance <=17 && parcelPlaced == 0){
    rotateMotor(0,0);
    parcelPlaced = 1;
    dropItem();
    rotateMotor(MOTOR_SPEED_RIGHT, -MOTOR_SPEED_LEFT);

  }*/


  else if (rightIRSensorValue == LOW && leftIRSensorValue == LOW && permission == 1)
  {
    rotateMotor(MOTOR_SPEED_RIGHT, -MOTOR_SPEED_LEFT);
    //rotateMotor(120, -120);
  }

  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW && permission == 1 )
  {
    rotateMotor(-MOTOR_SPEED_RIGHT, -MOTOR_SPEED_LEFT); 
    //Serial.println(rightIRSensorValue);
  }
  //If left sensor detects black line, then turn left  
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH && permission == 1 )
  {
    rotateMotor(MOTOR_SPEED_RIGHT, MOTOR_SPEED_LEFT); 
    //Serial.println(rightIRSensorValue);
  } 
  //If both the sensors detect black line, then stop 
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == HIGH && permission == 1 )
  { 
    rotateMotor(0,0);
    parcelPlaced = 0;
    picked = 0;
    permission = 0;
    //Serial.println(rightIRSensorValue);
  }

}

//-----------------------------------------------------------------

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}

//-----------------------------------------------------------------------------

void pickUp() {

  for (apos = 120; apos >= 20; apos -= 1) {
    arm.write(apos);      // Set servo position
    delay(70); 
  }

  for (wpos = 0; wpos <= 100; wpos += 1) {
    wrist.write(wpos);      // Set servo position
    delay(70);               // Wait for servo to reach position
  }

    

  for (fpos = 0; fpos <= 90; fpos += 1) {
    finger.write(fpos);      // Set servo position
    delay(100);               // Wait for servo to reach position
  }

  for (wpos = 100; wpos >= 0; wpos -= 1) {
    wrist.write(wpos);      // Set servo position
    delay(70);               // Wait for servo to reach position
  }

  for (apos = 20; apos <= 120; apos += 1) {
    arm.write(apos);      // Set servo position
    delay(70);               // Wait for servo to reach position
  }

  //for (apos = 120; apos >= 20; apos -= 1) {
  //  arm.write(apos);      // Set servo position
   // delay(70);               // Wait for servo to reach position
  //}


  if (initialround > 0){
    rotateMotor(MOTOR_SPEED_RIGHT, -MOTOR_SPEED_LEFT);
    //rotateMotor(120, -120);

  }
  
  delay(5000);

}

//----------------------------------------------------------------------

void dropItem(){

  for (apos = 120; apos >= 22; apos -= 1) {
    arm.write(apos);      
    delay(70);               
  }

  for (wpos = 0; wpos <= 70; wpos += 1) {
    wrist.write(wpos);      
    delay(70);               
  }

  for (fpos = 90; fpos >= 40; fpos -= 1) {
    finger.write(fpos);      
    delay(100);               
  }

  for (fpos = 40; fpos <= 90; fpos += 1) {
    finger.write(fpos);      
    delay(100);               
  }

  for (wpos = 70; wpos >= 0; wpos -= 1) {
    wrist.write(wpos);      
    delay(70);               
  }

  for (apos = 22; apos <= 120; apos += 1) {
    arm.write(apos);      
    delay(70);             
  }

}

