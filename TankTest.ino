#include "Enes100.h"
#include <Servo.h>
#include "Tank.h"

//global 
float const tht_tol = .1;
float const pi = 3.1415;
float const turnTimeTol = 1;
float const condTol = 2;
float const pwm = 255;
//Digital pins
int const wifiTXPin = 52; //wifi module pins
int const wifiRXPin = 50;
int const motor1Pow = 4; //motors pins
int const motor1Dir = 5;
int const motor2Pow = 6;
int const motor2Dir = 7;
int const pumpPow = 8; // Pump pins
int const pumpDir = 9;
int const servoPin = 10;
int const ultraTrigPin = 11; //Ultrasonic pins
int const ultraEchoPin = 12;
//Analog pins
int const conductPin = 0;
int const float1Pin = 1;
int const float2Pin = 2;
int const float3Pin = 3;
int const photoPin = 4;
//will need to add LED to diagram and code

float missionX = 1; 
float missionY = 3;

Servo myservo; // initializes servo object

void setup() {
  // put your setup code here, to run once:
  //pin setup
  Tank.begin();
  pinMode(motor1Pow, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Pow, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(pumpPow, OUTPUT);
  pinMode(pumpDir, OUTPUT);
  pinMode(ultraTrigPin, OUTPUT);
  pinMode(ultraEchoPin, INPUT); 
  pinMode(conductPin, INPUT);
  pinMode(float1Pin, INPUT);
  pinMode(float2Pin, INPUT);
  pinMode(float3Pin, INPUT);
  pinMode(photoPin, INPUT);
  //Set up servo pin
  myservo.attach(servoPin);

  // Team Name, Mission Type, Marker ID, TX Pin, RX Pin
  Enes100.begin("Elephante", WATER, 13 , wifiTXPin, wifiRXPin);
  Enes100.print("Destination is at (");
  Enes100.print(missionX);
  Enes100.print(", ");
  Enes100.print(missionY);
  Enes100.println(")");

  //bring arm up


}

void loop() {
  forward();
  delay(500);
  stopMotors();
}

void straight(float x_dest, float y_dest){
  Enes100.updateLocation(); //get location
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  float tht = Enes100.location.theta;
  float desired_tht = atan2((y_dest-y),(x_dest-x)); //angle we want to go
  Enes100.println(desired_tht);
  if (abs(desired_tht - tht) > tht_tol){
    turn(desired_tht);
  }
  forward(); //go forward for 1/2 second
  delay(500);
  stopMotors();
  Enes100.updateLocation(); //get location
  x = Enes100.location.x;
  y = Enes100.location.y;
  tht = Enes100.location.theta;
  if (abs(desired_tht-tht) > tht_tol){
    turn(desired_tht); // readjust towards target
  }
}

void turn(int desired_tht){
  Enes100.updateLocation(); //get location
  float tht = Enes100.location.theta;
  while(abs(desired_tht-tht) > tht_tol){ //while angle is far away
    float diff_tht = desired_tht-tht;
    if (desired_tht> tht){
      if(diff_tht >= pi) 
        turnRight(int((2*pi-diff_tht)/turnTimeTol)); //turn a smaller amount if difference is greater (closer, just on other side of discontinuity)
      else if(diff_tht < pi)
        turnLeft(int(diff_tht/turnTimeTol)); //turn a larger amount if difference is greater (farther)
    }
    else{
      if(diff_tht <= pi) 
        turnRight(int(diff_tht/turnTimeTol)); //turn a larger amount if difference is greater (farther)
      else if(diff_tht > pi)
        turnLeft(int((2*pi-diff_tht)/turnTimeTol)); //turn a smaller amount if difference is greater (closer, just on other side of discontinuity)
    }
  Enes100.updateLocation(); //get location
  tht = Enes100.location.theta;  
  }
}

void turnRight(int time){
  //Left wheels CCW, Right wheels CW
  motor1(1);
  motor2(0);
  delay(50*time);
  stopMotors();
}

void turnLeft(int time){
  //Left wheels CW, Right wheels CCW
  motor1(0);
  motor2(1);
  delay(50*time);
  stopMotors();
}

void motor1(int dir){ //0 is CW, 1 is CCW
  if(dir == 0)
    Tank.setLeftMotorPWM(pwm);
  else
    Tank.setLeftMotorPWM(-1*pwm);

}

void motor2(int dir){ //0 is CW, 1 is CCW
  if(dir == 1)
    Tank.setRightMotorPWM(pwm);
  else
    Tank.setRightMotorPWM(-1*pwm);
}

void stopMotors(){
  Tank.turnOffMotors();
}

void forward(){
  motor1(1); //turns both motors CCW
  motor2(1);
}

void reverse(){
  motor1(0); //turns both motors CW
  motor2(0);
}

float getDist(){
  return Tank.readDistanceSensor(1)*float(100);
}

float calcDist(float x1, float x2, float y1, float y2){
  //Enes100.println("X distance squared");
  //Enes100.println(pow((x1-x2),2));
  //Enes100.println("Y distance squared");
  //Enes100.println(pow((y1-y2),2));
  Enes100.println(pow((x1-x2),2)+pow((y1-y2),2));
  return float(sqrt(pow((x1-x2),2)+pow((y1-y2),2)))*float(100);
}

bool getCond(){
  float val = analogRead(conductPin);
  if (val > condTol){
    return true; //
  }
}

void mission(){
  // put your main code here, to run repeatedly:
  Enes100.updateLocation(); //get location
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  float tht = Enes100.location.theta;
  //Go to mission

  while(getDist() > 10 && calcDist(x, missionX, y, missionY) > 10 ){
  Enes100.print("entered while loop ");
  straight(missionX, missionY);
    Enes100.updateLocation();
    x = Enes100.location.x;
    y = Enes100.location.y;    
  }
  stopMotors();
  //Mission Code

  //Traverse Obstacles

  //Go under limbo

  //Celebration



}
/*
float getTheta(){
  float tht = Enes100.location.theta;
  if (tht < 0)
    tht += 2*pi;
  delay(50);
  return tht;
}


float* getPos(){
  Enes100.updateLocation(); //get location
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  float tht = Enes100.location.theta;
  float arr [3] = {x,y,tht}; // create array with location and angle
  return arr;
}
*/
