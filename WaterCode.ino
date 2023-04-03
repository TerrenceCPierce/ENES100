#include "Enes100.h"
#include <Servo.h>

//global 
float const tht_tol = .1;
float const pi = 3.1415;
float const turnTimeTol = .1;
float const condTol = 2;
float const pwm = 255;
int const straightTol = 250; //ms per stop when going straight
float const obsHeight = .45; //meters
//Digital pins
int const wifiTXPin = 2; //wifi module pins
int const wifiRXPin = 3;
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

int const aruco = 219;
//will need to add LED to diagram and code

float missionX = .5; 
float missionY = 1.5;

float limboX = 3.3;
float limboY = 1.5;

Servo myservo; // initializes servo object

void setup() {
  // put your setup code here, to run once:
  //pin setup
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
  Enes100.begin("Elephante", WATER, aruco, wifiTXPin, wifiRXPin);
  Enes100.print("Destination is at (");
  Enes100.print(missionX);
  Enes100.print(", ");
  Enes100.print(missionY);
  Enes100.println(")");

  //bring arm up
}

void loop() {
  // put your main code here, to run repeatedly:
    Enes100.updateLocation(); //get location
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  float tht = Enes100.location.theta;
  //Go to mission
  while(getDist() > 10 && calcDist(x, Enes100.missionSite.x, y, Enes100.missionSite.y) > 10 ){
  straight(Enes100.missionSite.x, Enes100.missionSite.y);
    Enes100.updateLocation();
    x = Enes100.location.x;
    y = Enes100.location.y;
  }
  stopMotors();

  go2mission();
  delay(250);

  //Mission Code
  mission();

  //Traverse Obstacles
  obstacles();

  //Go to limbo
  go2limbo();

  //Go under limbo
  limbo();

  //Celebration
  Enes100.println("Finished Loop");
  while(1){}

}

void straight(float x_dest, float y_dest){
  Enes100.updateLocation(); //get location
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  float tht = Enes100.location.theta;
  float desired_tht = atan2((y_dest-y),(x_dest-x)); //angle we want to go
  //Enes100.println("Desired Theta: " + String(desired_tht));
  if (abs(desired_tht - tht) > tht_tol){
    turn(desired_tht);
  }
  forward(); //go forward for 1/2 second
  delay(straightTol);
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
        //Enes100.println("Case 1");
        turnRight(int((2*pi-diff_tht)/turnTimeTol)); //turn a smaller amount if difference is greater (closer, just on other side of discontinuity)
        
      else if(diff_tht < pi)
        //Enes100.println("Case 2");
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

void turnRight(float time){
  //Left wheels CCW, Right wheels CW
  motor1(1);
  motor2(0);
  delay(100*time);
  stopMotors();
  delay(100);
}

void turnRight(){
  motor1(1);
  motor2(0);

}

void turnLeft(float time){
  //Left wheels CW, Right wheels CCW
  motor1(0);
  motor2(1);
  delay(100*time);
  stopMotors();
  delay(100)
}

void turnLeft(){
  motor1(0);
  motor2(1);
}

void motor1(int dir){ //0 is CW, 1 is CCW
  digitalWrite(motor1Pow,HIGH);
  if(dir == 0)
    digitalWrite(motor1Dir,HIGH);
  else
    digitalWrite(motor1Dir,LOW);

}

void motor2(int dir){ //0 is CW, 1 is CCW
  digitalWrite(motor2Pow,HIGH);
  if(dir == 0)
    digitalWrite(motor2Dir,HIGH);
  else
    digitalWrite(motor2Dir,LOW);
}

void stopMotors(){
  digitalWrite(motor1Pow,LOW); //turns the power off to both motors
  digitalWrite(motor2Pow,LOW);
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
  //code from https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
  digitalWrite(ultraTrigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(ultraTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultraTrigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float duration = pulseIn(ultraEchoPin, HIGH);
  // Calculating the distance
  float distance = duration * 0.034 / 2;
  return distance;
}

float calcDist(float x1, float x2, float y1, float y2){
  return float(sqrt(pow((x1-x2),2)+pow((y1-y2),2)))*float(100);
}

bool getCond(){
  float val = analogRead(conductPin);
  if (val > condTol){
    return true; //
  }
}

void go2mission(){
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
}

void mission(){
  //do stuff
}

void obstacles(){
  turn(0);
  Enes100.updateLocation();
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  float tht = Enes100.location.theta;
  float obsCoord[2] = {0,0};
  float des_tht; 

  if(missionY > 1)    
    des_tht = -1*pi/2; //towards middle
  else
    des_tht = pi/2;

  while(x < 1.5){
    turn(0);
    float USDist = getDist();
    while(USDist > 10){
      Enes100.updateLocation();
      x = Enes100.location.x;
      y = Enes100.location.y;
      straight(x+.1,y);
      USDist = getDist();
      Enes100.println(USDist);
      if(USDist < 10){
        Enes100.updateLocation();
        x = Enes100.location.x;
        y = Enes100.location.y;
        obsCoord[0] = x;
        obsCoord[1] = y;
        Enes100.println("Detected Obstacle");
      }
    }
    stopMotors();
    Enes100.updateLocation();
    x = Enes100.location.x;
    y = Enes100.location.y;
    tht = Enes100.location.theta;
    if(x < 1.5){

      //Enes100.println("Going down an obstacle height to " + String(y-obsHeight)+  "from" + String(y));
      float y1 = y;
      while(calcDist(x,x,y,y1-obsHeight)>3){
        straight(x,y1-obsHeight);
        Enes100.updateLocation();
        x = Enes100.location.x;
        y = Enes100.location.y;
        tht = Enes100.location.theta;
      }
      if(y <= missionY-1.8*obsHeight){ //last row change this to accomodate both starting positions
        straight(1.6, y);
      }
    }
  }

  Enes100.updateLocation();
  x = Enes100.location.x;
  y = Enes100.location.y;
  tht = Enes100.location.theta;

  while(x < 3){
    Enes100.println("Entered <3 while loop");
    float USDist = getDist();
    while(USDist > 10 & x < 3){
      Enes100.updateLocation();
      x = Enes100.location.x;
      y = Enes100.location.y;
      straight(x+.1,y);
      USDist = getDist();
      Enes100.println(USDist);
    }
    stopMotors();
    Enes100.updateLocation();
    x = Enes100.location.x;
    y = Enes100.location.y;
    tht = Enes100.location.theta;
    if(x < 3){ //stopped because it detected an object
      //Enes100.println("Going down an obstacle height to " + String(y-obsHeight)+  "from" + String(y));
      if(obsCoord[0] > 1.5){ // turn in same direction as previous, no detected objects previously
        Enes100.println("Entered <3 while loop if statement");
        float y1 = y;
        while(calcDist(x,x,y,y1-obsHeight)>3){
          straight(x,y1-obsHeight);
          Enes100.updateLocation();
          x = Enes100.location.x;
          y = Enes100.location.y;
          tht = Enes100.location.theta;
        }
      }
      else{ // turn in opposite direction
        Enes100.println("Entered <3 while loop else statement");
        float y1 = y;
        while(calcDist(x,x,y,y1+obsHeight)>3){
          straight(x,y1+obsHeight);
          Enes100.updateLocation();
          x = Enes100.location.x;
          y = Enes100.location.y;
          tht = Enes100.location.theta;
        }
      }
    }
  }
}

void go2limbo(){
  // put your main code here, to run repeatedly:
  Enes100.updateLocation(); //get location
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  float tht = Enes100.location.theta;
  //Go to limbo

  while(getDist() > 10 && calcDist(x, limboX, y, limboY) > 10 ){
  straight(limboX, limboY);
    Enes100.updateLocation();
    x = Enes100.location.x;
    y = Enes100.location.y;    
  }
  stopMotors();
}

void limbo(){
  turn(0);
  while(getDist() > 10){
    forward(); 
    delay(straightTol);
    stopMotors();
  }
  stopMotors();
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
