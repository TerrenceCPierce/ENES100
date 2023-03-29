#include "Enes100.h"
#include <Servo.h>
#include "Tank.h"

//use a define to get shortcut to enes100 position x

//global 
float const tht_tol = .05;
float const pi = 3.1415;
float const turnTimeTol = 1;
float const condTol = 2;
float const distTol = 15;

float const pwm = 255;
int const straightTol = 250; //ms per stop when going straight
float const obsHeight = .45; //meters
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

int const aruco = 12;
//will need to add LED to diagram and code

float missionX = .5; 
float missionY = 1.5;

float obs1X = 1.5;

float obs2X = 2.6;

float limboX = 3.3;
float limboY = 1.5;

float finalX = 3.6;
float finalY = 1.5;

Servo myservo; // initializes servo object

void setup() {
  // put your setup code here, to run once:
  //pin setup
  
  Tank.begin();
  /*
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
  */

  // Team Name, Mission Type, Marker ID, TX Pin, RX Pin
  Enes100.begin("Elephante", WATER, aruco , wifiTXPin, wifiRXPin);
  Enes100.print("Destination is at (");
  Enes100.print(missionX);
  Enes100.print(", ");
  Enes100.print(missionY);
  Enes100.println(")");

  //bring arm up


}

void loop() {
  //float USDist = getDist();
  //Enes100.println(USDist);


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
  forward(); 
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

void straight(float desired_tht){
  Enes100.updateLocation(); //get location
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  float tht = Enes100.location.theta;
  if (abs(desired_tht - tht) > tht_tol){
    turn(desired_tht);
  }
  forward(); 
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

void turn(float desired_tht){
  Enes100.updateLocation(); //get location
  float tht = Enes100.location.theta;
  while(abs(desired_tht-tht) > tht_tol){ //while angle is far away

    float diff_tht = desired_tht-tht;
    if (desired_tht> tht){
      if(diff_tht >= pi){
        //Enes100.println("Case 1");
        turnRight(); //turn a smaller amount if difference is greater (closer, just on other side of discontinuity)
    }
      else if(diff_tht < pi){
        //Enes100.println("Case 2");
        turnLeft(); //turn a larger amount if difference is greater (farther)
      }
    }
    else{
      if(diff_tht <= pi){ 
        //Enes100.println("Case 3");
        turnRight(); //turn a larger amount if difference is greater (farther)

      }
      else if(diff_tht > pi){
        //Enes100.println("Case 4");
        turnLeft(); //turn a smaller amount if difference is greater (closer, just on other side of discontinuity)

      }
    }

  
  Enes100.updateLocation(); //get location
  tht = Enes100.location.theta;
  //delay(100);
  stopMotors();
  delay(200);
  //Enes100.println(tht);
  }
  stopMotors();
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
  delay(50*time);
  stopMotors();
  delay(100);
}

void turnLeft(){
  motor1(0);
  motor2(1);
}

void motor1(int dir){ //0 is CW, 1 is CCW
  if(dir == 1)
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
  Tank.setLeftMotorPWM(0);
  Tank.setRightMotorPWM(0);
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
  //Enes100.println(pow((x1-x2),2)+pow((y1-y2),2));
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

  if(y<1){
    missionY = 1.5;
  }
  else{
    missionY = 0.5;    
  }

  while(getDist() > distTol && calcDist(x, missionX, y, missionY) > distTol ){
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

  while(x < obs1X){
    turn(0);
    float USDist = getDist();
    while(USDist > distTol){
      Enes100.updateLocation();
      x = Enes100.location.x;
      y = Enes100.location.y;
      straight(0);
      USDist = getDist();
      Enes100.println(USDist);
      if(USDist < distTol){
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
    if(x < obs1X){
      //Enes100.println("Going down an obstacle height to " + String(y-obsHeight)+  "from" + String(y));
      float y1 = y;
      while(calcDist(x,x,y,y1-obsHeight)>3){
        straight(des_tht);
        Enes100.updateLocation();
        x = Enes100.location.x;
        y = Enes100.location.y;
        tht = Enes100.location.theta;
      }
      if(y <= missionY-1.8*obsHeight){ //last row change this to accomodate both starting positions
        straight(obs1X + .1, y);
      }
    }
  }

  Enes100.updateLocation();
  x = Enes100.location.x;
  y = Enes100.location.y;
  tht = Enes100.location.theta;

  while(x < obs2X){
    Enes100.println("Entered <3 while loop");
    float USDist = getDist();
    while(USDist > distTol & x < obs2X){
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
    if(x < obs2X){ //stopped because it detected an object
      //Enes100.println("Going down an obstacle height to " + String(y-obsHeight)+  "from" + String(y));
      if(obsCoord[0] > obs1X){ // turn in same direction as previous, no detected objects previously
        //Enes100.println("Entered <3 while loop if statement");
        float y1 = y;
        while(calcDist(x,x,y,y1-obsHeight)>obs2X){
          straight(x,y1-obsHeight);
          Enes100.updateLocation();
          x = Enes100.location.x;
          y = Enes100.location.y;
          tht = Enes100.location.theta;
        }
      }
      else{ // turn in opposite direction
        //Enes100.println("Entered <3 while loop else statement");
        float y1 = y;
        while(calcDist(x,x,y,y1+obsHeight)>obs2X){
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

  while(getDist() > distTol && calcDist(x, limboX, y, limboY) > distTol ){
  straight(limboX, limboY);
    Enes100.updateLocation();
    x = Enes100.location.x;
    y = Enes100.location.y;    
  }
  stopMotors();
}

void limbo(){
  turn(0);
  Enes100.updateLocation(); //get location
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  while(getDist() > distTol && calcDist(x, finalX, y, finalY) > distTol){
    forward(); 
    delay(straightTol);
    stopMotors();
    Enes100.updateLocation();
    x = Enes100.location.x;
    y = Enes100.location.y; 
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
