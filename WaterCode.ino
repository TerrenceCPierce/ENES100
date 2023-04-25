#include "Enes100.h"
#include <Servo.h>

//use a define to get shortcut to enes100 position x
#define getLoc Enes100.updateLocation()
#define xLoc Enes100.location.x
#define yLoc Enes100.location.y
#define thtLoc Enes100.location.theta

float x;
float y;
float tht;

//global 
float const tht_tol = .1;
float const pi = 3.1415;
float const turnTimeTol = 1;
float const analogTol = 2000;
float const distTol = 10;
float const destTol = 3;
float const photoTol = 60; //Tested on 4/24
float const condTol = 350; //Salt measured at 434 with correct proportions
float const waterCondTol = 550; //Measured at 608-638

float const pwm = 255;
int const straightTol = 250; //ms per stop when going straight
float const obsHeight = .45; //meters
//Digital pins
int const wifiTXPin = 2; //wifi module pins
int const wifiRXPin = 3;
int const motor1In1 = 5; //motors pins
int const motor1In2 = 4;
int const motor2In1 = 7;
int const motor2In2 = 6;
int const pumpIn1 = 8; // Pump pins
int const pumpIn2 = 9;
int const servoPin = 10;
int const ultraTrigPin = 11; //Ultrasonic pins
int const ultraEchoPin = 12;
//Analog pins
int const conductPin = A0;
int const float1Pin = A1;
int const float2Pin = A2;
int const photoPin = A4;

int const aruco = 12;
//will need to add LED to diagram and code

float missionX = .42; 
float missionY = 1.4;

float missionYHigh = 1.4;
float missionYLow = .4;

float startObsX = .6;
float startObsY = 1.6;

float obs1X = 1.5;

float obs2X = 2.6;

float limboX = 3.2;
float limboY = 1.5;

float finalX = 3.4;
float finalY = 1.5;

Servo myservo; // initializes servo object

void setup() {
  // put your setup code here, to run once:
  //pin setup
  pinMode(motor1In1, OUTPUT);
  pinMode(motor1In2, OUTPUT);
  pinMode(motor2In1, OUTPUT);
  pinMode(motor1In2, OUTPUT);
  pinMode(pumpIn1, OUTPUT);
  pinMode(pumpIn2, OUTPUT);
  pinMode(ultraTrigPin, OUTPUT);
  pinMode(ultraEchoPin, INPUT); 
  pinMode(conductPin, INPUT);
  pinMode(float1Pin, INPUT);
  pinMode(float2Pin, INPUT);
  pinMode(photoPin, INPUT);
  //Set up servo pin
  myservo.attach(servoPin);
  delay(500);
  // Team Name, Mission Type, Marker ID, TX Pin, RX Pin
  Enes100.begin("Elephante", WATER, aruco , wifiTXPin, wifiRXPin);
  delay(500);
  // Team Name, Mission Type, Marker ID, TX Pin, RX Pin
  Enes100.begin("Elephante", WATER, aruco , wifiTXPin, wifiRXPin);
  delay(500);
  // Team Name, Mission Type, Marker ID, TX Pin, RX Pin
  Enes100.begin("Elephante", WATER, aruco , wifiTXPin, wifiRXPin);
  delay(500);
  // Team Name, Mission Type, Marker ID, TX Pin, RX Pin
  Enes100.begin("Elephante", WATER, aruco , wifiTXPin, wifiRXPin);


  //bring arm up
  myservo.write(0);
  delay(3000);
  setServo(89);

}

void loop() {
  mainCodeNoPump();
  //ultraTestEnes100();
  while(1){}
}

void mainCode(){
  delay(1000);
  updateLoc(); //updates location a few times to get rid of incorrect values at start
  updateLoc();
  updateLoc();
  delay(3000);
  
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

void mainCodeNoPump(){
  delay(1000);
  updateLoc(); //updates location a few times to get rid of incorrect values at start
  updateLoc();
  updateLoc();
  delay(3000);
  
  go2mission();
  delay(250);
  //Mission Code without pumping
  missionOnlyNoPumpTest();
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

void servoTest(){
  setServo(89);
  delay(2000);
  setServo(45);
  delay(2000);
  setServo(1);
  delay(1000);
  setServo(45);
}

void ultraTest(){
  Serial.begin(9600);
  while(1){
    Serial.println(getDist());
    delay(500);
  }
}

void ultraTestEnes100(){
  while(1){
    Enes100.println(getDist());
    delay(500);
  }
}

void turnTest(){
  delay(1000);
  turn(0);
  delay(1000);
  turn(-1*pi/2);
  delay(1000);
  turn(pi/2);
  delay(1000);
  turn(pi);  
}

void missionTest(){
  delay(1000);
  updateLoc(); //updates location a few times to get rid of incorrect values at start
  updateLoc();
  updateLoc();
  go2mission();
  delay(1000);
  setServo(0);
  delay(3000);
  pump(0);
  delay(15000);
  if (isSalt() == 1){
    Enes100.println("Salt Water");
  }
  else if (isSalt() == 0){
    Enes100.println("Fresh Water");
  }
  else{
    Enes100.println("Failure to detect");    
  }
  delay(2000);
  if (isPolluted() == 1){
    Enes100.println("Polluted Water");
  }
  else if (isPolluted() == 0){
    Enes100.println("Unpolluted Water");
  }
  else{
    Enes100.println("Failure to detect");
  }
  delay(2000);
  pumpOff();

}

void missionOnlyTest(){
  delay(1000);
  setServo(0);
  delay(3000);
  pump(1);
  delay(15000);
  if (isSalt() == 1){
    Enes100.println("Salt Water");
  }
  else if (isSalt() == 0){
    Enes100.println("Fresh Water");
  }
  else{
    Enes100.println("Failure to detect");    
  }
  delay(2000);
  if (isPolluted() == 1){
    Enes100.println("Polluted Water");
  }
  else if (isPolluted() == 0){
    Enes100.println("Unpolluted Water");
  }
  else{
    Enes100.println("Failure to detect");
  }
  delay(2000);
  pumpOff();
}

void missionOnlyNoPumpTest(){
  delay(1000);
  setServo(0);
  delay(3000);
  if (isSalt() == 1){
    Enes100.println("Salt Water");
  }
  else if (isSalt() == 0){
    Enes100.println("Fresh Water");
  }
  else{
    Enes100.println("Failure to detect");    
  }
  delay(2000);
}

void pumpTest(){
  setServo(1);
  delay(1000);
  pump(0);
  delay(10000);
  pumpOff();
  delay(1000);
  pump(1);
  delay(10000);
  pumpOff();
}

void manualPumpTest(){
  setServo(1);
  digitalWrite(8,HIGH);
  digitalWrite(9,LOW);
  delay(5000);
  digitalWrite(8,LOW);
  digitalWrite(9,LOW);
}

void saltTolTest(){
  //Serial.begin(9600);
  while(1){
    float val = analogRead(conductPin);
    Enes100.println(val);    
    delay(500);
  }
}

//overloaded straight function that tells OSV to go in direction of x_dest and y_dest
void straight(float x_dest, float y_dest){ 
  updateLoc(); //get Location and update values
  float desired_tht = atan2((y_dest-y),(x_dest-x)); //angle we want to go
  //Enes100.println("Desired Theta: " + String(desired_tht));
  //Enes100.println("Y Destination" + String(y_dest));
  
  if (abs(desired_tht - tht) > tht_tol){ //makes sure vehicle is pointing in the right direction
    turn(desired_tht);
  }

  //goes forward a certain amount after adjusting angle
  forward(); 
  delay(straightTol);
  stopMotors();
  updateLoc(); //get Location and update values

  if (abs(desired_tht-tht) > tht_tol){
    turn(desired_tht); // readjust angle towards target
  }
}

//overloaded straight function that tells OSV to go in direction of desired_tht
void straight(float desired_tht){ 
  updateLoc(); //get Location and update values
  if (abs(desired_tht - tht) > tht_tol){//makes sure vehicle is pointing in the right direction
    turn(desired_tht);
  }

  //goes forward a certain amount after adjusting angle
  forward(); 
  delay(straightTol);
  stopMotors();
  updateLoc(); //get location and update values

  if (abs(desired_tht-tht) > tht_tol){ //makes sure vehicle is pointing in the right direction
    turn(desired_tht); // readjust towards target
  }
}

void turn(float desired_tht){
  updateLoc(); //get location and update values
  while(abs(desired_tht-tht) > tht_tol){ //while angle is far away
    float diff_tht = desired_tht-tht;

    //Series of conditions which determine shortest path to turn
    if (desired_tht > tht){ 
      if(diff_tht >= pi){
        turnRight(); 
      }
      else if(diff_tht < pi){
        turnLeft(); 
      }
    }
    else{
      if(diff_tht <= pi){ 
        turnRight();

      }
      else if(diff_tht > pi){
        turnLeft();
      }
    }
  
  updateLoc(); //get location and update values
  delay(10);
  stopMotors(); //stops to check conditions
  delay(200);
  //Enes100.println(tht);
  }
  // angle is now within tolerance
  stopMotors();
}

void turnRight(){
  motor1(1); //turn left motor CCW (from outside of wheel) 
  motor2(0); // turn right motors CW (from outside of wheel) 

}

void turnLeft(){
  motor1(0); //turn left motor CW (from outside of wheel) 
  motor2(1); // turn right motors CCW (from outside of wheel) 
}

void motor1(int dir){ //0 is forward, 1 is back
  if(dir == 0){
    digitalWrite(motor1In1,HIGH);
    digitalWrite(motor1In2,LOW);
  }
  else{
    digitalWrite(motor1In1,LOW);
    digitalWrite(motor1In2,HIGH);
  }

}

void motor2(int dir){ //0 is forward, 1 is back
  if(dir == 0){
    digitalWrite(motor2In1,HIGH);
    digitalWrite(motor2In2,LOW);
  }
  else{
    digitalWrite(motor2In1,LOW);
    digitalWrite(motor2In2,HIGH);
  }
}

void stopMotors(){ //turna power off for both motors
  digitalWrite(motor1In1,LOW);
  digitalWrite(motor1In2,LOW);
  digitalWrite(motor2In1,LOW);
  digitalWrite(motor2In2,LOW);
}

void forward(){
  motor1(1); //turns both motors CW (from outside of wheel) 
  motor2(1);
}

void reverse(){
  motor1(0); //turns both motors CCW (from outside of wheel) 
  motor2(0);
}

//gets distance from ultrasonic sensor in cm
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
//calculates distance between two points in cm
float calcDist(float x1, float x2, float y1, float y2){
  return float(sqrt(pow((x1-x2),2)+pow((y1-y2),2)))*float(100);
}

//gets location and updates position variables
void updateLoc(){
  getLoc;
  x = xLoc;
  y = yLoc;
  tht = thtLoc;
}

//goes to the mission site
void go2mission(){
  setServo(90);
  delay(100);
  getLoc; //get location
  delay(100);
  x = xLoc;
  y = yLoc; //getting wrong y value, ~.55 when starting at 1.5
  tht = thtLoc;
  Enes100.println("Y val:" + String(y));
  updateLoc(); //trying to get location again in case this location is correct
  Enes100.println("Y val:" + String(y));
  
  //find correct mission Y based on what y is
  if(y < 1){
    missionY = missionYHigh;
  }
  else if(y > 1){
    missionY = missionYLow;    
  }
    Enes100.println("Mission Y val:" + String(missionY));
  //while the ultrasonic sensor does not detect anything within tolerance and while the calculated distance is far enough away, keep going forward
  while(getDist() > distTol && calcDist(x, missionX, y, missionY) > destTol ){
  //Enes100.print("entered while loop ");
  //Enes100.print("Mission Y:" + String(missionY));
  straight(missionX, missionY);
  updateLoc();
  }
  
  stopMotors();
}

void avoidTank(){
  setServo(89); //bring arm up
  //gets out of way of tank
  reverse();
  delay(500);
  turn(0);
  updateLoc();
  while(calcDist(x, startObsX, y, y) > destTol ){
    straight(startObsX, y);
    updateLoc();
  }

  while(calcDist(x, x, y, startObsY) > destTol ){
    straight(x, startObsX);
    updateLoc();
  }

  
}

//navigate OSV through obstacles
void obstacles(){
  avoidTank();

  delay(1000);
  turn(0); //turn to face right
  updateLoc(); //get location and update values
  float obsCoord[2] = {0,0}; //initialize obstacle Coordinates to start as x=0 and y=0
  float des_tht; //initialize desired theta for turning towards middle

  if(missionY > 1)    
    des_tht = -1*pi/2; //towards middle
  else
    des_tht = pi/2; //towards middle

  //Code for making it past first column of obstacles
  while(x < obs1X){
    turn(0); //turn to face right

    //while no obstacle is detected
    while(getDist() > distTol && calcDist(x,y,obs2X,y)>destTol){
      updateLoc(); //get location and update values
      straight(0); // go straight to the right

      if(getDist() < distTol){ //if an obstacle is detected
        updateLoc(); //get location and update values

        if(x < obs1X){ //if the obstacle detected is in the first column
          obsCoord[0] = x; //save x coordinate of obstacle
          obsCoord[1] = y; //save y coordinate of obstacle
          Enes100.println("Detected Obstacle");
          Enes100.println(String(obsCoord[0]));
          Enes100.println(String(obsCoord[1]));
        }
      }
    }
    Enes100.println("Distance less than tol");
    //once an obstacle is detected, stop
    stopMotors();
    updateLoc(); //get location and update values

    if(x < obs1X){ //if OSV is still in the first row of obstacles
      float y1 = y;

      //while distance to middle of next obstacle is far, keep moving
      while(calcDist(x,x,y,y1+(obsHeight*des_tht/(pi/2)))>destTol){
        straight(des_tht);
        updateLoc(); //get location and update values
      }

      //if made it to the last row, just move right
      if(y <= missionY+1.8*(obsHeight*des_tht/(pi/2))){ //last row
        while(calcDist(x,obs1X+.3,y,y)>destTol){
          straight(0);
        }
      }
    }
  }

  updateLoc(); //get location and update values
  
  //Code for making it past second column of obstacles
  while(x < obs2X){
    Enes100.println("At second column of obstacles");
    
    //while no obstacles are detected and while not past the second column of obstacles
    while(getDist() > distTol & x < obs2X +.3){
      updateLoc(); //get location and update values
      straight(0); //move right
    }

    //once an obstacle is detected or past the second column, stop
    stopMotors();
    updateLoc(); //get location and update values

    if(x < obs2X){ //stopped because it detected an object
      if(obsCoord[0] == 0){ // turn in same direction as previous, no detected objects previously because obsCoord is still default value
        float y1 = y;

        //because no previous obstacles, basically treat this as first column with no known information
        //while the OSV is far away from the next row of obstacles, move in that direction
        while(calcDist(x,x,y,y1+(obsHeight*des_tht/(pi/2)))>destTol){
          straight(des_tht); //move in desired direction
          updateLoc(); //get location and update values
        }
      }

      //If an obstacle was detected in the first column, since two obstacles can't be in the same row, turn in that direction
      else if(obsCoord[0] <= obs1X){ //detected obstacle in 1st column
        float y1 = y;
        //while far away from destination, move towards destination
        while(calcDist(x,x,y,obsCoord[1])>destTol){
          straight(x,obsCoord[1]); //move straight towards current x value, but y value of previous obstacle
          updateLoc(); //get location and update values
        }

      }
      else{ // this condition should never happen, if it does, something went wrong
      /*
        //Enes100.println("Entered <3 while loop else statement");
        float y1 = y;
        while(calcDist(x,x,y,y1+obsHeight)>distTol){
          straight(x,y1+obsHeight);
          getLoc;
          x = xLoc;
          y = yLoc;
          tht = thtLoc;
        }
        */
        Enes100.println("Entered 2nd column else statement, conditions went wrong, Exiting----");
        while(1){}
      }
    }
  }
}

//navigate OSV to limbo
void go2limbo(){
  updateLoc(); //get location and update values
  //Go to limbo

  //While far away from limbo and no obstacles detected, move towards limbo entrance
  while(calcDist(x, limboX, y, limboY) > destTol ){
  straight(limboX, limboY);
    updateLoc(); //get location and update values 
  }
  //stop once arrived at limbo
  stopMotors();
}

//go under limbo
void limbo(){
  setServo(0); //bring arm down to go through limbo
  delay(1000);
  turn(0); //turn right
  updateLoc(); //get location and update values

  //move forward to the right while distance sensor does not detect the wall or while the x and y values are far from final location
  //this uses the vision system as little as possible since the limbo blocks the camera from getting position values for a small amount of time
  while(calcDist(x, finalX, y, finalY) > destTol){
    forward(); 
    delay(straightTol);
    stopMotors();
    updateLoc(); //get location and update values
  }

  //reached the end, stop the motors
  stopMotors();
}

void pump(float time, int dir){ //0 dir is in, 1 dir is out
  if(dir == 1){
    digitalWrite(pumpIn1,HIGH);
    digitalWrite(pumpIn2,LOW);
  }
  else{
    digitalWrite(pumpIn1,LOW);
    digitalWrite(pumpIn2,HIGH);
  }
  delay(time);
  digitalWrite(pumpIn1,LOW);
  digitalWrite(pumpIn2,LOW);
}

void pump(int dir){ //0 dir is in, 1 dir is out
  if(dir == 1){
    digitalWrite(pumpIn1,HIGH);
    digitalWrite(pumpIn2,LOW);
  }
  else{
    digitalWrite(pumpIn1,LOW);
    digitalWrite(pumpIn2,HIGH);
  }
}

void pumpOff(){
  digitalWrite(pumpIn1,LOW);
  digitalWrite(pumpIn2,LOW);
}

void setServo(float finAng){
  myservo.attach(servoPin);
  float ang = myservo.read();
  finAng = abs(90-finAng);
  //ang = abs(90-ang);
  if(ang < finAng){
    for (ang; ang <= finAng; ang += 1) { // goes from 0 degrees to 90 degrees
      // in steps of 1 degree
      myservo.write(ang);              // tell servo to go to position in variable 'ang
      delay(75);                       // waits 15ms for the servo to reach the position
    }
  }
  else if(ang >= finAng){
    for (ang; ang >= finAng; ang -= 1) { // goes from 90 degrees to 0 degrees
      // in steps of 1 degree
      myservo.write(ang);              // tell servo to go to position in variable 'ang
      delay(75);                       // waits 15ms for the servo to reach the position
    }
  }
  delay(1000);
  myservo.detach();
}

//returns if the water has salt or not
int isSalt(){
  float val = analogRead(conductPin);
  Enes100.println(val);
  if (val > waterCondTol){
    return 0; //
  }
  else if(val > condTol){
    return 1;
  }
  else{
    return -1;
  }
}

int getLevel(){
  float read30 = analogRead(float1Pin);
  float read40 = analogRead(float2Pin);

  if (read30 > analogTol && read40 > analogTol){
    return 40;
  }

  else if (read30 > analogTol && read40 < analogTol){
    return 30;
  }

  else if(read30 < analogTol && read40 < analogTol){
    return 20;
  }
  else{
    return -1;
  }
}

bool isPolluted(){
  float val = analogRead(photoPin);
  Enes100.println(val);
  if (val > photoTol){
    return true; //
  }
  else{
    return false;
  }
}

void mission(){
  setServo(0);
  delay(1000);
  int salt = isSalt();
  /*
  if (salt == -1){
    Enes100.println("Missed water");
    setServo(90);
    go2mission();
    mission();
  }
  */
  delay(1000);
  int lev = getLevel();
  if(lev == -1){
    Enes100.println("Not detecting water level");
  }
  delay(1000);
  pump(0);
  delay(10000);
  bool pol = isPolluted();

  delay(20000);
  pumpOff();

  Enes100.mission(DEPTH, lev);
  int waterType = salt*pow(2,0)+pol*pow(2, 1); //0 for fresh, unpolluted, 1 for salt, unpolluted, 2 for fresh polluted, 3 for salt unpolluted
  switch(waterType){
    case 0:
      Enes100.mission(WATER_TYPE, FRESH_UNPOLLUTED);
      break;
    case 1:
      Enes100.mission(WATER_TYPE, SALT_UNPOLLUTED);
      break;
    case 2:
      Enes100.mission(WATER_TYPE, FRESH_POLLUTED);
      break;
    case 3:
      Enes100.mission(WATER_TYPE, SALT_POLLUTED);
      break;
    default:
      Enes100.println("Binary statement wrong");
      break;
  }
  
}
