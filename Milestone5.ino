#include "Enes100.h"
#include <Servo.h>
#include "Tank.h"

//use a define to get shortcut to enes100 position x
#define getLoc Enes100.updateLocation()
#define xLoc Enes100.location.x
#define yLoc Enes100.location.y
#define thtLoc Enes100.location.theta

float x;
float y;
float tht;

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

int const aruco = 11;
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
  Enes100.begin("Elephante", WATER, aruco , wifiTXPin, wifiRXPin);


  //bring arm up
}

void loop() {
  // put your main code here, to run repeatedly:

  //go straight from top left to top right
  updateLoc();
  while(calcDist(x, finalX, y, finalY) > distTol){
    straight(0);
    delay(straightTol);
    stopMotors();
    updateLoc(); //get location and update values
  }

  //turn 4 times

  turn(-1*pi/2);
  delay(1000);
  turn(pi);
  delay(1000);
  turn(pi/2);
  delay(1000);
  turn(0);


  delay(5000);
  //pump water in for 10 seconds
  pump(10000,0);

  while(1){}


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
  //delay(100);
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

//returns if the water has salt or not
bool getCond(){
  float val = analogRead(conductPin);
  if (val > condTol){
    return true; //
  }
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
    missionY = 1.5;
  }
  else if(y > 1){
    missionY = 0.5;    
  }

  //while the ultrasonic sensor does not detect anything within tolerance and while the calculated distance is far enough away, keep going forward
  while(getDist() > distTol && calcDist(x, missionX, y, missionY) > distTol ){
  //Enes100.print("entered while loop ");
  //Enes100.print("Mission Y:" + String(missionY));
  straight(missionX, missionY);
  updateLoc();
  }
  
  stopMotors();
}

void mission(){
  //do stuff
}

//navigate OSV through obstacles
void obstacles(){
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
    while(getDist() > distTol){
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

    //once an obstacle is detected, stop
    stopMotors();
    updateLoc(); //get location and update values

    if(x < obs1X){ //if OSV is still in the first row of obstacles
      float y1 = y;

      //while distance to middle of next obstacle is far, keep moving
      while(calcDist(x,x,y,y1+(obsHeight*des_tht/(pi/2)))>3){
        straight(des_tht);
        updateLoc(); //get location and update values
      }

      //if made it to the last row, just move right
      if(y <= missionY+1.8*(obsHeight*des_tht/(pi/2))){ //last row
        while(calcDist(x,obs1X+.3,y,y)>3){
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
        while(calcDist(x,x,y,y1+(obsHeight*des_tht/(pi/2)))>3){
          straight(des_tht); //move in desired direction
          updateLoc(); //get location and update values
        }
      }

      //If an obstacle was detected in the first column, since two obstacles can't be in the same row, turn in that direction
      else if(obsCoord[0] <= obs1X){ //detected obstacle in 1st column
        float y1 = y;
        //while far away from destination, move towards destination
        while(calcDist(x,x,y,obsCoord[1])>3){
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
  while(getDist() > distTol && calcDist(x, limboX, y, limboY) > distTol ){
  straight(limboX, limboY);
    updateLoc(); //get location and update values 
  }
  //stop once arrived at limbo
  stopMotors();
}

//go under limbo
void limbo(){
  turn(0); //turn right
  updateLoc(); //get location and update values

  //move forward to the right while distance sensor does not detect the wall or while the x and y values are far from final location
  //this uses the vision system as little as possible since the limbo blocks the camera from getting position values for a small amount of time
  while(getDist() > distTol && calcDist(x, finalX, y, finalY) > distTol){
    forward(); 
    delay(straightTol);
    stopMotors();
    updateLoc(); //get location and update values
  }

  //reached the end, stop the motors
  stopMotors();
}

void pump(float time, int dir){ //0 dir is in, 1 dir is out
  digitalWrite(pumpPow,HIGH);
  if(dir == 0)
    digitalWrite(pumpDir,HIGH);
  else
    digitalWrite(pumpDir,LOW);
  delay(time);
  digitalWrite(pumpDir,LOW);
  digitalWrite(pumpDir,LOW);
}
