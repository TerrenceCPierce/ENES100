#include "Enes100.h"

//global 
float const tht_tol = .1;
float const pi = 3.1415;
float const turnTimeTol = .1;

void setup() {
  // put your setup code here, to run once:
    Enes100.begin("Elephante", WATER, 219, 10, 11);

    Enes100.print("Destination is at (");
    Enes100.print(Enes100.missionSite.x);
    Enes100.print(", ");
    Enes100.print(Enes100.missionSite.y);
    Enes100.println(")");


}

void loop() {
  // put your main code here, to run repeatedly:
  straight(Enes100.missionSite.x, Enes100.missionSite.y);
}

void straight(float x_dest, float y_dest){
  Enes100.updateLocation();
  float x = Enes100.location.x;
  float y = Enes100.location.y;
  float tht = getTheta();
  //Turn Motor pins high
  delay(500);
  if (!(tht <= atan2((y_dest-y),(x_dest-x))+ tht_tol && tht >= atan2((y_dest-y),(x_dest-x))- tht_tol)){
    turn(atan2((y_dest-y),(x_dest-x)));
  }
}

void turn(int desired_tht){
  float tht = getTheta();
  while(abs(desired_tht-tht) > tht_tol){
    float diff_tht = desired_tht-tht;
    if (desired_tht> tht){
      //Add code to check if close
      if(diff_tht >= pi) 
        turnRight(int(diff_tht/turnTimeTol));
      if(diff_tht < pi)
        turnLeft(int(diff_tht/turnTimeTol));
    }
    else{
      if(diff_tht <= pi) 
        turnRight(int(diff_tht/turnTimeTol));
      if(diff_tht > pi)
        turnLeft(int(diff_tht/turnTimeTol));
    }
    tht = getTheta();
  }

}

void turnRight(int time){
  //Left wheels CCW, Right wheels CW
  delay(50*time);
}

void turnLeft(int time){
  //Left wheels CW, Right wheels CCW
  delay(50*time);
  
}

float getTheta(){
  float tht = Enes100.location.theta;
  if (tht < 0)
    tht += 2*pi;
  delay(50);
  return tht;
}