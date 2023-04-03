int const motor1Pow = 4; //motors pins
int const motor1Dir = 5;
int const motor2Pow = 6;
int const motor2Dir = 7;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1Pow, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Pow, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  digitalWrite(motor1Pow, LOW);
  digitalWrite(motor1Dir, LOW);
  digitalWrite(motor2Pow, LOW);
  digitalWrite(motor2Dir, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
  forward();
  delay(1000);
  while(1){}
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
  digitalWrite(motor1Pow,LOW);
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