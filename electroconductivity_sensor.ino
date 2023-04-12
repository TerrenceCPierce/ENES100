void setup() {
pinMode(9,OUTPUT); //this tells the Arduino that I want to use digital pin 9 as an output, or a power supply, not as a sensor
Serial.begin(9600); //this initializes the Serial Monitor so that I can see things on my computer screen
}

void loop() {

Serial.println(analogRead(A0));
if(analogRead(A0)>876) //this is the condition to be checked
{
  digitalWrite(9,HIGH); //this turns pin 9 onto full voltage (5V)
}

else
{
  digitalWrite(9,LOW);
}

}
