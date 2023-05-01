#include <Servo.h>
#define SERVO_PIN 25

Servo tail_servo;  // create servo object to control a servo

int pos = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("PWM Start");  

  tail_servo.attach(SERVO_PIN);  // attaches the servo on pin 13 to the servo object
  tail_servo.write(pos);
}

void loop() {
  
  // put your main code here, to run repeatedly:

    while(Serial.available() > 0)
  {          
    int value = Serial.readString().toInt(); 
    if((value>= 0) &&  (value<= 180))
    {
          tail_servo.write(value);    
          Serial.print("Angular position ");   
          Serial.println(value);
    }
    else
    {
      Serial.println("INVALID POSITION");  
    }
  }
}
