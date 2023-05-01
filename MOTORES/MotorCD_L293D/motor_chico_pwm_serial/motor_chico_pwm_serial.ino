int motor1Pin1 = 33; 
int motor1Pin2 = 26; 
int enable1Pin = 27; 

#define PWM_MIN 3500
#define PWM_MAX 9200
 
// Setting PWM properties
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 16;
int dutyCycle = PWM_MIN;
 
void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(motor1Pin1, pwmChannel);   
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(enable1Pin, HIGH);
  Serial.begin(115200);
 
  // testing
  Serial.print("Testing DC Motor...");
  ledcWrite(pwmChannel, dutyCycle); 
}
 
void loop() {

    while(Serial.available() > 0)
  {          
    int value = Serial.readString().toInt(); 
    if((value>= PWM_MIN) &&  (value<= PWM_MAX))
    {
          ledcWrite(pwmChannel, value);     
          Serial.println("PWM");   
          Serial.println(value);
    }
    else
    {
      Serial.println("INVALID PWM 3500-9200");  
    }

  }
}
