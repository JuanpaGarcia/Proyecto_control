int motor1Pin1 = 33; 
int motor1Pin2 = 26; 
int enable1Pin = 27; 
 
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 16;
int dutyCycle = 55000;


//value max 65535
//value min 
void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);   
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor1Pin1 , HIGH);
  Serial.begin(115200);
 
  // testing
  Serial.print("Testing DC Motor...");
  ledcWrite(pwmChannel, dutyCycle); 
}
 
void loop() {

    while(Serial.available() > 0)
  {          
    int value = Serial.readString().toInt(); 
    ledcWrite(pwmChannel, value);     
    Serial.print("PWM");   
    Serial.println(value);
  }
}
