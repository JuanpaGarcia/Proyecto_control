int motor1Pin1 = 33; 
int motor1Pin2 = 26; 
int enable1Pin = 27; 
 
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 16;
int dutyCycle = 45000;
 
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
}
 
void loop() {

  while (dutyCycle <= 65500){
    ledcWrite(pwmChannel, dutyCycle);   
    Serial.print("Forward with duty cycle: ");
    Serial.println(dutyCycle);
    dutyCycle = dutyCycle + 50;
    delay(500);
  }
  dutyCycle = 44000;
}
