// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif
//FREERTOS
///////////

//*****************************************************************************
// Libraries

//*****************************************************************************

//*****************************************************************************
// Defines

//Define motor pins and pwm channels
#define MIN_VOLTAGE_H_BRIDGE  2.4
#define ENABLE_1_PIN 27
#define MOTOR_1_INPUT_1_PIN 26
#define MOTOR_1_INPUT_2_PIN 33
#define PWM_FREQUENCY 30000
#define PWM_RESOLUTION_BITS 16
#define MOTOR_1_PWM_CHANNEL 0
#define MIN_PWM_FOR_DRIVER  3500
#define MAX_PWM_FOR_DRIVER  9200

#define DELAY_PERIOD_MS 5

//*****************************************************************************

//*****************************************************************************
// ENUM
enum motors{MOTOR_1 =0, MOTOR_2};
//*****************************************************************************
// Global variables

//mpu global variables

//motors global
int pwm = MIN_PWM_FOR_DRIVER;

//Semaphores
static SemaphoreHandle_t print_sem;     // Waits for parameter to be read

//*****************************************************************************


//*****************************************************************************
// Functions
void set_motor(void);
void set_motor_forward(motors A);

//*****************************************************************************


//*****************************************************************************
// Tasks

void motor_test(void *parameters) 
{
  while(1)
  {
    set_motor_forward(MOTOR_1);
    
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    set_motor_backward(MOTOR_1);
    vTaskDelay(6000 / portTICK_PERIOD_MS);
  }
}

void motor_values(void *parameters) 
{
  while(1)
  {
    pwm += 500;
    if(MAX_PWM_FOR_DRIVER <= pwm)
    {
      pwm = MIN_PWM_FOR_DRIVER;
    }
    Serial.print("Duty cycle: ");
    Serial.println(pwm);
    ledcWrite(MOTOR_1_PWM_CHANNEL, pwm); 
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  
  Serial.begin(115200);
  while (!Serial) 
  {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  //set the motors pin and pwm  
  set_motor();
 
  // Create mutexes and semaphores before starting tasks
  print_sem = xSemaphoreCreateBinary();
  
    xTaskCreatePinnedToCore(
                          motor_test,   /* Function to implement the task */
                          "Direction", /* Name of the task */
                          1300,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          3,          /* Priority of the task */
                          NULL,       /* Task handle. */
                          app_cpu);  /* Core where the task should run */

    xTaskCreatePinnedToCore(
                      motor_values,   /* Function to implement the task */
                      "SPEED", /* Name of the task */
                      1300,      /* Stack size in words */
                      NULL,       /* Task input parameter */
                      3,          /* Priority of the task */
                      NULL,       /* Task handle. */
                      app_cpu);  /* Core where the task should run */
  
}

void loop() 
{
}

void set_motor(void)
{
  // sets the pins as outputs for motor 1:
  pinMode(MOTOR_1_INPUT_1_PIN, OUTPUT);
  pinMode(MOTOR_1_INPUT_2_PIN, OUTPUT);
  pinMode(ENABLE_1_PIN, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(MOTOR_1_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ENABLE_1_PIN, MOTOR_1_PWM_CHANNEL);

  //set outputs on LOW 
  digitalWrite(MOTOR_1_INPUT_1_PIN, LOW);
  digitalWrite(MOTOR_1_INPUT_2_PIN, LOW);
}

void set_motor_forward(motors A)
{
    if(MOTOR_1==A)
  {
    Serial.println("Forward");
    digitalWrite(MOTOR_1_INPUT_2_PIN, LOW);
    digitalWrite(MOTOR_1_INPUT_1_PIN, HIGH);
  }
  else
  {
  
  }
}

void set_motor_backward(motors A)
{
  if(MOTOR_1==A)
  {
    Serial.println("Backward");
    digitalWrite(MOTOR_1_INPUT_1_PIN, LOW);
    digitalWrite(MOTOR_1_INPUT_2_PIN, HIGH);
  }
  else
  {
  
  }
}
