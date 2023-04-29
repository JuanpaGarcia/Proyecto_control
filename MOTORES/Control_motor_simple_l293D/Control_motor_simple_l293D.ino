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
#define ENABLE_1_PIN 35
#define MOTOR_1_INPUT_1_PIN 34
#define MOTOR_1_INPUT_2_PIN 33
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8
#define MOTOR_1_PWM_CHANNEL 0


#define DELAY_PERIOD_MS 5

//*****************************************************************************


//*****************************************************************************
// Global variables

//mpu global variables

//Semaphores
static SemaphoreHandle_t print_sem;     // Waits for parameter to be read

//*****************************************************************************


//*****************************************************************************
// Functions
void set_motor(void);
//*****************************************************************************


//*****************************************************************************
// Tasks

// Producer: write a given number of times to shared buffer
void motor_test(void *parameters) 
{
  char pwm = 0;
  while(1)
  {
    
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
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
                          "IMU", /* Name of the task */
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
   // Try to initialize!
  // sets the pins as outputs for motor 1:
  pinMode(MOTOR_1_INPUT_1_PIN, OUTPUT);
  pinMode(MOTOR_1_INPUT_2_PIN, OUTPUT);
  pinMode(ENABLE_1_PIN, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(MOTOR_1_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ENABLE_1_PIN, MOTOR_1_PWM_CHANNEL);
  
}
