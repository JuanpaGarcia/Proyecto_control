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
#include "BNO055_support.h"     //Contains the bridge code between the API and Arduino
#include <Wire.h>
#include <Servo.h>
//*****************************************************************************

//*****************************************************************************
// Defines
#define DELAY_PERIOD_MS 5
#define SWEEP_SERVO_DELAY_MS 750

//i2c
#define I2C_CLOCK_SPEED 400000
#define SDA 21
#define SCL 22

//IMU global variables
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

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

//Servo Defines
#define SERVO_PIN 25
#define MIN_POS_SERVO 0
#define MAX_POS_SERVO 180
#define SWEEP_SPAN 60

//*****************************************************************************

//*****************************************************************************
// ENUM
enum motors{MOTOR_1 =0, MOTOR_2};
//*****************************************************************************

//*****************************************************************************
// Global variables

//motors global
int pwm = MIN_PWM_FOR_DRIVER;


// Servo variables
Servo tail_servo;  
int ang_pos_servo = 90;

//PID 1  VARIABLES
float PID_1_kp = 8; //Mine was 8_
float PID_1_ki = 0.2; //Mine was 0.2
float PID_1_kd = 3100; //Mine was 3100
float reference = 0;           //Should be pitch
float angle_error_PID_1 = 0.0;
float previous_angle_error_PID_1 = 0.0;
float PID_1_p, PID_1_i, PID_1_d, PID_1_total;
int u_k_PID_1 = MIN_PWM_FOR_DRIVER;

//ANGLES REFERENCE VARIABLES
float roll_reference = 5;
float pitch_reference = 6;
float yaw_reference = 7;

//Semaphores
static SemaphoreHandle_t print_sem;     // Waits for parameter to be read

//*****************************************************************************


//*****************************************************************************
// Functions
//MPU6050
void set_mpu6050(void);
void calibrate_mpu6050(void);

//MPU6050
void set_BNO055(void);

// MOTORS
void set_motor(void);
void set_motor_forward(motors A);
void set_motor_backward(motors A);
void set_servo(void);


//*****************************************************************************


//*****************************************************************************
// Tasks

// Producer: write a given number of times to shared buffer
void IMU_get_data(void *parameters) 
{
  while(1)
  {  
    bno055_read_euler_hrp(&myEulerData);            //Update Euler data into the structure
    
    // Release the binary semaphore
    xSemaphoreGive(print_sem);
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void IMU_print_data(void *parameters) 
{
  while(1)
  {
    xSemaphoreTake(print_sem, portMAX_DELAY);

    //Serial.print("Heading(Yaw):");             //To read out the Heading (Yaw)
    Serial.print(float(myEulerData.h) / 16.00);       //Convert to degrees
    Serial.print(",");
 
    //Serial.print("Roll:");                 //To read out the Roll
    Serial.print(float(myEulerData.r) / 16.00);       //Convert to degrees
    Serial.print(",");
 
    //Serial.print("Pitch:");                //To read out the Pitch
    Serial.print(float(myEulerData.p) / 16.00);       //Convert to degrees
    Serial.println("");
    
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void servo_angular_velocity_task(void *parameters)
{
  static bool turn = false;
  while(1)
  {
    if(false == turn)
    {
      tail_servo.write(ang_pos_servo - SWEEP_SPAN); 
      turn = true; 
    }
    else
    {
      tail_servo.write(ang_pos_servo + SWEEP_SPAN );
      turn = false;
    }
    vTaskDelay(SWEEP_SERVO_DELAY_MS / portTICK_PERIOD_MS);
  }
}

//calculate references for controllers
void References_task(void *parameters) 
{
  while (1) 
  {
    roll_reference = 5;
    pitch_reference = 6;
    yaw_reference = 7;
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void PID_motor_DC_1(void *parameters) 
{
  while (1) 
  {
    
    angle_error_PID_1 = pitch_reference - (float(myEulerData.p) / 16.00);
    PID_1_p = PID_1_kp * angle_error_PID_1;
    PID_1_d = PID_1_kd * ( (angle_error_PID_1 - previous_angle_error_PID_1) / DELAY_PERIOD_MS );

    if(-3 < distance_error && distance_error < 3)
    {
      PID_i = PID_i + (PID_1_kd * distance_error);
    }
    else
    {
      PID_i = 0;
    }

    PID_1_total = PID_p + PID_i + PID_d; 

    //change motor spin
    if(0 >= PID_1_total)
    {
      set_motor_forward(MOTOR_1);
      u_k_PID_1 = PID_1_total;
    }
    else
    {
      set_motor_backward(MOTOR_1);
      u_k_PID_1 = PID_1_total*-1;
    }
    
    if (MAX_PWM_FOR_DRIVER <= u_k_PID_1) 
    { 
      u_k_PID_1 = MAX_PWM_FOR_DRIVER;
    }  
    previous_angle_error_PID_1 = angle_error_PID_1;
    
    int a = 2;
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  
  Serial.begin(115200);
  Wire.begin(SDA, SCL, I2C_CLOCK_SPEED);
  while (!Serial) 
  {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  //set the IMU to desired 
  set_BNO055();  

  //set motors
  set_motor();
  //Set Servo
  set_servo();
  
  // Create mutexes and semaphores before starting tasks
  print_sem = xSemaphoreCreateBinary();
  
    xTaskCreatePinnedToCore(
                          IMU_get_data,   /* Function to implement the task to get the MPU data an conver it to navegation angles */
                          "IMU", /* Name of the task */
                          1300,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          5,          /* Higher Priority */
                          NULL,       /* Task handle. */
                          app_cpu);  /* Core where the task should run */
  
    xTaskCreatePinnedToCore(
                          IMU_print_data,   /* Function to implement the task print the navegation angles */
                          "Print IMU", /* Name of the task */
                          1024,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          3,          /* Priority of the task */
                          NULL,       /* Task handle. */
                          app_cpu);  /* Core where the task should run */
                          
    xTaskCreatePinnedToCore(
                          References_task,   /* Function to implement the task to make references por PID controler DC motor*/
                          "Direction", /* Name of the task */
                          600,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          3,          /* Priority of the task */
                          NULL,       /* Task handle. */
                          app_cpu);  /* Core where the task should run */

    xTaskCreatePinnedToCore(
                      PID_motor_DC_1,   /* Function to implement the task PID controles por DC motor for pitch control*/
                      "SPEED", /* Name of the task */
                      600,      /* Stack size in words */
                      NULL,       /* Task input parameter */
                      3,          /* Priority of the task */
                      NULL,       /* Task handle. */
                      app_cpu);  /* Core where the task should run */ 

    xTaskCreatePinnedToCore(
                      servo_angular_velocity_task,   /* Function to implement the task to get the MPU data an conver it to navegation angles */
                      "Tail movement", /* Name of the task */
                      600,      /* Stack size in words */
                      NULL,       /* Task input parameter */
                      3,          /* Higher Priority */
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
    digitalWrite(MOTOR_1_INPUT_1_PIN, LOW);
    digitalWrite(MOTOR_1_INPUT_2_PIN, HIGH);
  }
  else
  {
  
  }
}

void set_servo(void)
{
    tail_servo.attach(SERVO_PIN);  // attaches the servo on pin 13 to the servo object
    tail_servo.write(ang_pos_servo);  
}



void set_BNO055(void)
{
    //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device
 
  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
 
  delay(1);
}
