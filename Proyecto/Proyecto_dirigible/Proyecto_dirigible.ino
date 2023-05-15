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
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <Servo.h>
//*****************************************************************************

//*****************************************************************************
// Defines
#define DELAY_PERIOD_MS 5
#define DELAY_FLOAT_IN_SECONDS (0.005f)
#define SWEEP_SERVO_DELAY_MS 750

//i2c
#define I2C_CLOCK_SPEED 400000
#define SDA 21
#define SCL 22


//Define motor pins and pwm channels
#define MIN_VOLTAGE_H_BRIDGE  2.4
#define ENABLE_1_PIN 33 
#define MOTOR_1_INPUT_1_PIN 27
#define MOTOR_1_INPUT_2_PIN 26
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
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//*****************************************************************************

//*****************************************************************************
// Global variables

//motors global
int pwm = MIN_PWM_FOR_DRIVER;


// Servo variables
Servo tail_servo;  
int ang_pos_servo = 90;

//PID 1  VARIABLES
float PID_1_kp = 200; //Mine was 8_
//float PID_1_ki = 5; //Mine was 0.2
float PID_1_kd = 1500; //Mine was 3100
float reference = 0;           //Should be pitch
float angle_error_PID_1 = 0.0;
float previous_angle_error_PID_1 = 0.0;
float PID_1_p = 0.0, PID_1_i = 0.0, PID_1_d = 0.0, PID_1_total = 0.0;
int u_k_PID_1 = MIN_PWM_FOR_DRIVER;

//PID 2  VARIABLES
float PID_2_kp = 200; //Mine was 8_
//float PID_2_ki = 5; //Mine was 0.2
float PID_2_kd = 1500; //Mine was 3100
float angle_error_PID_2 = 0.0;
float previous_angle_error_PID_2 = 0.0;
float PID_2_p = 0.0, PID_2_i = 0.0, PID_2_d = 0.0, PID_2_total = 0.0;
int u_k_PID_2 = MIN_PWM_FOR_DRIVER;

//IMU ADAFRUIT LIBRARY
sensors_event_t event;

//IMU ANGLES in degrees
float roll = 5;
float pitch = 6;
float yaw= 7;

//ANGLES REFERENCE VARIABLES
float roll_reference = 5;
float pitch_reference = 6;
float yaw_reference = 7;

//Semaphores
static SemaphoreHandle_t print_sem;     // Waits for parameter to be read
static SemaphoreHandle_t PID_1_sem;     // Waits for parameter to be read


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
    bno.getEvent(&event);
    yaw = (float)event.orientation.x;
    pitch = (float)event.orientation.y;
    roll = (float)event.orientation.z;
    
    // Release the binary semaphore
    xSemaphoreGive(print_sem);
    xSemaphoreGive(PID_1_sem);
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void IMU_print_data(void *parameters) 
{
  //delay(4000);
  //Serial.println("Pitch Pitch_reference uk");
  while(1)
  {
    xSemaphoreTake(print_sem, portMAX_DELAY);

    /*
    Serial.print(yaw);
    Serial.print(F(" "));
    */

    
//    Serial.print(pitch);
//    Serial.print(F(" "));

    /*
    Serial.print(roll);
    Serial.print(F(" "));
    */

    //References
    //Serial.println(yaw_reference);
    //Serial.print(F(" "));
    
//    Serial.print(pitch_reference);
//    Serial.print(F(" "));
    
    //Serial.println(roll_reference);
    //Serial.print(F(" "));
    
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
    roll_reference = 0;   //forward and backwar tilt, z,x
    pitch_reference = 6;  // NOT USED z,y
    yaw_reference = 7;    // x,y angle
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void PID_motor_DC_1(void *parameters) // 
{
  float mapped_value = 0.0;
  while (1) 
  {
    //xSemaphoreTake(PID_1_sem, portMAX_DELAY);
    angle_error_PID_1 = pitch_reference  - pitch;
    PID_1_p = PID_1_kp * angle_error_PID_1;
    PID_1_d = PID_1_kd * ( (angle_error_PID_1 - previous_angle_error_PID_1) / DELAY_FLOAT_IN_SECONDS );

    if(-3 < angle_error_PID_1 && angle_error_PID_1 < 3)
    {
      PID_1_i = PID_1_i + (PID_1_kd * angle_error_PID_1);
    }
    else
    {
      PID_1_i = 0;
    }

    PID_1_total = PID_1_p + PID_1_i + PID_1_d; 
    //Serial.print(PID_1_total);
    //Serial.print("\t");
    //change motor spin
    if(0 <= PID_1_total)
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
    
    //Serial.println(u_k_PID_1);


    mapped_value = map(u_k_PID_1, 0, MAX_PWM_FOR_DRIVER, MIN_PWM_FOR_DRIVER -500 ,MAX_PWM_FOR_DRIVER );

     Serial.println(mapped_value);
    ledcWrite(MOTOR_1_PWM_CHANNEL, mapped_value); 
    previous_angle_error_PID_1 = angle_error_PID_1;
    
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void PID_tail(void *parameters) // 
{
  float mapped_value = 0.0;
  while (1) 
  {
    //xSemaphoreTake(PID_1_sem, portMAX_DELAY);
    angle_error_PID_2 = yaw_reference  - yaw;
    PID_2_p = PID_2_kp * angle_error_PID_2;
    PID_2_d = PID_2_kd * ( (angle_error_PID_2 - previous_angle_error_PID_2) / DELAY_FLOAT_IN_SECONDS );

    if(-3 < angle_error_PID_2 && angle_error_PID_2 < 3)
    {
      PID_2_i = PID_2_i + (PID_2_kd * angle_error_PID_2);
    }
    else
    {
      PID_2_i = 0;
    }

    PID_2_total = PID_2_p + PID_2_i + PID_2_d; 

    PID_2_total = map(PID_total, -150, 150, 0, 150);
  
    if(PID_2_total < 30){PID_2_total = 30;}
    if(PID_2_total > 160) {PID_2_total = 150; } 
    
    //Serial.println(u_k_PID_1);

    ang_pos_servo = PID_2_total;
    tail_servo.write(ang_pos_servo);
    
    previous_angle_error_PID_2 = angle_error_PID_2;
    
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
  PID_1_sem = xSemaphoreCreateBinary();
  
    xTaskCreatePinnedToCore(
                          IMU_get_data,   /* Function to implement the task to get the MPU data an conver it to navegation angles */
                          "IMU", /* Name of the task */
                          1300,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          4,          /* Higher Priority */
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
                          900,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          3,          /* Priority of the task */
                          NULL,       /* Task handle. */
                          app_cpu);  /* Core where the task should run */

    xTaskCreatePinnedToCore(
                      PID_motor_DC_1,   /* Function to implement the task PID controles por DC motor for pitch control*/
                      "SPEED", /* Name of the task */
                      900,      /* Stack size in words */
                      NULL,       /* Task input parameter */
                      4,          /* Priority of the task */
                      NULL,       /* Task handle. */
                      app_cpu);  /* Core where the task should run */ 

    xTaskCreatePinnedToCore(
                      servo_angular_velocity_task,   /* Function to implement the task to get the MPU data an conver it to navegation angles */
                      "Tail movement", /* Name of the task */
                      900,      /* Stack size in words */
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
  ///////////////

    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
 
  delay(1);
}
