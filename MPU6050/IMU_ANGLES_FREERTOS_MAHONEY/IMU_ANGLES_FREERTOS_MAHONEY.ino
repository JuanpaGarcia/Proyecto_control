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

//*****************************************************************************

//*****************************************************************************
// Defines
#define DELAY_PERIOD_MS 5 //delay period for tasks in mili seconds

//*****************************************************************************

//*****************************************************************************
// Global variables

//mpu global variables
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

//Semaphores
static SemaphoreHandle_t print_sem;     // Waits for parameter to be read

//*****************************************************************************


//*****************************************************************************
// Functions
//MPU6050
void set_mpu6050(void);
void calibrate_mpu6050(void);
//*****************************************************************************


//*****************************************************************************
// Tasks

// Producer: write a given number of times to shared buffer
void mpu_get_data(void *parameters) 
{
  while(1)
  {
    bno055_read_euler_hrp(&myEulerData);            //Update Euler data into the structure
    
    xSemaphoreGive(print_sem);
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void mpu_print_data(void *parameters) 
{
  while(1)
  {
    xSemaphoreTake(print_sem, portMAX_DELAY);
    /* Print out the values */
 
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
//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  
  Serial.begin(115200);
  Wire.begin();

  //set the IMU to desired 
  set_mpu6050();  
#ifdef CALIBRATE_IMU 
  calibrate_mpu6050();
#endif
  
  // Create mutexes and semaphores before starting tasks
  print_sem = xSemaphoreCreateBinary();
  
    xTaskCreatePinnedToCore(
                          mpu_get_data,   /* Function to implement the task */
                          "IMU", /* Name of the task */
                          1300,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          3,          /* Priority of the task */
                          NULL,       /* Task handle. */
                          app_cpu);  /* Core where the task should run */
  
    xTaskCreatePinnedToCore(
                          mpu_print_data,   /* Function to implement the task */
                          "Print IMU", /* Name of the task */
                          1024,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          3,          /* Priority of the task */
                          NULL,       /* Task handle. */
                          app_cpu);  /* Core where the task should run */
                            
}

void loop() 
{
}

void set_mpu6050(void)
{
  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device
 
  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
 
  delay(1);
}
