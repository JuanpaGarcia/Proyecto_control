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
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//*****************************************************************************

//*****************************************************************************
// Defines
#define I2C_SCL_PIN 0
#define I2C_SDA_PIN 0
#define DELAY_PERIOD_MS 5

//*****************************************************************************


//*****************************************************************************
// Global variables

//mpu global variables
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp; //Get new sensor events with the readings accel and gyro

//Semaphores
static SemaphoreHandle_t print_sem;     // Waits for parameter to be read

//*****************************************************************************


//*****************************************************************************
// Functions
void set_mpu6050(void);
//*****************************************************************************


//*****************************************************************************
// Tasks

// Producer: write a given number of times to shared buffer
void mpu_get_data(void *parameters) 
{
  while(1)
  {
    mpu.getEvent(&a, &g, &temp);
    // Release the binary semaphore
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
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print(g.gyro.z);
    Serial.println("");
    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

// Consumer: continuously read from shared buffer
void PID_motor_1(void *parameters) 
{

  while (1) 
  {

    vTaskDelay(DELAY_PERIOD_MS / portTICK_PERIOD_MS);
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }
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

    set_mpu6050();    //set the IMU to desired 
}

void loop() 
{
}

void set_mpu6050(void)
{
    // Try to initialize!
  if (!mpu.begin()) 
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1) 
    {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);  
}
