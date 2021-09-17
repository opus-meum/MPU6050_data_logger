/*
    MPU6050 6 Axis IMU data logger. Modified from:
*/

#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>

#define LED           9
#define CHIP_SELECT   4

#define USE_SD_CARD

File data_log;
MPU6050 mpu;

volatile uint32_t counter = 0;
const uint32_t limit = 4 * 60 * 60 * 100;

bool LED_on = false;

struct dataFrame
{
  IntVector gyro;
  IntVector accel;
  float temperature;
} rawData;

void flash_error_LED(void)
{
  while (1)
  {
    if (LED_on == false)
    {
      digitalWrite(LED, HIGH);
      LED_on = true;
    }
    else 
    {
      digitalWrite(LED, LOW);
      LED_on = false;
    }
    delay(500);
  }
}

void setup() 
{
  pinMode(9, OUTPUT);
  Serial1.begin(115200);

#ifdef USE_SD_CARD
  if (!SD.begin(CHIP_SELECT))
  {
    Serial1.println("SD Card initialisaiton failed");
    flash_error_LED();
  }
  else 
  {
    Serial1.println("SD Card initialisaiton successful");
  }
#endif

  // Initialize MPU6050
  Serial1.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G))
  {
    Serial1.println("Could not find a valid MPU6050 sensor, check wiring!");
    flash_error_LED();
  }

  mpu.writeRegister8(MPU6050_REG_SMPRT_DIV, MPU6050_SMPLRT_100_NF);
  mpu.writeRegister8(MPU6050_REG_INT_ENABLE, 1);  

#ifdef USE_SD_CARD
  data_log = SD.open("test.txt", O_WRITE | O_CREAT | O_APPEND);
  if (data_log)
  {
    Serial1.println("Opened data.txt");    
    data_log.println("GXraw, GYraw, GZraw, AXraw, AYraw, AZraw, Temp");
  }
  else
  {
    Serial1.println("Failed to open data.txt");
    flash_error_LED();
  }
#endif
  Serial1.println("Setup complete");

  digitalWrite(9,HIGH);
  attachInterrupt(6, dataReadyHandler, RISING);
}

void loop()
{}

void dataReadyHandler(void)
{
  rawData.gyro = mpu.readRawGyro();
  rawData.accel = mpu.readRawAccel();
  rawData.temperature = mpu.readTemperature();

#ifdef USE_SD_CARD
  data_log.print(rawData.gyro.XAxis);   
  data_log.print(","); 
  data_log.print(rawData.gyro.YAxis);   
  data_log.print(","); 
  data_log.print(rawData.gyro.ZAxis);   
  data_log.print(","); 
  data_log.print(rawData.accel.XAxis);   
  data_log.print(","); 
  data_log.print(rawData.accel.YAxis);   
  data_log.print(","); 
  data_log.print(rawData.accel.ZAxis);   
  data_log.print(","); 
  data_log.println(rawData.temperature);

  if (!(counter % 36))
  {
    data_log.flush();
  }
  else if (!(counter % 100))
  {
    Serial1.println(counter / 100);
  }
#else
  Serial1.print(rawData.gyro.XAxis);   
  Serial1.print(","); 
  Serial1.print(rawData.gyro.YAxis);   
  Serial1.print(","); 
  Serial1.print(rawData.gyro.ZAxis);   
  Serial1.print(","); 
  Serial1.print(rawData.accel.XAxis);   
  Serial1.print(","); 
  Serial1.print(rawData.accel.YAxis);   
  Serial1.print(","); 
  Serial1.print(rawData.accel.ZAxis);   
  Serial1.print(","); 
  Serial1.print(rawData.temperature);
  Serial1.print(","); 
  Serial1.println(counter);
#endif

  if (counter >= limit)
  {
    detachInterrupt(6);
#ifdef USE_SD_CARD
    data_log.close();
#else
    Serial1.println("Done.");
#endif    
    digitalWrite(LED, LOW);
  }
  counter++;
}
