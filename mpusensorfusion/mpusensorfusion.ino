#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

unsigned long now, before;
unsigned long dt;
float dt_seconds;

float acceleration[3];
float velocity[3];
float location[3];

float offset_x, offset_y, offset_z;
float sum_x, sum_y, sum_z;
float raw_x, raw_y, raw_z;
float alpha = 0.96; // tuning parameter for the complementary filter

void setup(void) {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MPU6050");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("");
  delay(100);

  //Use a loop to take 100 readings and sum the values for each axis
  for (int i = 0; i < 100; i++) 
    {
        //Read raw data from MPU6050 (use your MPU6050 library)
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        raw_x = a.acceleration.x;
        raw_y = a.acceleration.y;
        raw_z = a.acceleration.z;

        //Serial.println(raw_x);
        
        //Sum the values for each axis
        sum_x += raw_x;
        sum_y += raw_y;
        sum_z += raw_z;

        //Serial.println(sum_x);
        
        //Delay for a short time to allow MPU6050 to stabilize
        delay(10);
    }

  //Calculate average for each axis
  float average_x = sum_x / 100.0;
  float average_y = sum_y / 100.0;
  float average_z = sum_z / 100.0;

  //Serial.println(average_x);
  //Calculate offset value for each axis
  offset_x = average_x;
  offset_y = average_y;
  offset_z = average_z;


}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  now = millis();
  dt = (now - before);
  dt_seconds = (float) dt / 1000.0;
  before = now;
  acceleration[0] = a.acceleration.x - offset_x;
  acceleration[1] = a.acceleration.y - offset_y;
  acceleration[2] = a.acceleration.z - offset_z;

  // Complementary filter to estimate the orientation and remove errors from the acceleration data
  velocity[0] = alpha * (velocity[0] + acceleration[0] * dt_seconds) + (1 - alpha) * g.gyro.x;
  velocity[1] = alpha * (velocity[1] + acceleration[1] * dt_seconds) + (1 - alpha) * g.gyro.y;
  velocity[2] = alpha * (velocity[2] + acceleration[2] * dt_seconds) + (1 - alpha) * g.gyro.z;

  location[0] += velocity[0] * dt_seconds;
 
