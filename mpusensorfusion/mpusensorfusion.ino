#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

unsigned long now, before;
unsigned long dt;
float dt_seconds;

float acceleration[3];
float angular_acceleration[3];

float velocity[3];
float angular_velocity[3];

float location[3];
float attitude[3];

float offset_x, offset_y, offset_z;
float sum_x, sum_y, sum_z;
float raw_x, raw_y, raw_z;

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
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Allow the sensor to stabilize
  delay(1000);

  // Calculate initial offset for acceleration
  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    raw_x = a.acceleration.x;
    raw_y = a.acceleration.y;
    raw_z = a.acceleration.z;

    sum_x += raw_x;
    sum_y += raw_y;
    sum_z += raw_z;

    delay(10);
  }

  float average_x = sum_x / 100.0;
  float average_y = sum_y / 100.0;
  float average_z = sum_z / 100.0;

  offset_x = -average_x;
  offset_y = -average_y;
  offset_z = -average_z;

  // Store the initial time
  before = millis();
}

void loop() {
  now = millis();
  dt = (now - before);
  dt_seconds = (float)dt / 1000.0;
  before = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  acceleration[0] = a.acceleration.x - offset_x;
  acceleration[1] = a.acceleration.y - offset_y;
  acceleration[2] = a.acceleration.z - offset_z;

  angular_acceleration[0] = g.gyro.x;
  angular_acceleration[1] = g.gyro.y;
  angular_acceleration[2] = g.gyro.z;

  attitude[0] = atan2(acceleration[1], acceleration[2]);  // Roll
  attitude[1] = atan2(-acceleration[0], sqrt(acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2]));  // Pitch

  // Adjust acceleration based on the current attitude
  float cos_roll = cos(attitude[0]);
  float sin_roll = sin(attitude[0]);
  float cos_pitch = cos(attitude[1]);
  float sin_pitch = sin(attitude[1]);

  float adjusted_acceleration_x = acceleration[0];
  float adjusted_acceleration_y = acceleration[1] * cos_roll - acceleration[2] * sin_roll;
  float adjusted_acceleration_z = acceleration[1] * sin_roll + acceleration[2] * cos_roll;

  velocity[0] += adjusted_acceleration_x * dt_seconds;
  velocity[1] += adjusted_acceleration_y * dt_seconds;
  velocity[2] += adjusted_acceleration_z * dt_seconds;

  angular_velocity[0] += angular_acceleration[0] * dt_seconds;
  angular_velocity[1] += angular_acceleration[1] * dt_seconds;
  angular_velocity[2] += angular_acceleration[2] * dt_seconds;

  location[0] += velocity[0] * dt_seconds;
  location[1] += velocity[1] * dt_seconds;
  location[2] += velocity[2] * dt_seconds;

  Serial.print(location[0]);
  Serial.print(", ");
  Serial.print(location[1]);
  Serial.print(", ");
  Serial.println(location[2]);

  delay(10);
}
