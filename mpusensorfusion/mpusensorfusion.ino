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
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);

  // Use a loop to take 100 readings and sum the values for each axis
  for (int i = 0; i < 100; i++) {
    // Read raw data from MPU6050 (use your MPU6050 library)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    raw_x = a.acceleration.x;
    raw_y = a.acceleration.y;
    raw_z = a.acceleration.z;

    // Sum the values for each axis
    sum_x += raw_x;
    sum_y += raw_y;
    sum_z += raw_z;

    // Delay for a short time to allow MPU6050 to stabilize
    delay(10);
  }

  // Calculate average for each axis
  float average_x = sum_x / 100.0;
  float average_y = sum_y / 100.0;
  float average_z = sum_z / 100.0;

  // Calculate offset value for each axis
  offset_x = -average_x;
  offset_y = -average_y;
  offset_z = -average_z;
}

void loop() {
  before = millis(); // Initialize 'before' variable

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  now = millis();
  dt = (now - before);
  dt_seconds = (float) dt / 1000.0;
  before = now;
  acceleration[0] = a.acceleration.x - offset_x;
  acceleration[1] = a.acceleration.y - offset_y;
  acceleration[2] = a.acceleration.z - offset_z;

  velocity[0] += acceleration[0] * dt_seconds;
  velocity[1] += acceleration[1] * dt_seconds;
  velocity[2] += acceleration[2] * dt_seconds;

  location[0] += velocity[0] * dt_seconds;
  location[1] += velocity[1] * dt_seconds;
  location[2] += velocity[2] * dt_seconds;

  Serial.print(acceleration[0]);
  Serial.print(", ");
  Serial.print(dt_seconds);
  Serial.print(", ");
  Serial.print(acceleration[0]);
  Serial.print(", ");
  Serial.print(velocity[0]);
  Serial.print(", ");
  Serial.println(location[0]);
}
