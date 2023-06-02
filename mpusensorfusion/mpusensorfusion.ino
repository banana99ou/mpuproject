#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

unsigned long now, before;
unsigned long dt;
float dt_seconds;

float acceleration[3];
float angular_acceleration[3];

float velocity[3];
float angular_velocity[3];

float location[3];
float attitude[3];

float raw_accel_x, raw_accel_y, raw_accel_z;
float raw_gyro_x, raw_gyro_y, raw_gyro_z;
float sum_accel_x, sum_accel_y, sum_accel_z;
float sum_gyro_x, sum_gyro_y, sum_gyro_z;

float offset_accel_x, offset_accel_y, offset_accel_z;
float offset_gyro_x, offset_gyro_y, offset_gyro_z;


// Function to rotate the acceleration vector
void adjust_acceleration_offset(float& offset_accel_x, float& offset_accel_y, float& offset_accel_z, float attitude[3]) {
  // Extract the rotation angles from the attitude array
  float a = attitude[0]; // Rotation around x-axis
  float b = attitude[1]; // Rotation around y-axis
  float c = attitude[2]; // Rotation around z-axis

  // Rotation matrix around the x-axis
  float x_rot[3][3] = {
    {1, 0, 0},
    {0, cos(a), -sin(a)},
    {0, sin(a), cos(a)}
  };

  // Rotation matrix around the y-axis
  float y_rot[3][3] = {
    {cos(b), 0, sin(b)},
    {0, 1, 0},
    {-sin(b), 0, cos(b)}
  };

  // Rotation matrix around the z-axis
  float z_rot[3][3] = {
    {cos(c), -sin(c), 0},
    {sin(c), cos(c), 0},
    {0, 0, 1}
  };

  // Perform the rotations
  float result[3] = {offset_accel_x, offset_accel_y, offset_accel_z};

  // Rotate around the z-axis
  float temp[3];
  for (int i = 0; i < 3; i++) {
    temp[i] = 0;
    for (int j = 0; j < 3; j++) {
      temp[i] += z_rot[i][j] * result[j];
    }
  }

  // Rotate around the y-axis
  for (int i = 0; i < 3; i++) {
    result[i] = 0;
    for (int j = 0; j < 3; j++) {
      result[i] += y_rot[i][j] * temp[j];
    }
  }

  // Rotate around the x-axis
  for (int i = 0; i < 3; i++) {
    temp[i] = 0;
    for (int j = 0; j < 3; j++) {
      temp[i] += x_rot[i][j] * result[j];
    }
  }

  // Update the output variables
  offset_accel_x = temp[0];
  offset_accel_y = temp[1];
  offset_accel_z = temp[2];
}

void setup(void) {
  servo.attach(ServoPin);
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

  //Use a loop to take 100 readings and sum the values for each axis
  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    raw_accel_x = a.acceleration.x;
    raw_accel_y = a.acceleration.y;
    raw_accel_z = a.acceleration.z;
    raw_gyro_x = g.gyro.x;
    raw_gyro_y = g.gyro.y;
    raw_gyro_z = g.gyro.z;

    sum_accel_x += raw_accel_x;
    sum_accel_y += raw_accel_y;
    sum_accel_z += raw_accel_z;
    sum_gyro_x += raw_gyro_x;
    sum_gyro_y += raw_gyro_y;
    sum_gyro_z += raw_gyro_z;

    delay(10);
  }

  offset_accel_x = -(sum_accel_x / 100.0);
  offset_accel_y = -(sum_accel_y / 100.0);
  offset_accel_z = -(sum_accel_z / 100.0);
  offset_gyro_x = -(sum_gyro_x / 100.0);
  offset_gyro_y = -(sum_gyro_y / 100.0);
  offset_gyro_z = -(sum_gyro_z / 100.0);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  now = millis();
  dt = (now - before);
  dt_seconds = (float) dt / 1000.0;
  before = now;

  angular_acceleration[0] = g.gyro.x + offset_gyro_x;
  angular_acceleration[1] = g.gyro.y + offset_gyro_y;
  angular_acceleration[2] = g.gyro.z + offset_gyro_z;

  angular_velocity[0] += angular_acceleration[0] * dt_seconds;
  angular_velocity[1] += angular_acceleration[1] * dt_seconds;
  angular_velocity[2] += angular_acceleration[2] * dt_seconds;

  attitude[0] += angular_velocity[0] * dt_seconds;
  attitude[1] += angular_velocity[1] * dt_seconds;
  attitude[2] += angular_velocity[2] * dt_seconds;

  //adjust acceleration offset
  //adjust_acceleration_offset(offset_accel_x, offset_accel_y, offset_accel_z, attitude);

  acceleration[0] = a.acceleration.x + offset_accel_x;
  acceleration[1] = a.acceleration.y + offset_accel_y;
  acceleration[2] = a.acceleration.z + offset_accel_z;

  velocity[0] += acceleration[0] * dt_seconds;
  velocity[1] += acceleration[1] * dt_seconds;
  velocity[2] += acceleration[2] * dt_seconds;

  location[0] += velocity[0] * dt_seconds;
  location[1] += velocity[1] * dt_seconds;
  location[2] += velocity[2] * dt_seconds;

  // Serial.print(a.acceleration.x);
  // Serial.print(", ");
  // Serial.print(a.acceleration.y);
  // Serial.print(", ");
  // Serial.print(a.acceleration.z);
  // Serial.print(", ");
  // Serial.print(offset_accel_x);
  // Serial.print(", ");
  // Serial.print(offset_accel_y);
  // Serial.print(", ");
  // Serial.print(offset_accel_z);
  // Serial.print(", ");
  // Serial.print(acceleration[0]);
  // Serial.print(", ");
  // Serial.print(acceleration[1]);
  // Serial.print(", ");
  // Serial.print(acceleration[2]);
  // Serial.print(", ");
  // Serial.print(velocity[0]);
  // Serial.print(", ");
  // Serial.print(velocity[1]);
  // Serial.print(", ");
  // Serial.print(velocity[2]);
  // Serial.print(", ");
  // Serial.print(location[0]);
  // Serial.print(", ");
  // Serial.print(location[1]);
  // Serial.print(", ");
  // Serial.print(location[2]);
  // Serial.print(", ");
  // Serial.print(g.gyro.x);
  // Serial.print(", ");
  // Serial.print(g.gyro.y);
  // Serial.print(", ");
  // Serial.print(g.gyro.z);
  // Serial.print(", ");
  // Serial.print(offset_gyro_x);
  // Serial.print(", ");
  // Serial.print(offset_gyro_y);
  // Serial.print(", ");
  // Serial.print(offset_gyro_z);
  // Serial.print(", ");
  // Seiral.print(angular_acceleration[0]);
  // Serial.print(", ");
  // Seiral.print(angular_acceleration[1]);
  // Serial.print(", ");
  // Seiral.print(angular_acceleration[2]);
  // Serial.print(", ");
  // Serial.print(angular_velocity[0]);
  // Serial.print(", ");
  // Serial.print(angular_velocity[1]);
  // Serial.print(", ");
  // Serial.print(angular_velocity[2]);
  // Serial.print(", ");
  Serial.print(attitude[0]);
  Serial.print(", ");
  Serial.print(attitude[1]);
  Serial.print(", ");
  Serial.println(attitude[2]);
}
