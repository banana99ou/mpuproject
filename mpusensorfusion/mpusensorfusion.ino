#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
    Serial.begin(115200);
    if (!mpu.begin()){
        Serial.println("Failed to find MPU6050);
        while(1){
            delay(10);
        }
    }
    Serial.println("Found MPU6050");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
}

void loop(){
	
}
