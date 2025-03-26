#include <Wire.h>
#include "LSM6DS3.h"

LSM6DS3 imu(I2C_MODE, 0x6A);  // LSM6DS3 I2C address

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Wait for Serial Monitor
    if (imu.begin() != 0) {
        Serial.println("IMU Initialization Failed!");
        while (1);
    }
    Serial.println("IMU Initialized!");
}

void loop() {
    float ax = imu.readFloatAccelX();
    float ay = imu.readFloatAccelY();
    float az = imu.readFloatAccelZ();
    float gx = imu.readFloatGyroX();
    float gy = imu.readFloatGyroY();
    float gz = imu.readFloatGyroZ();

    // Send data as CSV format
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);

    delay(50);  // Adjust for smoother data flow
}
