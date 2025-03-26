#include <Wire.h>
#include <ArduinoBLE.h>
#include "LSM6DS3.h"

// Custom Service UUID for nRF compatibility
#define CUSTOM_SERVICE_UUID       "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CUSTOM_CHAR_UUID_TX       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// IMU Initialization
LSM6DS3 imu(I2C_MODE, 0x6A);  

// Create BLE Service and Characteristic
BLEService imuService(CUSTOM_SERVICE_UUID);
BLEStringCharacteristic txCharacteristic(CUSTOM_CHAR_UUID_TX, BLERead | BLENotify, 50);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // IMU Initialization
  if (imu.begin() != 0) {
    Serial.println("IMU Initialization Failed!");
    while (1);
  }
  Serial.println("IMU Initialized!");

  // BLE Initialization
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  // Configure BLE parameters
  BLE.setLocalName("IMUSensor");
  BLE.setAdvertisedService(imuService);

  // Add characteristic to service
  imuService.addCharacteristic(txCharacteristic);

  // Add service to BLE stack
  BLE.addService(imuService);

  // Start advertising
  BLE.advertise();

  Serial.println("BLE IMU Sensor Device Ready");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Read IMU data
      float ax = imu.readFloatAccelX();
      float ay = imu.readFloatAccelY();
      float az = imu.readFloatAccelZ();
      float gx = imu.readFloatGyroX();
      float gy = imu.readFloatGyroY();
      float gz = imu.readFloatGyroZ();

      // Create string in exactly the same format as serial output
      String imuData = 
        String(ax) + "," + 
        String(ay) + "," + 
        String(az) + "," + 
        String(gx) + "," + 
        String(gy) + "," + 
        String(gz);

      // Send data via BLE characteristic
      txCharacteristic.setValue(imuData);

      // Serial print for debugging (identical to original)
      Serial.print(ax); Serial.print(",");
      Serial.print(ay); Serial.print(",");
      Serial.print(az); Serial.print(",");
      Serial.print(gx); Serial.print(",");
      Serial.print(gy); Serial.print(",");
      Serial.println(gz);

      delay(50);  // 50ms delay to match original code
    }

    Serial.println("Disconnected");
  }
}