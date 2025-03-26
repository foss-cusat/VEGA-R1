// Referencing Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
//

int counter = 0;

// Instantiating Servo class
Servo yawServo; // top servo
Servo pitchServo; // bottom servo
Servo rollServo;
//

// Setting servo limits
float yawMin = 65; // top servo
float yawMax = 135;
float pitchMin = 65; // bottom servo
float pitchMax = 140;
//

// setting Quaternion variables
float q0; // w
float q1; // i
float q2; // j
float q3; // k
//

// Setting PID control coefficients
//float Kp = 1; // Proportional
//float Kd = 2; // Derivative
//

// Setting time variables
int milliOld;
int milliNew;
int dt;
//

// Setting Roll variables
float prevRollAngle;
float rollAngle;
float rollAngleChange;
float rollActual;
//

// Setting Pitch variables
float pitchTarget = 90;
float pitchActual;
float pitchError = 0;
float pitchErrorOld;
float pitchErrorChange;
float pitchErrorSlope = 0;
float pitchErrorArea = 0;
float pitchServoVal = 90;
//

// Setting Yaw variables
float yawTarget = 90;
float yawActual;
float yawError = 0;
float yawErrorOld;
float yawErrorChange;
float yawErrorSlope = 0;
float yawErrorArea = 0;
float yawServoVal = 90;
//

// Setting SD card pin
const int chipSelect = 10;
//

// Assigning LED pins
const int blueLED = 4;
const int greenLED = 5;
const int whiteLED = 6;
const int orangeLED = 7;
const int yellowLED = 8;
//

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);

  // Configuring LED pins
  pinMode(blueLED, OUTPUT);
  digitalWrite(blueLED, LOW);
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED, LOW);
  pinMode(whiteLED, OUTPUT);
  digitalWrite(whiteLED, LOW);
  pinMode(orangeLED, OUTPUT);
  digitalWrite(orangeLED, LOW);
  pinMode(yellowLED, OUTPUT);
  digitalWrite(yellowLED, LOW);
  //

  Serial.println("Welcome to the Flight Computer setup!");
  delay(500) ;

//  // Checking if SD card is working
//  Serial.println("Let's make sure SD card is present");
//  if (!SD.begin(chipSelect)) {
//    Serial.println("Card failed or not present, try again");
//    while (1) {}
//  }
//  else {
//    digitalWrite(yellowLED, HIGH);
//    Serial.println("Card initialized");
//  }
  

  delay(1000) ;

  // Initializing the IMU
  myIMU.begin();
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
  delay(1000);
  //

  // Calibrating BNO055 IMU
  Serial.println("Now let's calibrate the BNO055 IMU");
  delay(1000) ;

  if (myIMU.begin())
  {
    Serial.println("BNO055 Initialized");
  }
  else {
    Serial.println("No BNO055. Try again!");
    while (1) ;
  }

  Serial.println("Calibrating Inertials...");
  uint8_t system, gyro, accel, mg = 0;
  delay(1000) ;

//  while (1) {
//    myIMU.getCalibration(&system, &gyro, &accel, &mg) ;
//    Serial.print("System: ");
//    Serial.print(system);
//    Serial.print(", Gyro: ");
//    Serial.print(gyro);
//    Serial.print(", Accel: ");
//    Serial.print(accel);
//    Serial.print(", Mg: ");
//    Serial.println(mg);
//    if (system == 3)  {
//      digitalWrite(blueLED, HIGH);
//    }
//    else if (gyro == 3)  {
//      digitalWrite(greenLED, HIGH);
//    }
//    else if (accel == 3)  {
//      digitalWrite(whiteLED, HIGH);
//    }
//    else if (mg == 3)  {
//      digitalWrite(orangeLED, HIGH);
//    }
//    if (gyro == 3 && mg == 3 ) {
//      Serial.println("Inertials Calibrated");
//      break ;
//    }
//    if (counter > 120) {
//      Serial.println("Inertial Calibration Failed... Try again");
//      while (1);
//    }
//    delay(1000);
//    counter += 1;
//  }
  delay(3000);

  // Attaching servos to respective pins on the Arduino
  yawServo.attach(2);
  pitchServo.attach(3);
  //

  // Set initial servo values - 90 means motor is pointed straight down
  yawServo.write(yawServoVal);
  delay(100);
  pitchServo.write(pitchServoVal);
  delay(100);
  //

  //   Calibrating IMU
  //    uint8_t system, gyro, accel, mg = 0;
  //    myIMU.getCalibration(&system, &gyro, &accel, &mg);
  //        while (system != 3 || gyro != 3 || accel != 3 || mg != 3) // loops until system is calibrated
  //        {
  //          if (system == 3)  {
  //            digitalWrite(blueLED, HIGH);
  //          }
  //          else if (gyro == 3)  {
  //            digitalWrite(greenLED, HIGH);
  //          }
  //          else if (accel == 3)  {
  //            digitalWrite(whiteLED, HIGH);
  //          }
  //          else if (mg == 3)  {
  //            digitalWrite(orangeLED, HIGH);
  //          }
  //          delay(500);
  //        }
  //

  delay(2000);

  // Turn LEDs off - ready for launch
  digitalWrite(blueLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(whiteLED, LOW);
  digitalWrite(orangeLED, LOW);
  digitalWrite(yellowLED, LOW);
  //
  delay(1000);
  Serial.println("Setup Complete. Moving to ground state...");

  milliNew = millis();
}

void loop() {

//  // Open file on SD card
//  File dataFile = SD.open("FlightData.txt", FILE_WRITE);
//  //

  // Obtaining Quaternions
  imu::Quaternion quat = myIMU.getQuat();
  q0 = quat.w();
  q1 = quat.x();
  q2 = quat.y();
  q3 = quat.z();
  //

  // Converting Quaternions to Euler angles
  yawActual = asin(2 * (q0 * q2 - q3 * q1));
  pitchActual = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
  rollActual = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));


  rollActual = rollActual * 180 / 3.141592654;
  pitchActual = pitchActual * 180 / 3.141592654;
  yawActual = yawActual * 180 / 3.141592654;
  //

  // Determining change in time since last iteration
  milliOld = milliNew;
  milliNew = millis();
  dt = milliNew - milliOld;
  //

  // Calculating Pitch PID values
  pitchErrorOld = pitchError;
  pitchError = (pitchTarget - pitchActual);
  pitchErrorChange = pitchError - pitchErrorOld;
  pitchErrorSlope = pitchErrorChange / dt;
  pitchServoVal = pitchError + 90;
  //  pitchServoVal =  Kp * pitchError + Kd * pitchErrorSlope;
  //

  // Calculating Yaw PID values
  yawErrorOld = yawError;
  yawError = (yawTarget - yawActual);
  yawErrorChange = yawError - yawErrorOld;
  yawErrorSlope = yawErrorChange / dt;
  yawServoVal = yawError;
  //  yawServoVal =  Kp * yawError + Kd * yawErrorSlope;
  //

  // Taking into account Roll
  //  rollAngle = rollActual;
  //  rollAngleChange = rollAngle - prevRollAngle;
  //  prevRollAngle = rollAngle;
  //  yawServoVal += rollAngleChange;
  //  pitchServoVal += rollAngleChange;
  //

  // Limiting servo positions
  if (yawServoVal > yawMax) {
    yawServoVal = yawMax;
  }
  if (yawServoVal < yawMin) {
    yawServoVal = yawMin;
  }
  if (pitchServoVal > pitchMax) {
    pitchServoVal = pitchMax;
  }
  if (pitchServoVal < pitchMin) {
    pitchServoVal = pitchMin;
  }
  //

  // Updating servo positions
  yawServo.write(yawServoVal);
  pitchServo.write(pitchServoVal);
  //

  //  Serial.print(q0);
  //  Serial.print(",");
  //  Serial.print(q1);
  //  Serial.print(",");
  //  Serial.print(q2);
  //  Serial.print(",");
  //  Serial.print(q3);
  //  Serial.println(",");

  //  Serial.print(dt);
  //  Serial.print(",");
  Serial.print(yawActual);
  Serial.print(",");
  Serial.print(pitchActual);
  Serial.print(",");
  Serial.print(rollActual);
  Serial.print(",");
  Serial.print(yawServoVal);
  Serial.print(",");
  Serial.print(pitchServoVal);
  Serial.println(",");
  //  Serial.print(accel);
  //  Serial.print(",");
  //  Serial.print(gyro);
  //  Serial.print(",");
  //  Serial.print(mg);
  //  Serial.print(",");
  //  Serial.println(system);

//  //Write data to file on SD card
//  dataFile.print("DeltaT: "); dataFile.print(dt);
//  dataFile.print("Quat[w,x,y,z]: "); dataFile.print(", "); dataFile.print(q0); dataFile.print(", "); dataFile.print(q1); dataFile.print(", "); dataFile.print(q2); dataFile.print(", "); dataFile.print(q3);
//  dataFile.print("Pitch Servo Pos: "); dataFile.print(", "); dataFile.print(pitchServoVal);
//  dataFile.print(" Servo Pos: "); dataFile.print(", "); dataFile.print(yawServoVal);
//
//  // Close file on SD card
//  dataFile.close();


  // Forcing the loop to not exceed IMU sample rate
  //  delay(BNO055_SAMPLERATE_DELAY_MS);
  //
}
