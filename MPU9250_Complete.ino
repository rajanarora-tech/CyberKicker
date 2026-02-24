/*
 * MPU9250 Complete Test Code for Arduino Mega Pro
 * 
 * This code reads ALL data from the MPU9250:
 * - Accelerometer (X, Y, Z) - measures linear acceleration
 * - Gyroscope (X, Y, Z) - measures angular velocity
 * - Magnetometer (X, Y, Z) - measures magnetic field (compass)
 * - Temperature
 * - Calculates pitch, roll, and yaw (heading)
 * 
 * The MPU9250 is an advanced 9-axis IMU with:
 * - MPU6500 (accel + gyro)
 * - AK8963 magnetometer (on separate I2C bus)
 * 
 * Author: Test Code
 * Date: January 2026
 */

#include <Wire.h>

// MPU9250 I2C addresses
const int MPU9250_ADDR = 0x68;     // MPU9250 main sensor
const int AK8963_ADDR = 0x0C;      // Magnetometer (compass)

// MPU9250 Register addresses
const int PWR_MGMT_1 = 0x6B;
const int CONFIG = 0x1A;
const int GYRO_CONFIG = 0x1B;
const int ACCEL_CONFIG = 0x1C;
const int INT_PIN_CFG = 0x37;
const int ACCEL_XOUT_H = 0x3B;
const int TEMP_OUT_H = 0x41;
const int GYRO_XOUT_H = 0x43;
const int USER_CTRL = 0x6A;

// AK8963 (Magnetometer) Register addresses
const int MAG_CNTL = 0x0A;
const int MAG_XOUT_L = 0x03;
const int MAG_ASAX = 0x10;

// Sensor data variables
int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t mag_x, mag_y, mag_z;
int16_t temperature;

// Converted values
float accel_x_g, accel_y_g, accel_z_g;
float gyro_x_dps, gyro_y_dps, gyro_z_dps;
float mag_x_ut, mag_y_ut, mag_z_ut;
float temp_celsius;

// Magnetometer calibration (sensitivity adjustment)
float mag_scale_x = 1.0, mag_scale_y = 1.0, mag_scale_z = 1.0;

// Calibration offsets
float accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
float mag_x_offset = 0, mag_y_offset = 0, mag_z_offset = 0;

// Orientation angles
float pitch = 0, roll = 0, yaw = 0;
float heading = 0; // Compass heading in degrees

// Timing for gyro integration
unsigned long lastTime = 0;
float dt = 0;

// Filter coefficient
const float ALPHA = 0.96;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  
  Serial.println("========================================");
  Serial.println("  MPU9250 9-Axis IMU Test");
  Serial.println("  Arduino Mega Pro");
  Serial.println("========================================");
  Serial.println();
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz Fast Mode
  
  delay(100);
  
  // Check MPU9250 connection
  Serial.println("Initializing MPU9250...");
  if (!checkMPU9250Connection()) {
    Serial.println("ERROR: MPU9250 not found! Check connections.");
    Serial.println("Expected I2C address: 0x68");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("✓ MPU9250 found!");
  
  // Initialize MPU9250
  initializeMPU9250();
  
  // Initialize magnetometer
  Serial.println("\nInitializing AK8963 magnetometer...");
  if (!initializeMagnetometer()) {
    Serial.println("WARNING: Magnetometer initialization failed!");
    Serial.println("Compass readings will not be available.");
  } else {
    Serial.println("✓ Magnetometer initialized!");
  }
  
  delay(500);
  
  // Calibrate sensors
  Serial.println("\n========================================");
  Serial.println("  CALIBRATION PROCEDURE");
  Serial.println("========================================");
  Serial.println("\nFor Accelerometer & Gyroscope:");
  Serial.println("1. Place sensor on FLAT, STABLE surface");
  Serial.println("2. Keep PERFECTLY STILL");
  Serial.println("3. Wait for calibration to complete...\n");
  
  delay(2000);
  calibrateAccelGyro();
  
  Serial.println("\n✓ Accel/Gyro calibration complete!");
  Serial.println("\nFor Magnetometer (Compass):");
  Serial.println("After this message, slowly rotate the sensor");
  Serial.println("in a figure-8 pattern for 30 seconds.");
  Serial.println("This calibrates the compass.\n");
  Serial.println("Press Enter when ready...");
  
  while (!Serial.available()) {
    delay(100);
  }
  while (Serial.available()) Serial.read();
  
  calibrateMagnetometer();
  
  Serial.println("\n✓ All calibration complete!");
  Serial.println("\n========================================");
  Serial.println("  Starting Data Acquisition");
  Serial.println("========================================\n");
  
  lastTime = millis();
  delay(1000);
}

void loop() {
  // Calculate time step
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  if (dt > 0.1) dt = 0.02; // Prevent spikes
  
  // Read all sensors
  readAccelGyro();
  readMagnetometer();
  readTemperature();
  
  // Convert to physical units
  convertData();
  
  // Calculate orientation
  calculateOrientation();
  
  // Display data
  displayData();
  
  delay(100); // 10Hz update rate
}

bool checkMPU9250Connection() {
  Wire.beginTransmission(MPU9250_ADDR);
  byte error = Wire.endTransmission();
  return (error == 0);
}

void initializeMPU9250() {
  // Reset device
  writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x80);
  delay(100);
  
  // Wake up device
  writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x01); // Auto select clock source
  delay(200);
  
  // Configure gyroscope (±250 dps for best resolution)
  writeByte(MPU9250_ADDR, GYRO_CONFIG, 0x00);
  
  // Configure accelerometer (±2g for best resolution)
  writeByte(MPU9250_ADDR, ACCEL_CONFIG, 0x00);
  
  // Set Digital Low Pass Filter to 44Hz
  writeByte(MPU9250_ADDR, CONFIG, 0x03);
  
  // Enable bypass mode to access magnetometer
  writeByte(MPU9250_ADDR, INT_PIN_CFG, 0x02);
  delay(100);
}

bool initializeMagnetometer() {
  // Check magnetometer connection
  Wire.beginTransmission(AK8963_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    return false;
  }
  
  // Power down magnetometer
  writeByte(AK8963_ADDR, MAG_CNTL, 0x00);
  delay(10);
  
  // Enter fuse ROM access mode
  writeByte(AK8963_ADDR, MAG_CNTL, 0x0F);
  delay(10);
  
  // Read sensitivity adjustment values
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(MAG_ASAX);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, 3, true);
  
  uint8_t asax = Wire.read();
  uint8_t asay = Wire.read();
  uint8_t asaz = Wire.read();
  
  // Calculate adjustment scales
  mag_scale_x = (asax - 128) / 256.0 + 1.0;
  mag_scale_y = (asay - 128) / 256.0 + 1.0;
  mag_scale_z = (asaz - 128) / 256.0 + 1.0;
  
  // Power down magnetometer
  writeByte(AK8963_ADDR, MAG_CNTL, 0x00);
  delay(10);
  
  // Set continuous measurement mode (16-bit, 100Hz)
  writeByte(AK8963_ADDR, MAG_CNTL, 0x16);
  delay(10);
  
  return true;
}

void readAccelGyro() {
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 14, true);
  
  accel_x = Wire.read() << 8 | Wire.read();
  accel_y = Wire.read() << 8 | Wire.read();
  accel_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void readMagnetometer() {
  // Check if data is ready
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x02); // Status register
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, 1, true);
  uint8_t status = Wire.read();
  
  if (status & 0x01) { // Data ready
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(MAG_XOUT_L);
    Wire.endTransmission(false);
    Wire.requestFrom(AK8963_ADDR, 7, true);
    
    // Read magnetometer data (LSB first)
    uint8_t mag_xl = Wire.read();
    uint8_t mag_xh = Wire.read();
    uint8_t mag_yl = Wire.read();
    uint8_t mag_yh = Wire.read();
    uint8_t mag_zl = Wire.read();
    uint8_t mag_zh = Wire.read();
    Wire.read(); // ST2 register (must read to end data read)
    
    mag_x = (int16_t)(mag_xh << 8 | mag_xl);
    mag_y = (int16_t)(mag_yh << 8 | mag_yl);
    mag_z = (int16_t)(mag_zh << 8 | mag_zl);
  }
}

void readTemperature() {
  // Temperature is read together with accel/gyro
  // No separate read needed
}

void convertData() {
  // Convert accelerometer (±2g range, 16384 LSB/g)
  accel_x_g = (accel_x / 16384.0) - accel_x_offset;
  accel_y_g = (accel_y / 16384.0) - accel_y_offset;
  accel_z_g = (accel_z / 16384.0) - accel_z_offset;
  
  // Convert gyroscope (±250 dps range, 131 LSB/dps)
  gyro_x_dps = (gyro_x / 131.0) - gyro_x_offset;
  gyro_y_dps = (gyro_y / 131.0) - gyro_y_offset;
  gyro_z_dps = (gyro_z / 131.0) - gyro_z_offset;
  
  // Convert magnetometer (±4800 µT range, 0.6 µT/LSB for 16-bit)
  mag_x_ut = (mag_x * mag_scale_x * 0.6) - mag_x_offset;
  mag_y_ut = (mag_y * mag_scale_y * 0.6) - mag_y_offset;
  mag_z_ut = (mag_z * mag_scale_z * 0.6) - mag_z_offset;
  
  // Convert temperature (TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC)
  temp_celsius = ((temperature - 0) / 333.87) + 21.0;
}

void calculateOrientation() {
  // Calculate pitch and roll from accelerometer
  float accel_pitch = atan2(accel_y_g, sqrt(accel_x_g * accel_x_g + accel_z_g * accel_z_g)) * 180.0 / PI;
  float accel_roll = atan2(-accel_x_g, accel_z_g) * 180.0 / PI;
  
  // Complementary filter for pitch and roll
  pitch = ALPHA * (pitch + gyro_x_dps * dt) + (1 - ALPHA) * accel_pitch;
  roll = ALPHA * (roll + gyro_y_dps * dt) + (1 - ALPHA) * accel_roll;
  yaw += gyro_z_dps * dt;
  
  // Normalize angles to 0-360
  pitch = normalizeAngle(pitch);
  roll = normalizeAngle(roll);
  yaw = normalizeAngle(yaw);
  
  // Calculate magnetic heading (compass)
  // Tilt compensation
  float mag_x_comp = mag_x_ut * cos(pitch * PI / 180.0) + 
                     mag_z_ut * sin(pitch * PI / 180.0);
  float mag_y_comp = mag_x_ut * sin(roll * PI / 180.0) * sin(pitch * PI / 180.0) + 
                     mag_y_ut * cos(roll * PI / 180.0) - 
                     mag_z_ut * sin(roll * PI / 180.0) * cos(pitch * PI / 180.0);
  
  heading = atan2(mag_y_comp, mag_x_comp) * 180.0 / PI;
  heading = normalizeAngle(heading);
}

float normalizeAngle(float angle) {
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

void calibrateAccelGyro() {
  const int samples = 500;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  
  Serial.print("Calibrating");
  
  // Discard first readings
  for (int i = 0; i < 20; i++) {
    readAccelGyro();
    delay(10);
  }
  
  // Collect samples
  for (int i = 0; i < samples; i++) {
    readAccelGyro();
    
    sum_ax += accel_x / 16384.0;
    sum_ay += accel_y / 16384.0;
    sum_az += accel_z / 16384.0;
    sum_gx += gyro_x / 131.0;
    sum_gy += gyro_y / 131.0;
    sum_gz += gyro_z / 131.0;
    
    if (i % 50 == 0) Serial.print(".");
    delay(5);
  }
  Serial.println();
  
  accel_x_offset = sum_ax / samples;
  accel_y_offset = sum_ay / samples;
  accel_z_offset = (sum_az / samples) - 1.0; // Subtract 1g
  gyro_x_offset = sum_gx / samples;
  gyro_y_offset = sum_gy / samples;
  gyro_z_offset = sum_gz / samples;
}

void calibrateMagnetometer() {
  Serial.println("Rotate sensor in figure-8 for 30 seconds...");
  Serial.println("Starting in 3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("GO! Rotate now!");
  
  float mag_x_min = 32767, mag_x_max = -32768;
  float mag_y_min = 32767, mag_y_max = -32768;
  float mag_z_min = 32767, mag_z_max = -32768;
  
  unsigned long startTime = millis();
  while (millis() - startTime < 30000) {
    readMagnetometer();
    
    float mx = mag_x * mag_scale_x;
    float my = mag_y * mag_scale_y;
    float mz = mag_z * mag_scale_z;
    
    if (mx < mag_x_min) mag_x_min = mx;
    if (mx > mag_x_max) mag_x_max = mx;
    if (my < mag_y_min) mag_y_min = my;
    if (my > mag_y_max) mag_y_max = my;
    if (mz < mag_z_min) mag_z_min = mz;
    if (mz > mag_z_max) mag_z_max = mz;
    
    if ((millis() - startTime) % 1000 == 0) {
      Serial.print(".");
    }
    
    delay(10);
  }
  Serial.println();
  
  mag_x_offset = ((mag_x_max + mag_x_min) / 2.0) * 0.6;
  mag_y_offset = ((mag_y_max + mag_y_min) / 2.0) * 0.6;
  mag_z_offset = ((mag_z_max + mag_z_min) / 2.0) * 0.6;
  
  Serial.println("Magnetometer calibration complete!");
}

void displayData() {
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay < 500) return;
  lastDisplay = millis();
  
  Serial.println("========================================");
  Serial.println("     MPU9250 9-AXIS SENSOR DATA");
  Serial.println("========================================");
  Serial.println();
  
  // Accelerometer
  Serial.println("ACCELEROMETER (g):");
  Serial.print("  X: "); printValue(accel_x_g, 3);
  Serial.print("  Y: "); printValue(accel_y_g, 3);
  Serial.print("  Z: "); printValue(accel_z_g, 3);
  Serial.println();
  Serial.println();
  
  // Gyroscope
  Serial.println("GYROSCOPE (deg/s):");
  Serial.print("  X: "); printValue(gyro_x_dps, 2);
  Serial.print("  Y: "); printValue(gyro_y_dps, 2);
  Serial.print("  Z: "); printValue(gyro_z_dps, 2);
  Serial.println();
  Serial.println();
  
  // Magnetometer
  Serial.println("MAGNETOMETER (uT):");
  Serial.print("  X: "); printValue(mag_x_ut, 1);
  Serial.print("  Y: "); printValue(mag_y_ut, 1);
  Serial.print("  Z: "); printValue(mag_z_ut, 1);
  Serial.println();
  Serial.println();
  
  // Orientation
  Serial.println("ORIENTATION (0-360 deg):");
  Serial.print("  Pitch: "); printAngle(pitch);
  Serial.print("  Roll:  "); printAngle(roll);
  Serial.print("  Yaw:   "); printAngle(yaw);
  Serial.println();
  Serial.println();
  
  // Compass Heading
  Serial.println("COMPASS:");
  Serial.print("  Heading: ");
  printAngle(heading);
  Serial.print("  (");
  printCardinalDirection(heading);
  Serial.println(")");
  Serial.println();
  
  // Temperature
  Serial.print("TEMPERATURE: ");
  Serial.print(temp_celsius, 1);
  Serial.println(" C");
  Serial.println();
  
  // Raw values
  Serial.println("RAW VALUES:");
  Serial.print("  Accel: X="); Serial.print(accel_x);
  Serial.print(" Y="); Serial.print(accel_y);
  Serial.print(" Z="); Serial.println(accel_z);
  Serial.print("  Gyro:  X="); Serial.print(gyro_x);
  Serial.print(" Y="); Serial.print(gyro_y);
  Serial.print(" Z="); Serial.println(gyro_z);
  Serial.print("  Mag:   X="); Serial.print(mag_x);
  Serial.print(" Y="); Serial.print(mag_y);
  Serial.print(" Z="); Serial.println(mag_z);
  
  Serial.println();
  Serial.println("========================================");
  Serial.println();
}

void printValue(float value, int decimals) {
  if (value >= 0) Serial.print(" ");
  Serial.print(value, decimals);
}

void printAngle(float angle) {
  if (angle < 10) Serial.print("  ");
  else if (angle < 100) Serial.print(" ");
  Serial.print(angle, 1);
  Serial.print(" deg");
}

void printCardinalDirection(float heading) {
  if (heading >= 337.5 || heading < 22.5) Serial.print("N ");
  else if (heading >= 22.5 && heading < 67.5) Serial.print("NE");
  else if (heading >= 67.5 && heading < 112.5) Serial.print("E ");
  else if (heading >= 112.5 && heading < 157.5) Serial.print("SE");
  else if (heading >= 157.5 && heading < 202.5) Serial.print("S ");
  else if (heading >= 202.5 && heading < 247.5) Serial.print("SW");
  else if (heading >= 247.5 && heading < 292.5) Serial.print("W ");
  else if (heading >= 292.5 && heading < 337.5) Serial.print("NW");
}

void writeByte(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
