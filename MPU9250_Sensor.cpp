/*
 * MPU9250_Sensor.cpp
 * 
 * Implementation of MPU9250 IMU sensor class
 */

#include "MPU9250_Sensor.h"

MPU9250_Sensor::MPU9250_Sensor() {
  accel_x = accel_y = accel_z = 0;
  gyro_x = gyro_y = gyro_z = 0;
  temperature = 0;
  
  accel_x_g = accel_y_g = accel_z_g = 0;
  gyro_x_dps = gyro_y_dps = gyro_z_dps = 0;
  
  gyro_z_offset = 0;
  
  currentHeading = 0;
  initialHeading = 0;
  
  lastTime = 0;
  dt = 0;
}

bool MPU9250_Sensor::begin() {
  // Check MPU9250 connection
  Wire.beginTransmission(MPU9250_ADDR);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // Reset device
  writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x80);
  delay(100);
  
  // Wake up device
  writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x01);
  delay(200);
  
  // Configure gyroscope (±250 dps for best resolution)
  writeByte(MPU9250_ADDR, GYRO_CONFIG, 0x00);
  
  // Configure accelerometer (±2g for best resolution)
  writeByte(MPU9250_ADDR, ACCEL_CONFIG, 0x00);
  
  // Set Digital Low Pass Filter to 44Hz
  writeByte(MPU9250_ADDR, CONFIG, 0x03);
  
  lastTime = millis();
  
  return true;
}

void MPU9250_Sensor::calibrate() {
  const int samples = 500;
  float sum_gz = 0;
  
  // Discard first readings
  for (int i = 0; i < 20; i++) {
    readAccelGyro();
    delay(10);
  }
  
  // Collect samples
  for (int i = 0; i < samples; i++) {
    readAccelGyro();
    sum_gz += gyro_z / 131.0;  // Convert to dps
    delay(5);
  }
  
  gyro_z_offset = sum_gz / samples;
}

void MPU9250_Sensor::update() {
  // Calculate time step
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Prevent spikes on first call or after long delays
  if (dt > 0.1 || dt <= 0) {
    dt = 0.02;
  }
  
  // Read sensor
  readAccelGyro();
  
  // Convert to physical units
  convertData();
  
  // Integrate gyro to get heading
  currentHeading += gyro_z_dps * dt;
  
  // Normalize to 0-360
  currentHeading = normalizeAngle(currentHeading);
}

void MPU9250_Sensor::resetHeading() {
  initialHeading = currentHeading;
}

float MPU9250_Sensor::getCurrentHeading() {
  return currentHeading;
}

float MPU9250_Sensor::getHeadingError() {
  float error = currentHeading - initialHeading;
  
  // Normalize error to -180 to +180
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  
  return error;
}

float MPU9250_Sensor::getGyroZ() {
  return gyro_z_dps;
}

void MPU9250_Sensor::readAccelGyro() {
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

void MPU9250_Sensor::convertData() {
  // Convert accelerometer (±2g range, 16384 LSB/g)
  accel_x_g = accel_x / 16384.0;
  accel_y_g = accel_y / 16384.0;
  accel_z_g = accel_z / 16384.0;
  
  // Convert gyroscope (±250 dps range, 131 LSB/dps)
  gyro_x_dps = (gyro_x / 131.0) - 0;  // No offset for X
  gyro_y_dps = (gyro_y / 131.0) - 0;  // No offset for Y
  gyro_z_dps = (gyro_z / 131.0) - gyro_z_offset;  // Apply Z offset
}

float MPU9250_Sensor::normalizeAngle(float angle) {
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

void MPU9250_Sensor::writeByte(uint8_t address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}
