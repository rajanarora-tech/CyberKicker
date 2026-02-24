/*
 * MPU9250_Sensor.h
 * 
 * Class for handling MPU9250 IMU sensor
 * Extracts and processes gyroscope data for heading control
 */

#ifndef MPU9250_SENSOR_H
#define MPU9250_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

class MPU9250_Sensor {
private:
  // MPU9250 I2C addresses
  static const int MPU9250_ADDR = 0x68;
  static const int AK8963_ADDR = 0x0C;
  
  // Register addresses
  static const int PWR_MGMT_1 = 0x6B;
  static const int CONFIG = 0x1A;
  static const int GYRO_CONFIG = 0x1B;
  static const int ACCEL_CONFIG = 0x1C;
  static const int INT_PIN_CFG = 0x37;
  static const int ACCEL_XOUT_H = 0x3B;
  static const int GYRO_XOUT_H = 0x43;
  
  // Raw sensor data
  int16_t accel_x, accel_y, accel_z;
  int16_t gyro_x, gyro_y, gyro_z;
  int16_t temperature;
  
  // Converted values
  float accel_x_g, accel_y_g, accel_z_g;
  float gyro_x_dps, gyro_y_dps, gyro_z_dps;
  
  // Calibration offsets
  float gyro_z_offset;
  
  // Orientation
  float currentHeading;
  float initialHeading;
  
  // Timing
  unsigned long lastTime;
  float dt;
  
  // Helper functions
  void writeByte(uint8_t address, uint8_t reg, uint8_t data);
  void readAccelGyro();
  void convertData();
  float normalizeAngle(float angle);
  
public:
  MPU9250_Sensor();
  
  bool begin();
  void calibrate();
  void update();
  void resetHeading();
  
  float getCurrentHeading();
  float getHeadingError();
  float getGyroZ();
  
  // Getters for debugging
  float getAccelX() { return accel_x_g; }
  float getAccelY() { return accel_y_g; }
  float getAccelZ() { return accel_z_g; }
};

#endif
