/*
 * MPU6050 Ultra-Stable Test Code for Arduino Mega Pro
 * 
 * Features:
 * - Angles constrained to 0-360° range
 * - Multiple filtering stages for maximum stability
 * - Dead zone for near-zero values
 * - Complementary filter with gyroscope integration
 * - Hardware and software noise reduction
 * 
 * Author: Test Code
 * Date: January 2026
 */

#include <Wire.h>

// MPU6050 I2C address
const int MPU6050_ADDR = 0x68;

// MPU6050 Register addresses
const int PWR_MGMT_1 = 0x6B;
const int ACCEL_XOUT_H = 0x3B;
const int CONFIG = 0x1A;

// Variables to store sensor data
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;

// Converted values
float accel_x_g, accel_y_g, accel_z_g;
float gyro_x_dps, gyro_y_dps, gyro_z_dps;
float temp_celsius;

// Stable filtered angles (0-360 degrees)
float pitch = 0, roll = 0, yaw = 0;

// Calibration offsets
float accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

// Timing for gyro integration
unsigned long lastTime = 0;
float dt = 0;

// Filter coefficients
const float COMPLEMENTARY_ALPHA = 0.98;  // Trust gyro more (reduces jitter)
const float LOW_PASS_ALPHA = 0.85;       // Smoothing for accelerometer

// Dead zone to eliminate micro-fluctuations
const float GYRO_DEADZONE = 0.5;         // Ignore gyro rates below this (°/s)
const float ACCEL_DEADZONE = 0.02;       // Ignore accel changes below this (g)
const float ANGLE_DEADZONE = 0.3;        // Ignore angle changes below this (°)

// Moving average buffers
const int MA_SIZE = 10;
float accel_x_history[MA_SIZE] = {0};
float accel_y_history[MA_SIZE] = {0};
float accel_z_history[MA_SIZE] = {0};
float gyro_x_history[MA_SIZE] = {0};
float gyro_y_history[MA_SIZE] = {0};
float gyro_z_history[MA_SIZE] = {0};
int history_index = 0;

// Filtered values
float filtered_accel_x = 0, filtered_accel_y = 0, filtered_accel_z = 0;
float filtered_gyro_x = 0, filtered_gyro_y = 0, filtered_gyro_z = 0;

// Previous angle values for dead zone comparison
float prev_pitch = 0, prev_roll = 0, prev_yaw = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  
  Serial.println("MPU6050 Ultra-Stable with Angle Limiting");
  Serial.println("=========================================");
  Serial.println();
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
  
  // Set DLPF to maximum filtering (5Hz bandwidth - very smooth)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x06); // DLPF_CFG = 6: 5Hz bandwidth (most stable)
  Wire.endTransmission(true);
  
  // Configure accelerometer (±2g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  // Configure gyroscope (±250°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  Serial.println("MPU6050 initialized with maximum stability!");
  Serial.println();
  Serial.println("CALIBRATION INSTRUCTIONS:");
  Serial.println("1. Place sensor on flat, stable surface");
  Serial.println("2. Do NOT touch or move the sensor");
  Serial.println("3. Wait for calibration to complete...");
  Serial.println();
  
  delay(2000);
  calibrateSensor();
  
  Serial.println("\nCalibration complete!");
  Serial.println("Starting ultra-stable readings...");
  Serial.println("Angles will be displayed in 0-360° range");
  Serial.println("=========================================");
  Serial.println();
  
  lastTime = millis();
  delay(1000);
}

void loop() {
  // Calculate time difference
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Limit dt to prevent spikes on first run
  if (dt > 0.1) dt = 0.02;
  
  // Read and process sensor data
  readMPU6050();
  applyMovingAverage();
  convertData();
  applyLowPassFilter();
  applyDeadZone();
  calculateStableAngles();
  
  // Display data periodically
  displayData();
  
  delay(20); // 50Hz update rate
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void applyMovingAverage() {
  // Store in circular buffer
  accel_x_history[history_index] = accelerometer_x;
  accel_y_history[history_index] = accelerometer_y;
  accel_z_history[history_index] = accelerometer_z;
  gyro_x_history[history_index] = gyro_x;
  gyro_y_history[history_index] = gyro_y;
  gyro_z_history[history_index] = gyro_z;
  
  history_index = (history_index + 1) % MA_SIZE;
  
  // Calculate averages
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  
  for (int i = 0; i < MA_SIZE; i++) {
    sum_ax += accel_x_history[i];
    sum_ay += accel_y_history[i];
    sum_az += accel_z_history[i];
    sum_gx += gyro_x_history[i];
    sum_gy += gyro_y_history[i];
    sum_gz += gyro_z_history[i];
  }
  
  accelerometer_x = sum_ax / MA_SIZE;
  accelerometer_y = sum_ay / MA_SIZE;
  accelerometer_z = sum_az / MA_SIZE;
  gyro_x = sum_gx / MA_SIZE;
  gyro_y = sum_gy / MA_SIZE;
  gyro_z = sum_gz / MA_SIZE;
}

void convertData() {
  // Convert to physical units
  accel_x_g = (accelerometer_x / 16384.0) - accel_x_offset;
  accel_y_g = (accelerometer_y / 16384.0) - accel_y_offset;
  accel_z_g = (accelerometer_z / 16384.0) - accel_z_offset;
  
  gyro_x_dps = (gyro_x / 131.0) - gyro_x_offset;
  gyro_y_dps = (gyro_y / 131.0) - gyro_y_offset;
  gyro_z_dps = (gyro_z / 131.0) - gyro_z_offset;
  
  temp_celsius = (temperature / 340.0) + 36.53;
}

void applyLowPassFilter() {
  // Exponential moving average (low-pass filter)
  filtered_accel_x = LOW_PASS_ALPHA * filtered_accel_x + (1 - LOW_PASS_ALPHA) * accel_x_g;
  filtered_accel_y = LOW_PASS_ALPHA * filtered_accel_y + (1 - LOW_PASS_ALPHA) * accel_y_g;
  filtered_accel_z = LOW_PASS_ALPHA * filtered_accel_z + (1 - LOW_PASS_ALPHA) * accel_z_g;
  
  filtered_gyro_x = LOW_PASS_ALPHA * filtered_gyro_x + (1 - LOW_PASS_ALPHA) * gyro_x_dps;
  filtered_gyro_y = LOW_PASS_ALPHA * filtered_gyro_y + (1 - LOW_PASS_ALPHA) * gyro_y_dps;
  filtered_gyro_z = LOW_PASS_ALPHA * filtered_gyro_z + (1 - LOW_PASS_ALPHA) * gyro_z_dps;
}

void applyDeadZone() {
  // Apply dead zone to gyroscope (eliminate drift)
  if (abs(filtered_gyro_x) < GYRO_DEADZONE) filtered_gyro_x = 0;
  if (abs(filtered_gyro_y) < GYRO_DEADZONE) filtered_gyro_y = 0;
  if (abs(filtered_gyro_z) < GYRO_DEADZONE) filtered_gyro_z = 0;
  
  // Apply dead zone to accelerometer
  if (abs(filtered_accel_x) < ACCEL_DEADZONE) filtered_accel_x = 0;
  if (abs(filtered_accel_y) < ACCEL_DEADZONE) filtered_accel_y = 0;
}

void calculateStableAngles() {
  // Calculate angles from accelerometer
  float accel_pitch = atan2(filtered_accel_y, sqrt(filtered_accel_x * filtered_accel_x + 
                           filtered_accel_z * filtered_accel_z)) * 180.0 / PI;
  float accel_roll = atan2(-filtered_accel_x, filtered_accel_z) * 180.0 / PI;
  
  // Integrate gyroscope for smooth updates
  pitch = COMPLEMENTARY_ALPHA * (pitch + filtered_gyro_y * dt) + 
          (1 - COMPLEMENTARY_ALPHA) * accel_pitch;
  roll = COMPLEMENTARY_ALPHA * (roll - filtered_gyro_x * dt) + 
         (1 - COMPLEMENTARY_ALPHA) * accel_roll;
  yaw += filtered_gyro_z * dt; // Yaw integrates over time
  
  // Apply angle dead zone to prevent micro-jitter
  if (abs(pitch - prev_pitch) < ANGLE_DEADZONE) pitch = prev_pitch;
  if (abs(roll - prev_roll) < ANGLE_DEADZONE) roll = prev_roll;
  if (abs(yaw - prev_yaw) < ANGLE_DEADZONE) yaw = prev_yaw;
  
  // Normalize angles to 0-360° range
  pitch = normalizeAngle(pitch);
  roll = normalizeAngle(roll);
  yaw = normalizeAngle(yaw);
  
  // Store for next comparison
  prev_pitch = pitch;
  prev_roll = roll;
  prev_yaw = yaw;
}

float normalizeAngle(float angle) {
  // Constrain angle to 0-360° range
  while (angle < 0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  
  // Round to 1 decimal place to prevent micro-changes
  angle = round(angle * 10.0) / 10.0;
  
  return angle;
}

void calibrateSensor() {
  const int numSamples = 500;
  float sum_accel_x = 0, sum_accel_y = 0, sum_accel_z = 0;
  float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
  
  Serial.println("Calibrating");
  
  // Discard initial unstable readings
  for (int i = 0; i < 20; i++) {
    readMPU6050();
    delay(10);
  }
  
  // Collect calibration samples
  for (int i = 0; i < numSamples; i++) {
    readMPU6050();
    
    sum_accel_x += accelerometer_x / 16384.0;
    sum_accel_y += accelerometer_y / 16384.0;
    sum_accel_z += accelerometer_z / 16384.0;
    
    sum_gyro_x += gyro_x / 131.0;
    sum_gyro_y += gyro_y / 131.0;
    sum_gyro_z += gyro_z / 131.0;
    
    if (i % 50 == 0) Serial.print(".");
    delay(5);
  }
  
  // Calculate offsets
  accel_x_offset = sum_accel_x / numSamples;
  accel_y_offset = sum_accel_y / numSamples;
  accel_z_offset = (sum_accel_z / numSamples) - 1.0;
  
  gyro_x_offset = sum_gyro_x / numSamples;
  gyro_y_offset = sum_gyro_y / numSamples;
  gyro_z_offset = sum_gyro_z / numSamples;
  
  // Initialize filtered values
  filtered_accel_x = 0;
  filtered_accel_y = 0;
  filtered_accel_z = 1.0;
  filtered_gyro_x = 0;
  filtered_gyro_y = 0;
  filtered_gyro_z = 0;
  
  // Initialize angles to 0
  pitch = 0;
  roll = 0;
  yaw = 0;
  prev_pitch = 0;
  prev_roll = 0;
  prev_yaw = 0;
}

void displayData() {
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay < 500) return;
  lastDisplay = millis();
  
  Serial.println("=== STABLE MPU6050 DATA (0-360°) ===");
  Serial.println();
  
  // Orientation angles
  Serial.println("ORIENTATION (degrees, 0-360):");
  Serial.print("  Pitch: "); 
  printAngle(pitch);
  Serial.print("  Roll:  "); 
  printAngle(roll);
  Serial.print("  Yaw:   "); 
  printAngle(yaw);
  Serial.println();
  
  // Accelerometer
  Serial.println("ACCELEROMETER (g):");
  Serial.print("  X: "); printValue(filtered_accel_x, 3);
  Serial.print("  Y: "); printValue(filtered_accel_y, 3);
  Serial.print("  Z: "); printValue(filtered_accel_z, 3);
  Serial.println();
  
  // Gyroscope
  Serial.println("GYROSCOPE (°/s):");
  Serial.print("  X: "); printValue(filtered_gyro_x, 2);
  Serial.print("  Y: "); printValue(filtered_gyro_y, 2);
  Serial.print("  Z: "); printValue(filtered_gyro_z, 2);
  Serial.println();
  
  // Temperature
  Serial.print("TEMPERATURE: ");
  Serial.print(temp_celsius, 1);
  Serial.println(" °C");
  Serial.println();
  
  Serial.println("====================================");
  Serial.println();
}

void printAngle(float angle) {
  if (angle < 10) Serial.print("  ");
  else if (angle < 100) Serial.print(" ");
  Serial.print(angle, 1);
  Serial.println("°");
}

void printValue(float value, int decimals) {
  if (value >= 0) Serial.print(" ");
  Serial.print(value, decimals);
  Serial.print("  ");
}
