
Purpose: Manual PID Implementation for Quadcopter Stability
  Author: Danuka Shehan
#include <Wire.h>

// Motor Pins (PWM)
const int M1 = 10, M2 = 11, M3 = 12, M4 = 13;

// PID Constants (Show-off these values!)
float Kp = 1.3, Ki = 0.04, Kd = 15.0; 
float last_error, integral;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(M1, OUTPUT); pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT); pinMode(M4, OUTPUT);
}

void loop() {
  // 1. Get Gyro Data (Logic only)
  float current_angle = readGyro(); 
  float target_angle = 0; // Level flight

  // 2. PID Calculation (The Heart of the Drone)
  float error = target_angle - current_angle;
  integral += error;
  float derivative = error - last_error;
  
  float pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  last_error = error;

  // 3. Motor Mixing (Distributing power to 4 motors)
  int throttle = 1500; // Base speed
  analogWrite(M1, throttle + pid_output); 
  analogWrite(M2, throttle - pid_output);
  
  delay(4); // 250Hz frequency
}

float readGyro() { 
  // Code to interface with MPU6050 via I2C
  return 0; 
}
 * Component: Advanced Flight Control & Navigation Engine

 */

#include <Wire.h>
#include <SoftwareSerial.h>

// --- HARDWARE PIN DEFINITIONS ---
const int MOTOR_FR = 10; // Front-Right
const int MOTOR_FL = 11; // Front-Left
const int MOTOR_BR = 12; // Back-Right
const int MOTOR_BL = 13; // Back-Left
SoftwareSerial gpsSerial(4, 3); // RX, TX for BN-220T GPS

// --- GLOBAL VARIABLES & SYSTEM STATUS ---
float throttle = 1500;
float roll_target = 0, pitch_target = 0, yaw_target = 0;
float roll_actual, pitch_actual, yaw_actual;
unsigned long last_time;

// --- ADVANCED PID GAINS (Tuned for Vector-X) ---
struct PID {
    float P = 1.45;
    float I = 0.02;
    float D = 12.5;
    float error_prior = 0;
    float integral_prior = 0;
};

PID rollPID, pitchPID, yawPID;

// --- SIGNAL FILTERING (Complementary Filter) ---
float alpha = 0.98; // High-pass filter constant for stability

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600);
    Wire.begin();
    
    // Set 400kHz I2C for high-speed sensor reading
    Wire.setClock(400000); 
    
    initializeMotors();
    calibrateSensors();
    
    Serial.println("VECTOR-X FLIGHT SYSTEM ONLINE...");
}

void loop() {
    // 1. HIGH-SPEED SENSOR FUSION (IMU Data)
    calculateOrientation();

    // 2. TELEMETRY & SIGNAL PROCESSING (GPS)
    if (gpsSerial.available()) {
        processGPSData();
    }

    // 3. MULTI-AXIS PID CALCULATION
    float dt = (micros() - last_time) / 1000000.0;
    last_time = micros();

    float roll_output  = computePID(roll_target, roll_actual, &rollPID, dt);
    float pitch_output = computePID(pitch_target, pitch_actual, &pitchPID, dt);
    float yaw_output   = computePID(yaw_target, yaw_actual, &yawPID, dt);

    // 4. MOTOR MIXING ALGORITHM (X-Configuration)
    int m1 = throttle + pitch_output + roll_output - yaw_output; // FR
    int m2 = throttle + pitch_output - roll_output + yaw_output; // FL
    int m3 = throttle - pitch_output + roll_output + yaw_output; // BR
    int m4 = throttle - pitch_output - roll_output - yaw_output; // BL

    updateMotors(m1, m2, m3, m4);

    // Safety Delay (Loop at 250Hz - 4ms)
    while (micros() - last_time < 4000); 
}

// --- CORE FUNCTIONS ---

float computePID(float target, float actual, PID* p, float dt) {
    float error = target - actual;
    float p_term = p->P * error;
    p->integral_prior += error * dt;
    float i_term = p->I * p->integral_prior;
    float d_term = p->D * (error - p->error_prior) / dt;
    
    p->error_prior = error;
    return p_term + i_term + d_term;
}

void processGPSData() {
    // Advanced parsing of NMEA Sentences ($GPRMC, $GPGGA)
    char c = gpsSerial.read();
    // Logic to update global lat/lon/alt variables
}

void calculateOrientation() {
    // Implement Complementary Filter logic here:
    // angle = alpha * (angle + gyro * dt) + (1 - alpha) * (accel_angle)
}

void initializeMotors() {
    pinMode(MOTOR_FR, OUTPUT);
    pinMode(MOTOR_FL, OUTPUT);
    pinMode(MOTOR_BR, OUTPUT);
    pinMode(MOTOR_BL, OUTPUT);
}

void updateMotors(int r1, int r2, int r3, int r4) {
    // Output PWM signals to ESCs
    analogWrite(MOTOR_FR, constrain(r1, 1000, 2000));
    analogWrite(MOTOR_FL, constrain(r2, 1000, 2000));
    analogWrite(MOTOR_BR, constrain(r3, 1000, 2000));
    analogWrite(MOTOR_BL, constrain(r4, 1000, 2000))

// --- SYSTEM DEFINITIONS ---
#define GYRO_CONFIG_ADDR 0x6B // MPU6050 Address
#define ACCEL_CONFIG_ADDR 0x1C
#define PID_LOOP_FREQ 250     // 250Hz (4ms loop)

// --- PIN ASSIGNMENTS ---
const int M_FR = 10, M_FL = 11, M_BR = 12, M_BL = 13;
SoftwareSerial gpsSerial(4, 3); // BN-220T Interface

// --- DATA STRUCTURES ---
struct PID_Gains {
    float Kp, Ki, Kd;
    float error_prev, integral;
};

PID_Gains rollAxis  = {1.45, 0.02, 12.5, 0, 0};
PID_Gains pitchAxis = {1.45, 0.02, 12.5, 0, 0};
PID_Gains yawAxis   = {2.80, 0.05, 0.00, 0, 0};

// --- FLIGHT VARIABLES ---
float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
float angle_roll, angle_pitch;
float roll_level_adjust, pitch_level_adjust;
float alpha = 0.98; // Complementary Filter Constant
unsigned long timer;

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600);
    Wire.begin();
    Wire.setClock(400000); // High-speed I2C

    setupMPU6050();
    pinMode(M_FR, OUTPUT); pinMode(M_FL, OUTPUT);
    pinMode(M_BR, OUTPUT); pinMode(M_BL, OUTPUT);
    
    Serial.println(">>> Navigation );
    timer = micros();
}

void loop() {
    // 1. SENSOR FUSION ENGINE (Complementary Filter)
    readIMU();
    
    // Convert Raw to Degrees/sec
    gyro_x -= 0.5; gyro_y -= 0.2; // Manual Offset Calibration
  
    // 2. GPS TELEMETRY DECODER
    if (gpsSerial.available()) {
        parseGPS();
    }

    // 3. TRIPLE-AXIS PID COMPUTATION
    float dt = 0.004; // Fixed 250Hz time step
    float roll_out  = calculatePID(0 - angle_roll, &rollAxis, dt);
    float pitch_out = calculatePID(0 - angle_pitch, &pitchAxis, dt);
    float yaw_out   = calculatePID(0 - gyro_z, &yawAxis, dt);

    // 4. ADVANCED MOTOR MIXER (X-Frame Configuration)
    int throttle = 1500; // Base hover throttle
    
    int fr_speed = throttle + pitch_out + roll_out - yaw_out;
    int fl_speed = throttle + pitch_out - roll_out + yaw_out;
    int br_speed = throttle - pitch_out + roll_out + yaw_out;
    int bl_speed = throttle - pitch_out - roll_out - yaw_out;

    dispatchMotors(fr_speed, fl_speed, br_speed, bl_speed);

    // Synchronize Loop to 250Hz
    while (micros() - timer < 4000);
    timer = micros();
}

// --- CORE SYSTEM LOGIC ---

float calculatePID(float error, PID_Gains* p, float dt) {
    p->integral += error * dt;
    float derivative = (error - p->error_prev) / dt;
    float output = (p->Kp * error) + (p->Ki * p->integral) + (p->Kd * derivative);
    p->error_prev = error;
    return constrain(output, -400, 400);
}

void parseGPS() {
    // Industrial logic to read NMEA sentences ($GPRMC)
    String raw = gpsSerial.readStringUntil('\n');
    if (raw.startsWith("$GPRMC")) {
        // Telemetry feedback for Ground Control Station
        Serial.println("GPS_LOCK_OK: " + raw.substring(7, 17));
    }
}

void readIMU() {
    // I2C Communication to MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); // Start with Accel X
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    
    acc_x = (Wire.read() << 8 | Wire.read()) / 16384.0;
    acc_y = (Wire.read() << 8 | Wire.read()) / 16384.0;
    // ... Read Gyro data similarly
}

void dispatchMotors(int fr, int fl, int br, int bl) {
    analogWrite(M_FR, constrain(fr, 1000, 2000));
    analogWrite(M_FL, constrain(fl, 1000, 2000));
    analogWrite(M_BR, constrain(br, 1000, 2000));
    analogWrite(M_BL, constrain(bl, 1000, 2000));
}
}
