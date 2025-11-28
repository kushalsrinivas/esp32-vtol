/* ESP32 TRANSMITTER - MPU6050 + ELEVON MIXING + ESP-NOW Telemetry
   TAILSITTER AIRCRAFT CONFIGURATION

   This transmitter reads MPU6050 sensor data, controls elevons with PID,
   and sends telemetry data via ESP-NOW to a receiver ESP32 for display.

   HARDWARE SETUP:
   ===============
   MPU6050 Connections:
     SDA -> GPIO 21
     SCL -> GPIO 22
     VCC -> 3.3V
     GND -> GND

   ELEVON Servo Connections (TAILSITTER - mounted underneath):
     GPIO 25 -> RIGHT Elevon Signal (Elevon mixing: Pitch + Roll)
     GPIO 26 -> LEFT Elevon Signal  (Elevon mixing: Pitch - Roll)
     GND     -> Servo GND (Brown/Black)
     5V EXT  -> Servo Red wires (MUST use external 5V supply)

   ELEVON MIXING:
   ==============
   - PITCH: Both elevons move together (Up/Down for pitch control)
   - ROLL:  Elevons move opposite (Differential for roll control)
   - Range: 30° to 150° (±60° from center) to prevent servo binding

   TELEMETRY DATA SENT:
   ====================
   - Pitch and Roll angles
   - Elevon positions (desired and actual)
   - Gyroscope rates (Pitch and Roll)
   - Raw accelerometer data (X, Y, Z)
   - Timestamp for connection monitoring

   IMPORTANT:
   - Replace RECEIVER_MAC_ADDRESS with your receiver ESP32's MAC address
   - To find MAC address, upload receiver code first and check serial monitor
   - This code now sends accelerometer data for the receiver's menu system
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

// ==================== PIN DEFINITIONS ====================
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU6050_ADDR 0x68
#define RIGHT_ELEVON_PIN 25 // Right elevon (underneath aircraft)
#define LEFT_ELEVON_PIN 26  // Left elevon (underneath aircraft)

// ==================== SENSOR SCALING ====================
#define ACCEL_SCALE_FACTOR 16384.0 // LSB/g for ±2g range
#define GYRO_SCALE_FACTOR 131.0    // LSB/(°/s) for ±250°/s range

// ==================== FILTER COEFFICIENTS ====================
#define B_ACCEL 0.14 // Accelerometer filter
#define B_GYRO 0.10  // Gyroscope filter

// ==================== ELEVON CONFIGURATION ====================
#define ELEVON_CENTER 90  // Center position (neutral)
#define ELEVON_MIN 30     // Minimum angle (90 - 60°)
#define ELEVON_MAX 150    // Maximum angle (90 + 60°)
#define ELEVON_RANGE 60.0 // ±60° from center

// ==================== PID PARAMETERS ====================
// Pitch control (both elevons together) - SMOOTH STABILIZATION
float Kp_pitch = 35.0; // Moderate response for smooth control
float Ki_pitch = 3.0;  // Small integral to eliminate steady-state error
float Kd_pitch = 8.0;  // Moderate damping to prevent oscillation

// Roll control (elevons differential) - SMOOTH STABILIZATION
float Kp_roll = 35.0; // Moderate response for smooth control
float Ki_roll = 3.0;  // Small integral to eliminate steady-state error
float Kd_roll = 8.0;  // Moderate damping to prevent oscillation

float i_limit = 15.0; // Integrator limit to prevent windup
float max_tilt_angle = 45.0;

// ==================== TELEMETRY DATA STRUCTURE ====================
// This structure will be sent via ESP-NOW
typedef struct
{
    float pitch;               // Pitch angle (was tiltX)
    float roll;                // Roll angle (was tiltY)
    float right_elevon_des;    // Right elevon desired position
    float right_elevon_actual; // Right elevon actual position
    float left_elevon_des;     // Left elevon desired position
    float left_elevon_actual;  // Left elevon actual position
    float gyro_pitch;          // Pitch rate (was gyroX)
    float gyro_roll;           // Roll rate (was gyroY)
    float accelX;              // Raw accelerometer data
    float accelY;
    float accelZ;
    uint32_t timestamp; // For checking connection
} TelemetryData;

TelemetryData telemetry;

// ==================== ESP-NOW CONFIGURATION ====================
// Receiver ESP32's MAC Address: FC:E8:C0:E0:D2:F4
// Format: {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX}
uint8_t receiverAddress[] = {0xFC, 0xE8, 0xC0, 0xE0, 0xD2, 0xF4};

esp_now_peer_info_t peerInfo;

// ==================== SENSOR VARIABLES ====================
float AccelX = 0, AccelY = 0, AccelZ = 0;
float GyroX = 0, GyroY = 0, GyroZ = 0;
float AccelX_prev = 0, AccelY_prev = 0, AccelZ_prev = 0;
float GyroX_prev = 0, GyroY_prev = 0, GyroZ_prev = 0;
float pitch = 0, roll = 0; // Aircraft pitch and roll angles

// ==================== CALIBRATION OFFSETS ====================
float pitch_offset = 0.0; // Level pitch offset (calibrated at startup)
float roll_offset = 0.0;  // Level roll offset (calibrated at startup)
float GyroX_offset = 0.0; // Gyro drift offset
float GyroY_offset = 0.0;
float GyroZ_offset = 0.0;

// ==================== PID VARIABLES ====================
// Pitch PID
float pitch_des = 0.0; // Desired pitch angle
float error_pitch = 0, error_pitch_prev = 0;
float integral_pitch = 0, integral_pitch_prev = 0;
float derivative_pitch = 0, pitch_PID = 0;

// Roll PID
float roll_des = 0.0; // Desired roll angle
float error_roll = 0, error_roll_prev = 0;
float integral_roll = 0, integral_roll_prev = 0;
float derivative_roll = 0, roll_PID = 0;

// Elevon outputs (after mixing)
float right_elevon_angle = ELEVON_CENTER;
float left_elevon_angle = ELEVON_CENTER;

// ==================== TIMING ====================
unsigned long current_time = 0, prev_time = 0;
float dt = 0;
unsigned long lastTelemetrySend = 0;
const unsigned long TELEMETRY_INTERVAL = 20; // Send every 20ms (50Hz)
const unsigned long LOOP_INTERVAL = 10;      // Main loop at 100Hz (10ms)

// ==================== SERVO OBJECTS ====================
Servo right_elevon;
Servo left_elevon;

// ==================== ESP-NOW CALLBACKS ====================
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    // Optional: Monitor send status
    // Serial.print("Send Status: ");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ==================== MPU6050 CALIBRATION ====================
void calibrateMPU6050()
{
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║     MPU6050 CALIBRATION STARTING      ║");
    Serial.println("╚═══════════════════════════════════════╝\n");

    Serial.println("⚠  IMPORTANT: Place aircraft on FLAT, LEVEL surface!");
    Serial.println("   Do NOT move the aircraft during calibration.\n");

    Serial.println("→ Calibrating in 3 seconds...");
    delay(1000);
    Serial.println("→ 2 seconds...");
    delay(1000);
    Serial.println("→ 1 second...");
    delay(1000);
    Serial.println("\n→ Calibrating now...\n");

    const int numSamples = 200; // Take 200 samples (2 seconds at 100Hz)
    float accelX_sum = 0, accelY_sum = 0, accelZ_sum = 0;
    float gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;

    // Collect samples
    for (int i = 0; i < numSamples; i++)
    {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);

        if (Wire.available() >= 14)
        {
            int16_t accelX_raw = Wire.read() << 8 | Wire.read();
            int16_t accelY_raw = Wire.read() << 8 | Wire.read();
            int16_t accelZ_raw = Wire.read() << 8 | Wire.read();
            int16_t temp = Wire.read() << 8 | Wire.read();
            int16_t gyroX_raw = Wire.read() << 8 | Wire.read();
            int16_t gyroY_raw = Wire.read() << 8 | Wire.read();
            int16_t gyroZ_raw = Wire.read() << 8 | Wire.read();

            // Scale to real units
            float accelX = accelX_raw / ACCEL_SCALE_FACTOR;
            float accelY = accelY_raw / ACCEL_SCALE_FACTOR;
            float accelZ = accelZ_raw / ACCEL_SCALE_FACTOR;

            // Calculate pitch and roll for this sample
            float pitch_sample = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.2957795;
            float roll_sample = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2957795;

            accelX_sum += pitch_sample;
            accelY_sum += roll_sample;

            gyroX_sum += gyroX_raw / GYRO_SCALE_FACTOR;
            gyroY_sum += gyroY_raw / GYRO_SCALE_FACTOR;
            gyroZ_sum += gyroZ_raw / GYRO_SCALE_FACTOR;
        }

        // Show progress
        if ((i + 1) % 20 == 0)
        {
            Serial.printf("  Progress: %d%%\r", ((i + 1) * 100) / numSamples);
        }

        delay(10); // 100Hz sampling
    }

    // Calculate averages (these are the offsets when level)
    pitch_offset = accelX_sum / numSamples;
    roll_offset = accelY_sum / numSamples;
    GyroX_offset = gyroX_sum / numSamples;
    GyroY_offset = gyroY_sum / numSamples;
    GyroZ_offset = gyroZ_sum / numSamples;

    Serial.println("\n");
    Serial.println("✓ Calibration complete!\n");
    Serial.println("Calibration Offsets:");
    Serial.printf("  Pitch offset:  %+.2f°\n", pitch_offset);
    Serial.printf("  Roll offset:   %+.2f°\n", roll_offset);
    Serial.printf("  Gyro X offset: %+.2f°/s\n", GyroX_offset);
    Serial.printf("  Gyro Y offset: %+.2f°/s\n", GyroY_offset);
    Serial.printf("  Gyro Z offset: %+.2f°/s\n\n", GyroZ_offset);

    Serial.println("→ These offsets will be subtracted from all readings");
    Serial.println("→ Aircraft is now calibrated to level surface\n");
}

// ==================== SETUP ====================
void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  ESP32 TRANSMITTER - TAILSITTER        ║");
    Serial.println("║  MPU6050 + ELEVON MIXING + ESP-NOW     ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    // ========== INITIALIZE MPU6050 ==========
    Serial.println("→ Initializing I2C and MPU6050...");
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    // Scan for MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    byte error = Wire.endTransmission();

    if (error != 0)
    {
        Serial.println("✗ ERROR: MPU6050 not found!");
        Serial.println("  Check wiring and restart.");
        while (1)
            delay(1000);
    }

    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0);    // Wake up
    Wire.endTransmission();
    delay(100);

    Serial.println("✓ MPU6050 initialized successfully\n");

    // ========== CALIBRATE MPU6050 ==========
    calibrateMPU6050();

    // ========== INITIALIZE ELEVONS ==========
    Serial.println("→ Initializing elevons...");
    right_elevon.attach(RIGHT_ELEVON_PIN);
    left_elevon.attach(LEFT_ELEVON_PIN);
    right_elevon.write(ELEVON_CENTER);
    left_elevon.write(ELEVON_CENTER);
    Serial.printf("✓ Elevons initialized and centered at %d°\n", ELEVON_CENTER);
    Serial.printf("  Range: %d° to %d° (±%d° from center)\n\n", ELEVON_MIN, ELEVON_MAX, (int)ELEVON_RANGE);

    // ========== INITIALIZE ESP-NOW ==========
    Serial.println("→ Initializing ESP-NOW...");

    // Initialize WiFi in Station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); // Ensure clean state
    delay(100);        // Give WiFi time to initialize

    // Print this ESP32's MAC address
    Serial.print("  Transmitter MAC Address: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("✗ ERROR: ESP-NOW initialization failed!");
        while (1)
            delay(1000);
    }

    // Register send callback
    esp_now_register_send_cb(OnDataSent);

    // Register receiver peer
    memcpy(peerInfo.peer_addr, receiverAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("✗ ERROR: Failed to add peer!");
        Serial.println("  Make sure you've set the correct receiver MAC address!");
        while (1)
            delay(1000);
    }

    Serial.println("✓ ESP-NOW initialized successfully");
    Serial.print("  Sending to receiver: ");
    for (int i = 0; i < 6; i++)
    {
        Serial.printf("%02X", receiverAddress[i]);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println("\n");

    // ========== PID SETTINGS ==========
    Serial.println("→ PID Controller Settings:");
    Serial.printf("  Pitch: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", Kp_pitch, Ki_pitch, Kd_pitch);
    Serial.printf("  Roll:  Kp=%.1f, Ki=%.1f, Kd=%.1f\n", Kp_roll, Ki_roll, Kd_roll);
    Serial.printf("  Integrator limit: %.1f\n", i_limit);
    Serial.println("  Elevon Mixing: ENABLED (Pitch + Roll)");
    Serial.printf("  Control Loop: 100Hz | Telemetry: 50Hz\n\n");

    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  SYSTEM READY - Starting Main Loop    ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    prev_time = millis();
}

// ==================== MAIN LOOP ====================
void loop()
{
    // Calculate dt
    current_time = millis();
    dt = (current_time - prev_time) / 1000.0;
    prev_time = current_time;

    // Read MPU6050 data at 100Hz (much smoother response)
    delay(10); // 100Hz update rate - NO MORE GLITCHING!

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);

    if (Wire.available() >= 14)
    {
        // Read raw values
        int16_t accelX_raw = Wire.read() << 8 | Wire.read();
        int16_t accelY_raw = Wire.read() << 8 | Wire.read();
        int16_t accelZ_raw = Wire.read() << 8 | Wire.read();
        int16_t temp = Wire.read() << 8 | Wire.read();
        int16_t gyroX_raw = Wire.read() << 8 | Wire.read();
        int16_t gyroY_raw = Wire.read() << 8 | Wire.read();
        int16_t gyroZ_raw = Wire.read() << 8 | Wire.read();

        // Scale to real units
        AccelX = accelX_raw / ACCEL_SCALE_FACTOR;
        AccelY = accelY_raw / ACCEL_SCALE_FACTOR;
        AccelZ = accelZ_raw / ACCEL_SCALE_FACTOR;

        // Apply low-pass filter to accelerometer
        AccelX = (1.0 - B_ACCEL) * AccelX_prev + B_ACCEL * AccelX;
        AccelY = (1.0 - B_ACCEL) * AccelY_prev + B_ACCEL * AccelY;
        AccelZ = (1.0 - B_ACCEL) * AccelZ_prev + B_ACCEL * AccelZ;
        AccelX_prev = AccelX;
        AccelY_prev = AccelY;
        AccelZ_prev = AccelZ;

        // Scale and filter gyroscope
        GyroX = gyroX_raw / GYRO_SCALE_FACTOR;
        GyroY = gyroY_raw / GYRO_SCALE_FACTOR;
        GyroZ = gyroZ_raw / GYRO_SCALE_FACTOR;

        // Apply gyro calibration offsets
        GyroX -= GyroX_offset;
        GyroY -= GyroY_offset;
        GyroZ -= GyroZ_offset;

        GyroX = (1.0 - B_GYRO) * GyroX_prev + B_GYRO * GyroX;
        GyroY = (1.0 - B_GYRO) * GyroY_prev + B_GYRO * GyroY;
        GyroZ = (1.0 - B_GYRO) * GyroZ_prev + B_GYRO * GyroZ;
        GyroX_prev = GyroX;
        GyroY_prev = GyroY;
        GyroZ_prev = GyroZ;

        // Calculate pitch and roll angles
        pitch = atan2(AccelY, sqrt(AccelX * AccelX + AccelZ * AccelZ)) * 57.2957795;
        roll = atan2(-AccelX, sqrt(AccelY * AccelY + AccelZ * AccelZ)) * 57.2957795;

        // Apply calibration offsets (subtract the level offset)
        pitch -= pitch_offset;
        roll -= roll_offset;

        pitch = constrain(pitch, -max_tilt_angle, max_tilt_angle);
        roll = constrain(roll, -max_tilt_angle, max_tilt_angle);

        // Compute PID for pitch and roll
        computePID_Pitch();
        computePID_Roll();

        // ========== ELEVON MIXING ==========
        // PITCH REVERSED: Servos mounted sideways, need negative pitch
        // Right Elevon = -Pitch + Roll (negative pitch for nose up, positive roll = right down)
        // Left Elevon  = -Pitch - Roll (negative pitch for nose up, positive roll = left up)

        // DIAGNOSTIC: Temporarily test PITCH ONLY (no roll mixing)
        right_elevon_angle = ELEVON_CENTER - pitch_PID; // + roll_PID;
        left_elevon_angle = ELEVON_CENTER - pitch_PID;  // - roll_PID;

        // Constrain to safe range (30° to 150°, ±60° from center)
        right_elevon_angle = constrain(right_elevon_angle, ELEVON_MIN, ELEVON_MAX);
        left_elevon_angle = constrain(left_elevon_angle, ELEVON_MIN, ELEVON_MAX);

        // Write to servos
        right_elevon.write((int)right_elevon_angle);
        left_elevon.write((int)left_elevon_angle);

        // ========== SEND TELEMETRY VIA ESP-NOW ==========
        if (millis() - lastTelemetrySend >= TELEMETRY_INTERVAL)
        {
            telemetry.pitch = pitch;
            telemetry.roll = roll;
            telemetry.right_elevon_des = pitch_des + roll_des;
            telemetry.right_elevon_actual = right_elevon_angle;
            telemetry.left_elevon_des = pitch_des - roll_des;
            telemetry.left_elevon_actual = left_elevon_angle;
            telemetry.gyro_pitch = GyroY;
            telemetry.gyro_roll = GyroX;
            telemetry.accelX = AccelX;
            telemetry.accelY = AccelY;
            telemetry.accelZ = AccelZ;
            telemetry.timestamp = millis();

            esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&telemetry, sizeof(telemetry));

            // Print status to serial monitor
            if (result == ESP_OK)
            {
                Serial.printf("TX→ Pitch:%.1f° Roll:%.1f° R-Elevon:%d° L-Elevon:%d° GyroPitch:%.1f GyroRoll:%.1f\n",
                              pitch, roll, (int)right_elevon_angle, (int)left_elevon_angle, GyroY, GyroX);
            }
            else
            {
                Serial.println("✗ Telemetry send failed!");
            }

            lastTelemetrySend = millis();
        }
    }
}

// ==================== PID FUNCTIONS ====================
void computePID_Pitch()
{
    // Error: desired pitch - actual pitch
    error_pitch = pitch_des - pitch;

    // Integral with anti-windup
    integral_pitch = integral_pitch_prev + error_pitch * dt;
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit);

    // Derivative (use gyro rate for better response)
    derivative_pitch = GyroY;

    // PID output (in degrees, will be added to center position)
    pitch_PID = Kp_pitch * error_pitch +
                Ki_pitch * integral_pitch -
                Kd_pitch * derivative_pitch;

    // Limit PID output to ±ELEVON_RANGE to prevent exceeding physical limits
    pitch_PID = constrain(pitch_PID, -ELEVON_RANGE, ELEVON_RANGE);

    error_pitch_prev = error_pitch;
    integral_pitch_prev = integral_pitch;
}

void computePID_Roll()
{
    // Error: desired roll - actual roll
    error_roll = roll_des - roll;

    // Integral with anti-windup
    integral_roll = integral_roll_prev + error_roll * dt;
    integral_roll = constrain(integral_roll, -i_limit, i_limit);

    // Derivative (use gyro rate for better response)
    derivative_roll = GyroX;

    // PID output (in degrees, will be added/subtracted for elevon mixing)
    roll_PID = Kp_roll * error_roll +
               Ki_roll * integral_roll -
               Kd_roll * derivative_roll;

    // Limit PID output to ±ELEVON_RANGE to prevent exceeding physical limits
    roll_PID = constrain(roll_PID, -ELEVON_RANGE, ELEVON_RANGE);

    error_roll_prev = error_roll;
    integral_roll_prev = integral_roll;
}
