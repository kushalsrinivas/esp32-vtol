/* MPU6050 Serial Plotter - Temporary Code
 *
 * This code reads MPU6050 data and plots it on the Serial Monitor/Plotter
 * with low-pass filtering for clean visualization.
 *
 * HARDWARE CONNECTIONS:
 * =====================
 * MPU6050 -> ESP32
 * VCC     -> 3.3V
 * GND     -> GND
 * SCL     -> D22 (GPIO 22)
 * SDA     -> D21 (GPIO 21)
 *
 * SERIAL PLOTTER USAGE:
 * =====================
 * 1. Upload this code to your ESP32
 * 2. Open Serial Plotter: Tools -> Serial Plotter
 * 3. Set baud rate to 115200
 * 4. Tilt the sensor to see the graphs update in real-time
 *
 * DATA OUTPUT:
 * ============
 * - AccelX, AccelY, AccelZ (in g)
 * - GyroX, GyroY, GyroZ (in deg/s)
 * - TiltX, TiltY (calculated angles in degrees)
 *
 * All data is filtered using the same LP filter from the main project.
 */

#include <Wire.h>

// ==================== PIN DEFINITIONS ====================
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU6050_ADDR 0x68

// ==================== SENSOR SCALING ====================
#define ACCEL_SCALE_FACTOR 16384.0 // LSB/g for ±2g range
#define GYRO_SCALE_FACTOR 131.0    // LSB/(°/s) for ±250°/s range

// ==================== FILTER COEFFICIENTS ====================
// These are the same LP filter coefficients used in the main project
#define B_ACCEL 0.14 // Accelerometer filter (higher = more responsive, more noise)
#define B_GYRO 0.10  // Gyroscope filter (higher = more responsive, more noise)

// ==================== SENSOR VARIABLES ====================
float AccelX = 0, AccelY = 0, AccelZ = 0;
float GyroX = 0, GyroY = 0, GyroZ = 0;
float AccelX_prev = 0, AccelY_prev = 0, AccelZ_prev = 0;
float GyroX_prev = 0, GyroY_prev = 0, GyroZ_prev = 0;
float tilt_X = 0, tilt_Y = 0;

// ==================== SETUP ====================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔═══════════════════════════════════════╗");
    Serial.println("║  MPU6050 Serial Plotter - LP Filter  ║");
    Serial.println("╚═══════════════════════════════════════╝\n");

    // Initialize I2C
    Serial.println("→ Initializing I2C and MPU6050...");
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); // 100kHz I2C clock

    // Check if MPU6050 is connected
    Wire.beginTransmission(MPU6050_ADDR);
    byte error = Wire.endTransmission();

    if (error != 0)
    {
        Serial.println("✗ ERROR: MPU6050 not found!");
        Serial.println("  Check wiring:");
        Serial.println("    VCC -> 3.3V");
        Serial.println("    GND -> GND");
        Serial.println("    SCL -> D22");
        Serial.println("    SDA -> D21");
        while (1)
        {
            delay(1000);
        }
    }

    // Wake up MPU6050 (it starts in sleep mode)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // Set to 0 to wake up
    Wire.endTransmission();
    delay(100);

    Serial.println("✓ MPU6050 initialized successfully!\n");

    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║  Streaming Data to Serial Plotter    ║");
    Serial.println("╚═══════════════════════════════════════╝\n");

    Serial.println("Filter Settings:");
    Serial.printf("  Accelerometer: B = %.2f\n", B_ACCEL);
    Serial.printf("  Gyroscope:     B = %.2f\n\n", B_GYRO);

    Serial.println("Switch to Serial Plotter for graphs!");
    Serial.println("----------------------------------------\n");

    delay(2000);
}

// ==================== MAIN LOOP ====================
void loop()
{
    // Read MPU6050 sensor registers
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Starting register (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14); // Request 14 bytes

    if (Wire.available() >= 14)
    {
        // Read raw 16-bit values
        int16_t accelX_raw = Wire.read() << 8 | Wire.read();
        int16_t accelY_raw = Wire.read() << 8 | Wire.read();
        int16_t accelZ_raw = Wire.read() << 8 | Wire.read();
        int16_t temp = Wire.read() << 8 | Wire.read(); // Temperature (not used)
        int16_t gyroX_raw = Wire.read() << 8 | Wire.read();
        int16_t gyroY_raw = Wire.read() << 8 | Wire.read();
        int16_t gyroZ_raw = Wire.read() << 8 | Wire.read();

        // ==================== ACCELEROMETER PROCESSING ====================
        // Scale raw values to g (gravity units)
        AccelX = accelX_raw / ACCEL_SCALE_FACTOR;
        AccelY = accelY_raw / ACCEL_SCALE_FACTOR;
        AccelZ = accelZ_raw / ACCEL_SCALE_FACTOR;

        // Apply low-pass filter: y[n] = (1-B)*y[n-1] + B*x[n]
        // This smooths out high-frequency noise while preserving slow movements
        AccelX = (1.0 - B_ACCEL) * AccelX_prev + B_ACCEL * AccelX;
        AccelY = (1.0 - B_ACCEL) * AccelY_prev + B_ACCEL * AccelY;
        AccelZ = (1.0 - B_ACCEL) * AccelZ_prev + B_ACCEL * AccelZ;

        // Save for next iteration
        AccelX_prev = AccelX;
        AccelY_prev = AccelY;
        AccelZ_prev = AccelZ;

        // ==================== GYROSCOPE PROCESSING ====================
        // Scale raw values to degrees per second
        GyroX = gyroX_raw / GYRO_SCALE_FACTOR;
        GyroY = gyroY_raw / GYRO_SCALE_FACTOR;
        GyroZ = gyroZ_raw / GYRO_SCALE_FACTOR;

        // Apply low-pass filter
        GyroX = (1.0 - B_GYRO) * GyroX_prev + B_GYRO * GyroX;
        GyroY = (1.0 - B_GYRO) * GyroY_prev + B_GYRO * GyroY;
        GyroZ = (1.0 - B_GYRO) * GyroZ_prev + B_GYRO * GyroZ;

        // Save for next iteration
        GyroX_prev = GyroX;
        GyroY_prev = GyroY;
        GyroZ_prev = GyroZ;

        // ==================== TILT ANGLE CALCULATION ====================
        // Calculate tilt angles from accelerometer data
        // These represent the physical orientation of the sensor
        tilt_X = atan2(AccelY, sqrt(AccelX * AccelX + AccelZ * AccelZ)) * 57.2957795; // Convert to degrees
        tilt_Y = atan2(-AccelX, sqrt(AccelY * AccelY + AccelZ * AccelZ)) * 57.2957795;

        // ==================== SERIAL OUTPUT ====================
        // Format for Serial Plotter (each variable will appear as a separate line)
        // You can comment out variables you don't want to see

        Serial.print("AccelX:");
        Serial.print(AccelX, 3);
        Serial.print(",");

        Serial.print("AccelY:");
        Serial.print(AccelY, 3);
        Serial.print(",");

        Serial.print("AccelZ:");
        Serial.print(AccelZ, 3);
        Serial.print(",");

        Serial.print("GyroX:");
        Serial.print(GyroX, 2);
        Serial.print(",");

        Serial.print("GyroY:");
        Serial.print(GyroY, 2);
        Serial.print(",");

        Serial.print("GyroZ:");
        Serial.print(GyroZ, 2);
        Serial.print(",");

        Serial.print("TiltX:");
        Serial.print(tilt_X, 1);
        Serial.print(",");

        Serial.print("TiltY:");
        Serial.println(tilt_Y, 1); // Use println for the last value

        // ==================== OPTIONAL: Text Output ====================
        // Uncomment below for detailed text output (comment out the Serial Plotter format above)
        /*
        Serial.println("─────────────────────────────────────");
        Serial.printf("Accel: X=%.3fg  Y=%.3fg  Z=%.3fg\n", AccelX, AccelY, AccelZ);
        Serial.printf("Gyro:  X=%.2f°/s  Y=%.2f°/s  Z=%.2f°/s\n", GyroX, GyroY, GyroZ);
        Serial.printf("Tilt:  X=%.1f°  Y=%.1f°\n", tilt_X, tilt_Y);
        Serial.println();
        */
    }

    // Update rate: 20Hz (50ms delay)
    // You can adjust this for faster/slower updates
    delay(50);
}

/* ==================== USAGE NOTES ====================
 *
 * SERIAL PLOTTER:
 * ---------------
 * - Best for visualizing real-time sensor data as graphs
 * - Open via: Tools -> Serial Plotter
 * - Each labeled variable appears as a colored line
 * - Tilt the sensor to see the graphs change
 *
 * SERIAL MONITOR:
 * ---------------
 * - Shows initialization messages and debug info
 * - Uncomment the "Text Output" section for detailed readings
 *
 * FILTER TUNING:
 * --------------
 * - B_ACCEL and B_GYRO control filter responsiveness
 * - Lower values (0.05-0.10) = smoother, slower response
 * - Higher values (0.15-0.30) = faster, more noise
 * - Current values (0.14, 0.10) are optimized for this project
 *
 * TROUBLESHOOTING:
 * ----------------
 * - If values are all zeros: Check I2C connections
 * - If values are noisy: Decrease B_ACCEL/B_GYRO
 * - If response is sluggish: Increase B_ACCEL/B_GYRO
 * - If sensor not found: Check 3.3V power and GND
 *
 * EXPECTED VALUES:
 * ----------------
 * - AccelZ should read ~1.0g when sensor is flat (gravity)
 * - AccelX, AccelY should be ~0.0g when flat
 * - Gyro values should be near 0 when stationary
 * - Tilt angles range from -90° to +90°
 */
