/* MPU6050 Diagnostic Tool with Serial Plotter Output and Servo Control
   This will help identify if the MPU6050 is responding on I2C,
   display real-time data in the Serial Plotter, and control servos based on tilt

   MPU6050 Wiring:
     SDA -> GPIO 21
     SCL -> GPIO 22
     VCC -> 3.3V
     GND -> GND

   SERVOS:
     GPIO 25 -> Servo 1 Signal (controlled by X-axis)
     GPIO 26 -> Servo 2 Signal (controlled by Y-axis)
     GND     -> Servo GND (Brown/Black)
     5V EXT  -> Servo Red wires (MUST use external 5V supply)

   Usage:
   - Open Serial Monitor first to see diagnostic info
   - After setup completes, switch to Serial Plotter to see live graphs
   - Tilt the MPU6050 to control the servos
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU6050_ADDR 0x68     // Default I2C address
#define MPU6050_ADDR_ALT 0x69 // Alternative address (if AD0 pin is HIGH)

// Servo pins
#define SERVO1_PIN 25 // Controlled by X-axis
#define SERVO2_PIN 26 // Controlled by Y-axis

// MPU6050 scale factors (for ±2g and ±250°/s settings)
#define ACCEL_SCALE_FACTOR 16384.0 // LSB/g for ±2g range
#define GYRO_SCALE_FACTOR 131.0    // LSB/(°/s) for ±250°/s range

// Low-pass filter coefficients (for ~20Hz loop rate)
// These filter out noise while keeping responsiveness
#define B_ACCEL 0.14 // Accelerometer filter coefficient
#define B_GYRO 0.10  // Gyroscope filter coefficient (more smoothing for gyro)

// PID Controller Parameters (inspired by dRehmFlight)
// Tuning guide: Start with P only, then add I for steady-state error, then D for damping
float Kp_servo1 = 45.0; // Proportional gain for servo1 (X-axis)
float Ki_servo1 = 5.0;  // Integral gain for servo1
float Kd_servo1 = 8.0;  // Derivative gain for servo1

float Kp_servo2 = 45.0; // Proportional gain for servo2 (Y-axis)
float Ki_servo2 = 5.0;  // Integral gain for servo2
float Kd_servo2 = 8.0;  // Derivative gain for servo2

float i_limit = 25.0;        // Integrator saturation limit (prevents windup)
float max_tilt_angle = 45.0; // Maximum tilt angle in degrees for mapping

// Filtered sensor values (in real units)
float AccelX = 0, AccelY = 0, AccelZ = 0;
float GyroX = 0, GyroY = 0, GyroZ = 0;

// Previous filtered values (for LP filter)
float AccelX_prev = 0, AccelY_prev = 0, AccelZ_prev = 0;
float GyroX_prev = 0, GyroY_prev = 0, GyroZ_prev = 0;

// Computed tilt angles from accelerometer
float tilt_X = 0; // Roll angle (degrees)
float tilt_Y = 0; // Pitch angle (degrees)

// Desired servo angles (setpoints)
float servo1_des = 90.0; // Desired angle for servo1 (center)
float servo2_des = 90.0; // Desired angle for servo2 (center)

// PID variables for Servo 1 (X-axis control)
float error_servo1 = 0;
float error_servo1_prev = 0;
float integral_servo1 = 0;
float integral_servo1_prev = 0;
float derivative_servo1 = 0;
float servo1_PID = 0;

// PID variables for Servo 2 (Y-axis control)
float error_servo2 = 0;
float error_servo2_prev = 0;
float integral_servo2 = 0;
float integral_servo2_prev = 0;
float derivative_servo2 = 0;
float servo2_PID = 0;

// Timing variables for accurate dt calculation
unsigned long current_time = 0;
unsigned long prev_time = 0;
float dt = 0;

// Servo objects
Servo servo1; // X-axis control
Servo servo2; // Y-axis control

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n\n=================================");
    Serial.println("MPU6050 Diagnostic Tool");
    Serial.println("=================================\n");

    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); // 100kHz - slower speed for reliability

    Serial.println("Step 1: Scanning I2C bus...");
    Serial.println("---------------------------------");

    bool deviceFound = false;
    byte foundAddr = 0;

    for (byte addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("Device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.print(" (");
            Serial.print(addr);
            Serial.println(")");

            if (addr == MPU6050_ADDR || addr == MPU6050_ADDR_ALT)
            {
                deviceFound = true;
                foundAddr = addr;
            }
        }
    }

    if (!deviceFound)
    {
        Serial.println("\n>>> ERROR: No MPU6050 found on I2C bus!");
        Serial.println("\nPossible issues:");
        Serial.println("1. Wiring problem - check connections");
        Serial.println("2. MPU6050 is damaged");
        Serial.println("3. Wrong I2C pins configured");
        Serial.println("4. Power supply issue (VCC should be 3.3V)");
        Serial.println("\nWiring should be:");
        Serial.println("  MPU SDA -> GPIO 21");
        Serial.println("  MPU SCL -> GPIO 22");
        Serial.println("  MPU VCC -> 3.3V");
        Serial.println("  MPU GND -> GND");
        return;
    }

    Serial.println("---------------------------------");
    Serial.print("\nStep 2: Reading WHO_AM_I register from 0x");
    Serial.println(foundAddr, HEX);
    Serial.println("---------------------------------");

    Wire.beginTransmission(foundAddr);
    Wire.write(0x75); // WHO_AM_I register
    byte error = Wire.endTransmission(false);

    if (error != 0)
    {
        Serial.println(">>> ERROR: Could not write to MPU6050!");
        Serial.print("I2C error code: ");
        Serial.println(error);
        return;
    }

    Wire.requestFrom(foundAddr, (uint8_t)1);
    if (Wire.available())
    {
        byte whoami = Wire.read();
        Serial.print("WHO_AM_I = 0x");
        Serial.print(whoami, HEX);

        if (whoami == 0x68)
        {
            Serial.println(" ✓ CORRECT (MPU6050 responding!)");
        }
        else if (whoami == 0x00 || whoami == 0xFF)
        {
            Serial.println(" ✗ INCORRECT (likely hardware failure)");
            Serial.println("\n>>> MPU6050 appears to be damaged or not powered");
        }
        else
        {
            Serial.println(" ? UNEXPECTED");
            Serial.println("This might be a different sensor or damaged MPU6050");
        }
    }
    else
    {
        Serial.println(">>> ERROR: No response from MPU6050!");
    }

    Serial.println("\n---------------------------------");
    Serial.println("Step 3: Wake up MPU6050");
    Serial.println("---------------------------------");

    // Wake up MPU6050 (PWR_MGMT_1 register)
    Wire.beginTransmission(foundAddr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // Wake up
    error = Wire.endTransmission();

    if (error == 0)
    {
        Serial.println("Wake up command sent ✓");
    }
    else
    {
        Serial.print(">>> ERROR sending wake up command. Error: ");
        Serial.println(error);
    }

    delay(100);

    Serial.println("\n---------------------------------");
    Serial.println("Step 4: Reading raw sensor data");
    Serial.println("---------------------------------");

    // Read raw accelerometer and gyro data
    Wire.beginTransmission(foundAddr);
    Wire.write(0x3B); // Starting register for accel readings
    Wire.endTransmission(false);
    Wire.requestFrom(foundAddr, (uint8_t)14);

    if (Wire.available() >= 14)
    {
        int16_t accelX = Wire.read() << 8 | Wire.read();
        int16_t accelY = Wire.read() << 8 | Wire.read();
        int16_t accelZ = Wire.read() << 8 | Wire.read();
        int16_t temp = Wire.read() << 8 | Wire.read();
        int16_t gyroX = Wire.read() << 8 | Wire.read();
        int16_t gyroY = Wire.read() << 8 | Wire.read();
        int16_t gyroZ = Wire.read() << 8 | Wire.read();

        Serial.println("Raw values:");
        Serial.print("  Accel X: ");
        Serial.println(accelX);
        Serial.print("  Accel Y: ");
        Serial.println(accelY);
        Serial.print("  Accel Z: ");
        Serial.println(accelZ);
        Serial.print("  Temp:    ");
        Serial.println(temp);
        Serial.print("  Gyro X:  ");
        Serial.println(gyroX);
        Serial.print("  Gyro Y:  ");
        Serial.println(gyroY);
        Serial.print("  Gyro Z:  ");
        Serial.println(gyroZ);

        // Check if all values are 0 or -1 (0xFFFF)
        if ((accelX == 0 && accelY == 0 && accelZ == 0 && gyroX == 0 && gyroY == 0 && gyroZ == 0) ||
            (accelX == -1 && accelY == -1 && accelZ == -1 && gyroX == -1 && gyroY == -1 && gyroZ == -1))
        {
            Serial.println("\n>>> WARNING: All values are same!");
            Serial.println("This usually means the sensor is damaged or in sleep mode");
        }
        else
        {
            Serial.println("\n✓ Sensor appears to be working!");
            Serial.println("Values are changing, which is good.");
        }
    }
    else
    {
        Serial.println(">>> ERROR: Could not read sensor data!");
    }

    Serial.println("\n=================================");
    Serial.println("Diagnostic complete!");
    Serial.println("=================================");

    // Initialize servos
    Serial.println("\nInitializing servos...");
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);

    // Center both servos
    servo1.write(90);
    servo2.write(90);
    Serial.println("Servos initialized and centered at 90°");

    Serial.println("\n=================================");
    Serial.println("PID CONTROLLER SETTINGS");
    Serial.println("=================================");
    Serial.println("Servo 1 (X-axis):");
    Serial.print("  Kp=");
    Serial.print(Kp_servo1);
    Serial.print(", Ki=");
    Serial.print(Ki_servo1);
    Serial.print(", Kd=");
    Serial.println(Kd_servo1);
    Serial.println("Servo 2 (Y-axis):");
    Serial.print("  Kp=");
    Serial.print(Kp_servo2);
    Serial.print(", Ki=");
    Serial.print(Ki_servo2);
    Serial.print(", Kd=");
    Serial.println(Kd_servo2);
    Serial.print("Integrator limit: ");
    Serial.println(i_limit);
    Serial.println("=================================\n");

    Serial.println("\nSwitching to Serial Plotter mode...");
    Serial.println("Open Tools -> Serial Plotter to see graphs");
    Serial.println("Tilt the MPU6050 to control servos with PID!");
    Serial.println("Starting in 3 seconds...\n");
    delay(3000);

    // Initialize timing
    prev_time = millis();
}

void loop()
{
    // Calculate dt (time since last loop) for PID controller
    current_time = millis();
    dt = (current_time - prev_time) / 1000.0; // Convert to seconds
    prev_time = current_time;

    // Read sensor data
    delay(50); // 20Hz update rate - good for PID stability

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);

    if (Wire.available() >= 14)
    {
        // Read raw 16-bit values
        int16_t accelX_raw = Wire.read() << 8 | Wire.read();
        int16_t accelY_raw = Wire.read() << 8 | Wire.read();
        int16_t accelZ_raw = Wire.read() << 8 | Wire.read();
        int16_t temp = Wire.read() << 8 | Wire.read();
        int16_t gyroX_raw = Wire.read() << 8 | Wire.read();
        int16_t gyroY_raw = Wire.read() << 8 | Wire.read();
        int16_t gyroZ_raw = Wire.read() << 8 | Wire.read();

        // Scale accelerometer to G's (1G = 9.8 m/s²)
        AccelX = accelX_raw / ACCEL_SCALE_FACTOR;
        AccelY = accelY_raw / ACCEL_SCALE_FACTOR;
        AccelZ = accelZ_raw / ACCEL_SCALE_FACTOR;

        // Apply low-pass filter to accelerometer
        // Formula: filtered = (1 - B) * previous + B * current
        AccelX = (1.0 - B_ACCEL) * AccelX_prev + B_ACCEL * AccelX;
        AccelY = (1.0 - B_ACCEL) * AccelY_prev + B_ACCEL * AccelY;
        AccelZ = (1.0 - B_ACCEL) * AccelZ_prev + B_ACCEL * AccelZ;

        // Store for next iteration
        AccelX_prev = AccelX;
        AccelY_prev = AccelY;
        AccelZ_prev = AccelZ;

        // Scale gyroscope to degrees/second
        GyroX = gyroX_raw / GYRO_SCALE_FACTOR;
        GyroY = gyroY_raw / GYRO_SCALE_FACTOR;
        GyroZ = gyroZ_raw / GYRO_SCALE_FACTOR;

        // Apply low-pass filter to gyroscope
        GyroX = (1.0 - B_GYRO) * GyroX_prev + B_GYRO * GyroX;
        GyroY = (1.0 - B_GYRO) * GyroY_prev + B_GYRO * GyroY;
        GyroZ = (1.0 - B_GYRO) * GyroZ_prev + B_GYRO * GyroZ;

        // Store for next iteration
        GyroX_prev = GyroX;
        GyroY_prev = GyroY;
        GyroZ_prev = GyroZ;

        // Calculate tilt angles from accelerometer (similar to dRehmFlight attitude estimation)
        // Roll (tilt around X-axis): atan2(AccelY, sqrt(AccelX^2 + AccelZ^2))
        // Pitch (tilt around Y-axis): atan2(-AccelX, sqrt(AccelY^2 + AccelZ^2))
        tilt_X = atan2(AccelY, sqrt(AccelX * AccelX + AccelZ * AccelZ)) * 57.2957795; // Convert rad to deg
        tilt_Y = atan2(-AccelX, sqrt(AccelY * AccelY + AccelZ * AccelZ)) * 57.2957795;

        // Constrain tilt angles to expected range
        tilt_X = constrain(tilt_X, -max_tilt_angle, max_tilt_angle);
        tilt_Y = constrain(tilt_Y, -max_tilt_angle, max_tilt_angle);

        // Map tilt angles to desired servo positions (setpoints)
        // Tilt angle -45° to +45° maps to servo 0° to 180°
        servo1_des = map(tilt_X * 100, -max_tilt_angle * 100, max_tilt_angle * 100, 0, 180);
        servo2_des = map(tilt_Y * 100, -max_tilt_angle * 100, max_tilt_angle * 100, 0, 180);

        // Constrain desired angles
        servo1_des = constrain(servo1_des, 0, 180);
        servo2_des = constrain(servo2_des, 0, 180);

        // Run PID controllers (based on dRehmFlight controlANGLE function)
        computePID_Servo1();
        computePID_Servo2();

        // Apply PID output to servos
        int servo1Angle = (int)servo1_PID;
        int servo2Angle = (int)servo2_PID;

        // Constrain to valid servo range (safety check)
        servo1Angle = constrain(servo1Angle, 0, 180);
        servo2Angle = constrain(servo2Angle, 0, 180);

        // Move servos
        servo1.write(servo1Angle);
        servo2.write(servo2Angle);

        // Serial Plotter format: label:value separated by commas
        Serial.print("TiltX:");
        Serial.print(tilt_X, 2);
        Serial.print(",TiltY:");
        Serial.print(tilt_Y, 2);
        Serial.print(",Servo1_Des:");
        Serial.print(servo1_des, 1);
        Serial.print(",Servo1_Actual:");
        Serial.print(servo1Angle);
        Serial.print(",Servo2_Des:");
        Serial.print(servo2_des, 1);
        Serial.print(",Servo2_Actual:");
        Serial.print(servo2Angle);
        Serial.print(",GyroX:");
        Serial.print(GyroX, 2);
        Serial.print(",GyroY:");
        Serial.println(GyroY, 2);
    }
}

//========================================================================================================================//
//                                                  PID CONTROLLER FUNCTIONS                                              //
//========================================================================================================================//

void computePID_Servo1()
{
    // DESCRIPTION: PID controller for Servo 1 (X-axis) based on dRehmFlight controlANGLE()
    /*
     * Computes PID control output based on desired servo angle (setpoint) and actual sensor feedback.
     * This implementation uses the tilt angle as feedback and gyro rate for derivative damping.
     *
     * P term: Proportional to error (desired - actual)
     * I term: Accumulated error over time (prevents steady-state error)
     * D term: Rate of change (provides damping, uses gyro for better performance)
     */

    // Calculate current servo position from tilt angle (feedback)
    float servo1_actual = map(tilt_X * 100, -max_tilt_angle * 100, max_tilt_angle * 100, 0, 180);
    servo1_actual = constrain(servo1_actual, 0, 180);

    // Calculate error: desired - actual
    error_servo1 = servo1_des - servo1_actual;

    // Integral term: accumulate error over time
    integral_servo1 = integral_servo1_prev + error_servo1 * dt;

    // Prevent integral windup (saturation) - inspired by dRehmFlight i_limit
    integral_servo1 = constrain(integral_servo1, -i_limit, i_limit);

    // Derivative term: use gyro rate for damping (more stable than differentiating error)
    // GyroY corresponds to rotation around Y-axis which affects X-tilt
    derivative_servo1 = GyroY;

    // Compute PID output
    // Note: Negative derivative because we want to oppose the rate of change
    servo1_PID = Kp_servo1 * error_servo1 +
                 Ki_servo1 * integral_servo1 -
                 Kd_servo1 * derivative_servo1;

    // Constrain output to valid servo range
    servo1_PID = constrain(servo1_PID, 0, 180);

    // Update previous values for next iteration
    error_servo1_prev = error_servo1;
    integral_servo1_prev = integral_servo1;
}

void computePID_Servo2()
{
    // DESCRIPTION: PID controller for Servo 2 (Y-axis) based on dRehmFlight controlANGLE()
    /*
     * Same structure as computePID_Servo1 but for Y-axis control
     */

    // Calculate current servo position from tilt angle (feedback)
    float servo2_actual = map(tilt_Y * 100, -max_tilt_angle * 100, max_tilt_angle * 100, 0, 180);
    servo2_actual = constrain(servo2_actual, 0, 180);

    // Calculate error: desired - actual
    error_servo2 = servo2_des - servo2_actual;

    // Integral term: accumulate error over time
    integral_servo2 = integral_servo2_prev + error_servo2 * dt;

    // Prevent integral windup (saturation)
    integral_servo2 = constrain(integral_servo2, -i_limit, i_limit);

    // Derivative term: use gyro rate for damping
    // GyroX corresponds to rotation around X-axis which affects Y-tilt
    derivative_servo2 = GyroX;

    // Compute PID output
    servo2_PID = Kp_servo2 * error_servo2 +
                 Ki_servo2 * integral_servo2 -
                 Kd_servo2 * derivative_servo2;

    // Constrain output to valid servo range
    servo2_PID = constrain(servo2_PID, 0, 180);

    // Update previous values for next iteration
    error_servo2_prev = error_servo2;
    integral_servo2_prev = integral_servo2;
}

//========================================================================================================================//
//                                                    TUNING GUIDE                                                        //
//========================================================================================================================//
/*
 * PID TUNING PROCESS (based on dRehmFlight methodology):
 *
 * 1. START WITH P ONLY:
 *    - Set Ki = 0, Kd = 0
 *    - Increase Kp until servo responds well to tilts
 *    - If too high: servo will oscillate
 *    - If too low: servo will be sluggish
 *    - Sweet spot: responsive but no oscillation
 *
 * 2. ADD INTEGRAL (I):
 *    - Start with small Ki (around 1-5)
 *    - I term eliminates steady-state error (servo reaching exact position)
 *    - If too high: slow oscillations or overshoot
 *    - Monitor integral_servoX in serial plotter to check for windup
 *
 * 3. ADD DERIVATIVE (D):
 *    - Start with small Kd (around 5-10)
 *    - D term provides damping (reduces overshoot and oscillation)
 *    - Uses gyro rate for better performance (cleaner than error derivative)
 *    - If too high: servo becomes too stiff/slow
 *    - If too low: may still have oscillations
 *
 * 4. FINE TUNE:
 *    - Adjust i_limit if integral windup is an issue
 *    - Adjust max_tilt_angle for desired sensitivity
 *    - Test with different tilt speeds and angles
 *
 * SUGGESTED STARTING VALUES (already set above):
 *    Kp = 45.0  (moderate response)
 *    Ki = 5.0   (small steady-state correction)
 *    Kd = 8.0   (light damping)
 *    i_limit = 25.0
 *
 * MONITORING:
 *    - Use Serial Plotter to observe:
 *      * TiltX, TiltY: actual tilt angles
 *      * Servo1_Des, Servo2_Des: desired positions (setpoints)
 *      * Servo1_Actual, Servo2_Actual: actual positions (outputs)
 *      * GyroX, GyroY: rates used for derivative term
 *
 * COMMON ISSUES:
 *    - Oscillation: Reduce Kp and/or increase Kd
 *    - Overshoot: Increase Kd
 *    - Slow response: Increase Kp
 *    - Doesn't reach target: Increase Ki
 *    - Integral windup: Reduce i_limit or Ki
 */
