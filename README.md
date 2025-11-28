# MPU6050 Diagnostic Tool for ESP32 with PID Servo Control

A comprehensive diagnostic utility for troubleshooting MPU6050 gyroscope/accelerometer sensor issues, **now featuring advanced PID controller for smooth servo control** based on dRehmFlight architecture. Helps identify wiring problems, I2C communication failures, sensor hardware issues, and provides production-grade servo stabilization.

## 📋 Overview

This diagnostic tool performs a complete health check of your MPU6050 sensor:
- **I2C Bus Scanning**: Detects all devices on I2C bus
- **Address Detection**: Finds MPU6050 at 0x68 or 0x69
- **WHO_AM_I Verification**: Confirms sensor identity
- **Wake-Up Test**: Ensures sensor responds to commands
- **Raw Data Reading**: Verifies sensor is producing valid data
- **🆕 PID Servo Control**: Advanced servo stabilization with feedback control
- **Real-time Visualization**: Serial Plotter integration for tuning

Perfect for:
- First-time MPU6050 setup
- Troubleshooting sensor problems
- Verifying wiring connections
- Diagnosing I2C issues
- Testing multiple sensors
- Hardware quality checks
- **🆕 Servo-based stabilization projects (gimbals, balancing robots, etc.)**
- **🆕 Learning PID control concepts**

## ✨ Features

### 🆕 PID Servo Control (NEW!)

**Advanced servo stabilization** based on [dRehmFlight](https://github.com/nickrehm/dRehmFlight) flight controller architecture:

#### Key Features
- ✅ **Proportional-Integral-Derivative (PID) control** for smooth, stable servo motion
- ✅ **Gyro-based derivative term** for superior damping (no oscillation)
- ✅ **Integral anti-windup** prevents overshoot and instability
- ✅ **Low-pass filtering** on all sensor inputs for noise rejection
- ✅ **Accurate timing** with dt calculation for consistent PID behavior
- ✅ **Tunable gains** (Kp, Ki, Kd) for customizable response
- ✅ **Serial Plotter integration** for visual tuning and monitoring

#### What This Means
**Before (simple mapping):** Servos would oscillate, overshoot, and respond erratically to tilts.  
**After (PID control):** Smooth, damped motion with precise positioning and predictable behavior.

#### Quick Start
1. Upload code with **default PID gains** (already set)
2. Open **Serial Plotter** (Tools → Serial Plotter)
3. Tilt MPU6050 and watch servos respond smoothly
4. See real-time tracking: `Desired` vs `Actual` positions
5. Adjust gains if needed (see [PID_TUNING_QUICK_REFERENCE.md](PID_TUNING_QUICK_REFERENCE.md))

#### Documentation
- 📘 **[PID_CONTROLLER_README.md](PID_CONTROLLER_README.md)** - Complete theory and implementation guide
- ⚡ **[PID_TUNING_QUICK_REFERENCE.md](PID_TUNING_QUICK_REFERENCE.md)** - Quick tuning lookup table
- 📝 **[PID_IMPLEMENTATION_SUMMARY.md](PID_IMPLEMENTATION_SUMMARY.md)** - What changed and why

### Diagnostic Steps

1. **I2C Bus Scan**
   - Scans all 127 possible I2C addresses
   - Lists all detected devices
   - Identifies MPU6050 specifically

2. **WHO_AM_I Register Check**
   - Reads sensor identity register
   - Expects 0x68 (correct)
   - Detects 0x00 or 0xFF (hardware failure)
   - Identifies unexpected values (wrong sensor/damage)

3. **Wake-Up Command**
   - Sends power management command
   - Ensures sensor exits sleep mode
   - Verifies command acknowledgment

4. **Raw Data Reading**
   - Reads all 6 axes (accel X/Y/Z, gyro X/Y/Z)
   - Checks for stuck-at-zero condition
   - Checks for stuck-at-max condition (0xFFFF)
   - Validates sensor is producing varying data

5. **Continuous Monitoring**
   - Streams live sensor data
   - Easy to verify physical movement
   - Perfect for testing responsiveness

### Output Format

**Serial Monitor Output**:
```
=================================
MPU6050 Diagnostic Tool
=================================

Step 1: Scanning I2C bus...
---------------------------------
Device found at address 0x68 (104)

---------------------------------
Step 2: Reading WHO_AM_I register from 0x68
---------------------------------
WHO_AM_I = 0x68 ✓ CORRECT (MPU6050 responding!)

---------------------------------
Step 3: Wake up MPU6050
---------------------------------
Wake up command sent ✓

---------------------------------
Step 4: Reading raw sensor data
---------------------------------
Raw values:
  Accel X: 512
  Accel Y: -234
  Accel Z: 16384
  Temp:    8456
  Gyro X:  23
  Gyro Y:  -45
  Gyro Z:  12

✓ Sensor appears to be working!
Values are changing, which is good.

=================================
Diagnostic complete!
=================================

AX:510 AY:-230 AZ:16380 | GX:24 GY:-44 GZ:13
AX:508 AY:-232 AZ:16382 | GX:22 GY:-46 GZ:11
...
```

## 🔧 Hardware Requirements

### Components
- **ESP32 Development Board** (ESP32-WROOM-32 or similar)
- **MPU6050** 6-axis Gyroscope + Accelerometer module
- **Jumper wires** (4 wires needed)
- **Breadboard** (optional)
- **USB cable** for programming and serial monitor

### Wiring Diagram

#### MPU6050 Connection
```
MPU6050 → ESP32
─────────────────
VCC    → 3.3V
GND    → GND
SDA    → GPIO 21
SCL    → GPIO 22
```

#### 🆕 Servo Connections (for PID Control)
```
SERVO 1 (X-axis) → ESP32
────────────────────────
Signal (Yellow/White) → GPIO 25
VCC (Red)            → 5V EXTERNAL SUPPLY
GND (Brown/Black)    → GND (shared with ESP32)

SERVO 2 (Y-axis) → ESP32
────────────────────────
Signal (Yellow/White) → GPIO 26
VCC (Red)            → 5V EXTERNAL SUPPLY
GND (Brown/Black)    → GND (shared with ESP32)
```

**⚠️ Important Notes**:
- MPU6050 uses **3.3V** power (some modules support 5V via onboard regulator)
- **🆕 Updated I2C pins** (21/22) - standard ESP32 I2C pins
- No external pull-up resistors needed (usually on module)
- AD0 pin determines I2C address:
  - **AD0 LOW** (default): Address 0x68
  - **AD0 HIGH**: Address 0x69
- **🆕 Servos require EXTERNAL 5V supply** - do NOT power from ESP32 3.3V pin!
- Common GND between ESP32 and servo power supply

## 💻 Software Requirements

### Arduino IDE Setup

1. **Install ESP32 Board Support**
   - Open Arduino IDE → `File` → `Preferences`
   - Add to "Additional Board Manager URLs":
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to `Tools` → `Board` → `Board Manager`
   - Search "ESP32" and install "esp32 by Espressif Systems"

2. **🆕 Install ESP32Servo Library**
   - Go to `Tools` → `Manage Libraries`
   - Search "ESP32Servo"
   - Install "ESP32Servo by Kevin Harrington"
   - Required for PID servo control

3. **Other Libraries**
   - Uses built-in `Wire.h` (I2C library)
   - Uses built-in `Arduino.h` (core functions)

### Board Configuration
- **Board**: ESP32 Dev Module
- **Upload Speed**: 921600
- **Flash Frequency**: 80MHz
- **Flash Size**: 4MB
- **Partition Scheme**: Default
- **Port**: Select your ESP32 COM port

## 📦 Installation

1. **Wire the MPU6050**: Follow wiring diagram above
2. **Open** `mpu6050_diagnostic.ino` in Arduino IDE
3. **Select** your ESP32 board and COM port
4. **Click Upload**
5. **Open Serial Monitor** at **115200 baud**
6. **Press ESP32 reset button** to restart diagnostic
7. **Read results** and troubleshoot as needed

## 🎮 Usage

### 🆕 Running with PID Servo Control (Recommended)

1. **Upload sketch**
2. **Open Serial Monitor** (Tools → Serial Monitor, 115200 baud)
3. **Read diagnostic output** - verify MPU6050 is working
4. **Wait 3 seconds** for automatic switch to PID mode
5. **Switch to Serial Plotter** (Tools → Serial Plotter)
6. **Tilt MPU6050** and watch servos respond with smooth PID control!

#### What You'll See in Serial Plotter
- `TiltX`, `TiltY`: Current tilt angles from sensor
- `Servo1_Des`, `Servo2_Des`: Where servos should be (setpoints)
- `Servo1_Actual`, `Servo2_Actual`: Where servos actually are (PID outputs)
- `GyroX`, `GyroY`: Rotation rates used for damping

**Good tuning:** Actual closely follows Desired  
**Needs tuning:** Actual oscillates or lags behind Desired

### Running Diagnostic Only (Classic Mode)

1. **Upload sketch**
2. **Open Serial Monitor** (Tools → Serial Monitor)
3. **Set baud rate to 115200**
4. **Press ESP32 reset button** or **power cycle**
5. **Wait 2 seconds** for diagnostic to start
6. **Read diagnostic output**

### Interpreting Results

#### ✓ Success (All Good)
```
Device found at address 0x68 (104)
WHO_AM_I = 0x68 ✓ CORRECT (MPU6050 responding!)
Wake up command sent ✓
✓ Sensor appears to be working!
```

**Result**: MPU6050 is working perfectly! Proceed with your project.

#### ❌ No Device Found
```
>>> ERROR: No MPU6050 found on I2C bus!

Possible issues:
1. Wiring problem - check connections
2. MPU6050 is damaged
3. Wrong I2C pins configured
4. Power supply issue (VCC should be 3.3V)
```

**Solutions**:
1. **Check wiring** carefully
   - SDA → GPIO 33 (not 21!)
   - SCL → GPIO 32 (not 22!)
   - VCC → 3.3V
   - GND → GND
2. **Verify power**: LED on MPU6050 module should be lit
3. **Try different cables**: Poor connections common with jumpers
4. **Test with another MPU6050**: Module may be faulty

#### ⚠️ Wrong WHO_AM_I Value
```
WHO_AM_I = 0x00 ✗ INCORRECT (likely hardware failure)
```
or
```
WHO_AM_I = 0xFF ✗ INCORRECT (likely hardware failure)
```

**Diagnosis**: MPU6050 is physically damaged or defective

**Solutions**:
1. **Try another module**: This one is likely dead
2. **Check for physical damage**: Cracks, burnt components
3. **Verify voltage**: Did you accidentally use 5V on 3.3V-only module?

#### ⚠️ All Zeros or All -1
```
Raw values:
  Accel X: 0
  Accel Y: 0
  Accel Z: 0
  ...

>>> WARNING: All values are same!
This usually means the sensor is damaged or in sleep mode
```

**Solutions**:
1. **Power cycle**: Unplug and reconnect
2. **Check wake-up**: Wake command may have failed
3. **Test another sensor**: Module may be faulty

#### ⚠️ No Continuous Data
```
>>> No data from sensor
>>> No data from sensor
...
```

**Solutions**:
1. **Check wiring** again (especially SDA/SCL)
2. **Verify I2C address**: Try 0x69 if 0x68 doesn't work
3. **Look for loose connections**

## 🔍 Understanding I2C Addresses

### Default Address: 0x68
- **AD0 pin** is LOW (connected to GND)
- Most common configuration
- Default for most MPU6050 modules

### Alternative Address: 0x69
- **AD0 pin** is HIGH (connected to VCC)
- Used when multiple MPU6050s on same bus
- Less common

### Changing Address in Code
```cpp
#define MPU6050_ADDR 0x68      // Default
#define MPU6050_ADDR_ALT 0x69  // Alternative

// To test alternative address:
Wire.beginTransmission(MPU6050_ADDR_ALT);
```

## 🛠️ Advanced Troubleshooting

### Problem: I2C Bus Shows Other Devices But Not MPU6050

**Possible Causes**:
- Wrong device on bus (check other sensors)
- MPU6050 not powered properly
- MPU6050 in deep sleep mode

**Solutions**:
1. Disconnect other I2C devices
2. Test MPU6050 alone
3. Verify 3.3V at MPU6050 VCC pin with multimeter

### Problem: Intermittent Connection

**Possible Causes**:
- Loose jumper wires
- Breadboard connection issues
- Electromagnetic interference
- Poor solder joints on module

**Solutions**:
1. Use shorter wires (<15cm recommended)
2. Solder wires directly to module
3. Add pull-up resistors (4.7kΩ) if very long wires
4. Keep away from motors and power wires

### Problem: Works Sometimes, Not Always

**Possible Causes**:
- Power supply noise
- Brown-out resets
- Temperature-related issues

**Solutions**:
1. Add decoupling capacitor (100nF) near MPU6050 VCC
2. Use quality USB power supply
3. Check for overheating (touch sensor)

## 📚 MPU6050 Technical Details

### I2C Registers Used

**WHO_AM_I Register (0x75)**:
- Contains device ID
- Should read 0x68
- Non-volatile (doesn't change)

**PWR_MGMT_1 Register (0x6B)**:
- Power management control
- Write 0x00 to wake up sensor
- Sensor starts in sleep mode

**Accel/Gyro Data Registers (0x3B-0x48)**:
- 14 bytes total
- Accel X/Y/Z (6 bytes)
- Temperature (2 bytes)
- Gyro X/Y/Z (6 bytes)
- 16-bit signed integers
- Big-endian byte order

### I2C Communication Timing

- **Clock Speed**: 100kHz (standard mode)
- **Max Speed**: 400kHz (fast mode) - supported by MPU6050
- **Response Time**: <5ms typically

### Expected Raw Values (At Rest)

**Accelerometer** (16-bit, ±2g scale):
- X, Y ≈ 0 (±500)
- Z ≈ 16384 (1g gravity)

**Gyroscope** (16-bit, ±250°/s scale):
- All axes ≈ 0 (±100)
- Small drift is normal

**Temperature**:
- Formula: `Temp_°C = (raw / 340) + 36.53`
- Typical range: 5000-12000 (raw)

## ⚙️ Configuration

### Change I2C Pins
```cpp
#define SDA_PIN 33  // Change to your preference
#define SCL_PIN 32  // Change to your preference
```

**Standard ESP32 I2C**: GPIO 21 (SDA), GPIO 22 (SCL)
**This project uses**: GPIO 33 (SDA), GPIO 32 (SCL)

### Adjust I2C Speed
```cpp
Wire.setClock(100000);  // 100kHz (current)

// For faster communication
Wire.setClock(400000);  // 400kHz (fast mode)

// For problematic/long wires
Wire.setClock(50000);   // 50kHz (slower but more reliable)
```

### Continuous Monitoring Speed
```cpp
delay(1000);  // Currently 1 second between readings

// Faster monitoring
delay(100);   // 10Hz update rate

// Slower monitoring
delay(5000);  // Every 5 seconds
```

## 📊 Example Outputs

### Healthy Sensor
```
=================================
MPU6050 Diagnostic Tool
=================================

Step 1: Scanning I2C bus...
---------------------------------
Device found at address 0x68 (104)

---------------------------------
Step 2: Reading WHO_AM_I register from 0x68
---------------------------------
WHO_AM_I = 0x68 ✓ CORRECT (MPU6050 responding!)

---------------------------------
Step 3: Wake up MPU6050
---------------------------------
Wake up command sent ✓

---------------------------------
Step 4: Reading raw sensor data
---------------------------------
Raw values:
  Accel X: 245
  Accel Y: -123
  Accel Z: 16389
  Temp:    8234
  Gyro X:  -12
  Gyro Y:  34
  Gyro Z:  -8

✓ Sensor appears to be working!
Values are changing, which is good.
```

### Faulty Sensor (Dead)
```
Step 1: Scanning I2C bus...
---------------------------------
Device found at address 0x68 (104)

---------------------------------
Step 2: Reading WHO_AM_I register from 0x68
---------------------------------
WHO_AM_I = 0xFF ✗ INCORRECT (likely hardware failure)

>>> MPU6050 appears to be damaged or not powered
```

### Wiring Problem
```
Step 1: Scanning I2C bus...
---------------------------------

>>> ERROR: No MPU6050 found on I2C bus!

Possible issues:
1. Wiring problem - check connections
2. MPU6050 is damaged
3. Wrong I2C pins configured
4. Power supply issue (VCC should be 3.3V)
```

## 🎯 Next Steps After Diagnosis

### If Sensor Works (✓)
1. **🆕 Start using PID servo control**: Already built-in!
2. **🆕 Tune PID gains**: See [PID_TUNING_QUICK_REFERENCE.md](PID_TUNING_QUICK_REFERENCE.md)
3. **Use in your project**: Adapt for gimbals, balancing robots, etc.
4. **Note the address**: 0x68 or 0x69 for your code
5. **Test movement**: Physical movement should change readings smoothly

### 🆕 PID Tuning Workflow
1. **Start with defaults**: Kp=45, Ki=5, Kd=8 (already set)
2. **Test response**: Tilt MPU6050 and observe servo
3. **If oscillates**: Reduce Kp or increase Kd
4. **If too slow**: Increase Kp or reduce Kd
5. **If doesn't reach target**: Increase Ki
6. **Use Serial Plotter**: Visual feedback is key!

### If Sensor Fails (❌)
1. **Double-check wiring**: Most common issue
2. **Try different wires**: Jumpers can be faulty
3. **Test with multimeter**: Verify 3.3V at VCC, GND at GND
4. **Replace module**: If truly dead, get a new one

## 🚀 Related Projects

After successful diagnosis, try:
- **gyro_st7735.ino**: Full gyroscope monitor with display
- **gyro.ino**: Serial plotter version for data logging
- **🆕 This project**: Now includes production-grade PID servo control!

## 📚 PID Control Resources

### Documentation Files
1. **[PID_CONTROLLER_README.md](PID_CONTROLLER_README.md)** (51KB)
   - Complete theory of PID control
   - How it works, why it's better
   - Key features from dRehmFlight
   - Step-by-step tuning guide
   - Troubleshooting section

2. **[PID_TUNING_QUICK_REFERENCE.md](PID_TUNING_QUICK_REFERENCE.md)** (6KB)
   - Problem → Solution matrix
   - 3-step tuning process
   - Preset configurations
   - Serial Plotter interpretation
   - Emergency reset values

3. **[PID_IMPLEMENTATION_SUMMARY.md](PID_IMPLEMENTATION_SUMMARY.md)** (10KB)
   - What changed and why
   - Before/after comparison
   - Key features summary
   - Quick start guide

### Based On
- **[dRehmFlight](https://github.com/nickrehm/dRehmFlight)** by Nicholas Rehm
  - Proven flight controller architecture
  - Key techniques: integral anti-windup, gyro-based derivative
  - Adapted for servo position control

## 📝 Version History

- **V2.0** (Current): 🆕 **PID Servo Control**
  - Advanced PID controller based on dRehmFlight
  - Gyro-based derivative for superior damping
  - Integral anti-windup for stability
  - Serial Plotter integration for tuning
  - Comprehensive documentation (3 guides)
  - Smooth, predictable servo motion

- **V1.0**: Initial diagnostic tool
  - Complete I2C bus scanning
  - WHO_AM_I verification
  - Wake-up command test
  - Raw data validation
  - Continuous monitoring mode

## 📄 License

This tool is provided free for educational and troubleshooting purposes.

## 🙏 Credits

Built using:
- ESP32 Arduino Core by Espressif
- Standard Wire library (I2C)
- ESP32Servo library by Kevin Harrington
- **🆕 PID control architecture inspired by [dRehmFlight](https://github.com/nickrehm/dRehmFlight) by Nicholas Rehm**

Special thanks to:
- **Nicholas Rehm** for the dRehmFlight project - excellent reference for PID stabilization
- **Flight controller community** for proven control techniques
- **ESP32 community** for Arduino framework support

---

**Debug with confidence! Now with smooth PID servo control!** 🔧🔍✅🎯

