# Complete Documentation Index

## Overview

This documentation provides a comprehensive, deep-level understanding of the MPU6050-based flight controller system with elevon mixing and ESP-NOW telemetry.

---

## 📚 Documentation Files

### 1. **PID Controller Mathematics** 
**File:** `DOCS_PID_MATHEMATICS.md`

**Topics covered:**
- Mathematical foundation of PID control
- Proportional, Integral, and Derivative terms (detailed analysis)
- Discrete implementation and Z-transforms
- Anti-windup protection mechanisms
- Gyro-based derivative (why it's better than error derivative)
- Tuning mathematics (Ziegler-Nichols, empirical methods)
- Performance metrics (rise time, settling time, overshoot)
- Frequency response analysis

**Who should read this:**
- Anyone wanting to understand how PID control works at a deep mathematical level
- Those tuning PID gains
- Engineers implementing control systems

---

### 2. **Hardware Wiring and Circuit Diagrams**
**File:** `DOCS_HARDWARE_WIRING.md`

**Topics covered:**
- Complete system wiring diagrams (transmitter and receiver)
- Pin mappings and connections
- Component specifications (ESP32, MPU6050, servos, ST7735)
- Power distribution and calculations
- Signal levels and logic compatibility
- Wiring best practices
- Common mistakes to avoid
- Troubleshooting hardware issues

**Who should read this:**
- Anyone building the physical system
- Those debugging hardware problems
- People wanting to understand the electronics

---

### 3. **Transmitter Architecture**
**File:** `DOCS_TRANSMITTER_ARCHITECTURE.md`

**Topics covered:**
- Software architecture and module organization
- Complete data flow pipeline
- Core components (MPU6050 interface, filters, PID, mixer, servos, ESP-NOW)
- Timing and scheduling analysis
- State machine
- Memory management
- CPU load and performance analysis
- Latency breakdown

**Who should read this:**
- Software developers working on the transmitter
- Those wanting to understand the control loop
- People optimizing performance

---

### 4. **Receiver Architecture**
**File:** `DOCS_RECEIVER_ARCHITECTURE.md`

**Topics covered:**
- Ground station software architecture
- Display system (ST7735 LCD details)
- User interface and menu system
- Data reception via ESP-NOW
- Screen layouts (6 different views)
- Screen update strategies (partial updates)
- Performance analysis
- Future enhancements

**Who should read this:**
- Developers working on the receiver/display
- Those wanting to add new screens or features
- Anyone understanding the telemetry system

---

### 5. **ESP-NOW Communication Protocol**
**File:** `DOCS_ESP_NOW_PROTOCOL.md`

**Topics covered:**
- ESP-NOW fundamentals and protocol stack
- Physical layer (2.4 GHz WiFi)
- Data link layer (802.11 action frames)
- Packet structure and byte layout
- Pairing and discovery process
- Implementation details (initialization, sending, receiving)
- Performance analysis (throughput, latency, packet loss)
- Comparison with WiFi UDP, Bluetooth, and RC systems
- Troubleshooting and advanced topics

**Who should read this:**
- Anyone wanting to understand wireless communication
- Those debugging connection issues
- Engineers considering ESP-NOW for their projects

---

### 6. **Elevon Mixing Mathematics**
**File:** `DOCS_ELEVON_MIXING.md`

**Topics covered:**
- Introduction to elevons (what they are and where used)
- Control surface theory (pitch, roll, yaw)
- Mixing mathematics (standard and modified formulas)
- Vector and matrix representations
- Geometric analysis (pure pitch, pure roll, combined)
- Servo mechanics and linkage geometry
- Range limiting and saturation handling
- Control authority and coupling
- Coordinate system transformations
- Aerodynamic principles
- Troubleshooting mixing issues

**Who should read this:**
- Anyone working with flying wings or tailsitters
- Those debugging servo direction issues
- Control engineers

---

### 7. **Sensor Fusion and Filtering**
**File:** `DOCS_SENSOR_FUSION_FILTERING.md`

**Topics covered:**
- MPU6050 sensor physics (MEMS accelerometer and gyroscope)
- Accelerometer-based angle calculation (detailed math)
- Low-pass filtering (IIR filter design and analysis)
- Sensor fusion methods (complementary filter, Kalman filter - not implemented)
- Calibration process and accuracy
- Noise analysis and sources
- Performance optimization
- Advanced topics (temperature compensation, adaptive filtering)

**Who should read this:**
- Those wanting to understand sensor processing
- Engineers working with IMU sensors
- Anyone debugging noisy sensor readings
- Those implementing sensor fusion

---

## 🔧 Quick Reference Guides

The following files provide quick, practical information:

- **`QUICK_START.md`** - Get started quickly with the system
- **`PID_TUNING_QUICK_REFERENCE.md`** - Quick PID tuning guide
- **`ESP_NOW_SETUP_GUIDE.md`** - ESP-NOW setup instructions
- **`ELEVON_CONFIGURATION.md`** - Elevon configuration overview
- **`START_HERE.md`** - Entry point for new users

---

## 📊 Technical Specifications Summary

### System Performance

| Metric | Value |
|--------|-------|
| **Control Loop Rate** | 100 Hz (10ms) |
| **Telemetry Rate** | 50 Hz (20ms) |
| **Sensor-to-Actuator Latency** | 22-52 ms |
| **Wireless Latency** | 1-6 ms (typical: 3ms) |
| **Display Update Rate** | 50 Hz |
| **CPU Usage (Transmitter)** | ~15% |
| **CPU Usage (Receiver)** | ~50% |
| **RAM Usage (Transmitter)** | ~70 KB (21%) |
| **RAM Usage (Receiver)** | ~75 KB (23%) |

### Control Parameters

| Parameter | Value | Range |
|-----------|-------|-------|
| **Pitch Kp** | 35.0 | 30-60 |
| **Pitch Ki** | 3.0 | 1-10 |
| **Pitch Kd** | 8.0 | 5-15 |
| **Roll Kp** | 35.0 | 30-60 |
| **Roll Ki** | 3.0 | 1-10 |
| **Roll Kd** | 8.0 | 5-15 |
| **Integral Limit** | 15.0°·s | 10-25 |
| **Max Tilt Angle** | 45° | 30-60 |

### Hardware Specifications

| Component | Model | Specifications |
|-----------|-------|----------------|
| **Microcontroller** | ESP32 DevKit V1 | Dual-core 240MHz, 520KB RAM, WiFi |
| **IMU Sensor** | MPU6050 | ±2g accel, ±250°/s gyro, 16-bit |
| **Servos** | Standard (SG90) | 30°-150° range, 5V, PWM 50Hz |
| **Display** | ST7735 | 1.8" 128×160 TFT LCD, SPI 40MHz |
| **Wireless** | ESP-NOW | 2.4GHz, 250 kbps, 200-400m range |

---

## 🎯 Learning Paths

### Path 1: Beginner (Just Want to Use It)
1. `START_HERE.md` - Introduction
2. `QUICK_START.md` - Setup and run
3. `ESP_NOW_SETUP_GUIDE.md` - Connect transmitter and receiver
4. `PID_TUNING_QUICK_REFERENCE.md` - Adjust for your aircraft

### Path 2: Intermediate (Want to Modify)
1. Start with Beginner path
2. `DOCS_HARDWARE_WIRING.md` - Understand connections
3. `DOCS_TRANSMITTER_ARCHITECTURE.md` - Understand software structure
4. `DOCS_ELEVON_MIXING.md` - Understand control surface mixing
5. Modify code to suit your needs

### Path 3: Advanced (Deep Understanding)
1. All files in order:
   - `DOCS_PID_MATHEMATICS.md`
   - `DOCS_SENSOR_FUSION_FILTERING.md`
   - `DOCS_ELEVON_MIXING.md`
   - `DOCS_TRANSMITTER_ARCHITECTURE.md`
   - `DOCS_RECEIVER_ARCHITECTURE.md`
   - `DOCS_ESP_NOW_PROTOCOL.md`
   - `DOCS_HARDWARE_WIRING.md`
2. Experiment with modifications
3. Implement your own features

### Path 4: Specific Topics

**PID Control:**
- `DOCS_PID_MATHEMATICS.md` (complete theory)
- `PID_TUNING_QUICK_REFERENCE.md` (practical tuning)
- `PID_CONTROLLER_README.md` (overview)

**Hardware:**
- `DOCS_HARDWARE_WIRING.md` (complete wiring)
- Hardware setup sections in `START_HERE.md`

**Wireless Communication:**
- `DOCS_ESP_NOW_PROTOCOL.md` (deep dive)
- `ESP_NOW_SETUP_GUIDE.md` (quick setup)

**Sensor Processing:**
- `DOCS_SENSOR_FUSION_FILTERING.md` (complete theory)
- Calibration section in transmitter code

**Elevon Control:**
- `DOCS_ELEVON_MIXING.md` (complete theory)
- `ELEVON_CONFIGURATION.md` (practical setup)

---

## 🔍 Search by Topic

### Control Theory
- PID mathematics → `DOCS_PID_MATHEMATICS.md`
- Elevon mixing → `DOCS_ELEVON_MIXING.md`
- Sensor fusion → `DOCS_SENSOR_FUSION_FILTERING.md`

### Software
- Transmitter code → `DOCS_TRANSMITTER_ARCHITECTURE.md`
- Receiver code → `DOCS_RECEIVER_ARCHITECTURE.md`
- Communication → `DOCS_ESP_NOW_PROTOCOL.md`

### Hardware
- Wiring → `DOCS_HARDWARE_WIRING.md`
- Components → `DOCS_HARDWARE_WIRING.md` (specifications section)
- Power → `DOCS_HARDWARE_WIRING.md` (power distribution section)

### Tuning and Calibration
- PID tuning → `PID_TUNING_QUICK_REFERENCE.md`, `DOCS_PID_MATHEMATICS.md`
- Sensor calibration → `DOCS_SENSOR_FUSION_FILTERING.md`
- Servo configuration → `ELEVON_CONFIGURATION.md`, `DOCS_ELEVON_MIXING.md`

### Troubleshooting
- Hardware issues → `DOCS_HARDWARE_WIRING.md` (troubleshooting section)
- Servo problems → `DOCS_ELEVON_MIXING.md` (troubleshooting section)
- Connection issues → `DOCS_ESP_NOW_PROTOCOL.md` (troubleshooting section)
- Jitter/oscillation → `DOCS_PID_MATHEMATICS.md`, `PID_TUNING_QUICK_REFERENCE.md`

---

## 📈 Documentation Statistics

- **Total pages:** ~250 pages (if printed)
- **Total words:** ~60,000 words
- **Total code examples:** 100+
- **Total diagrams:** 80+
- **Total tables:** 50+
- **Coverage:** Complete system (hardware, software, theory, practice)

---

## 🤝 Contributing

Found an error or want to improve documentation?
1. Note the file and section
2. Describe the issue or improvement
3. Submit to project maintainer

---

## 📝 License

This documentation is part of the MPU6050 Flight Controller project.
Free to use for educational and personal projects.

---

## ✅ Version History

- **v1.0** (Current) - Complete documentation suite
  - 7 deep-dive documents
  - Complete system coverage
  - Theory and practice combined
  - Beginner to advanced

---

**Last Updated:** November 27, 2025

**For questions or support:** Refer to specific document sections or contact project maintainer.

---

## 🚀 What to Read Next?

**New to the project?** → Start with `START_HERE.md`

**Want to build it?** → Go to `DOCS_HARDWARE_WIRING.md`

**Want to understand PID?** → Read `DOCS_PID_MATHEMATICS.md`

**Having issues?** → Check troubleshooting sections in relevant docs

**Want deep understanding?** → Read all 7 deep-dive documents in order!

---

**Happy learning! 🎓**

