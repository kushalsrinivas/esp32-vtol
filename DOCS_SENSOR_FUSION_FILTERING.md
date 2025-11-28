# Sensor Fusion and Filtering - Deep Dive

## Table of Contents
1. [Introduction](#introduction)
2. [MPU6050 Sensor Physics](#mpu6050-sensor-physics)
3. [Accelerometer-Based Angles](#accelerometer-based-angles)
4. [Low-Pass Filtering](#low-pass-filtering)
5. [Sensor Fusion (Not Implemented)](#sensor-fusion-not-implemented)
6. [Calibration](#calibration)
7. [Noise Analysis](#noise-analysis)
8. [Performance Optimization](#performance-optimization)

---

## Introduction

### The Sensor Fusion Problem

**Goal:** Determine aircraft orientation (pitch and roll angles)

**Available sensors:**
1. **Accelerometer**: Measures linear acceleration (gravity + motion)
2. **Gyroscope**: Measures angular velocity (rotation rates)

**Challenges:**
```
Accelerometer:
✓ Accurate when stationary (measures gravity direction)
✗ Noisy (vibration, high-frequency noise)
✗ Incorrect during acceleration (thrust, turns)
✗ No yaw information

Gyroscope:
✓ Clean signal (less noisy)
✓ Works during acceleration
✓ Fast response (high bandwidth)
✗ Drifts over time (integration error)
✗ Gives rates, not angles (must integrate)
```

**Solution:** Sensor fusion (combine both sensors' strengths)

---

## MPU6050 Sensor Physics

### MEMS Accelerometer

```
┌─────────────────────────────────────────────────────────────────┐
│             MEMS ACCELEROMETER PRINCIPLE                         │
└─────────────────────────────────────────────────────────────────┘

Structure: Micro-machined proof mass on spring

         Fixed frame
    ┌────────────────────┐
    │                    │
    │    ╔═══════╗       │
    │ ═══╣  MASS ╠═══    │  ← Springs
    │    ╚═══════╝       │
    │                    │
    └────────────────────┘
        │         │
     Capacitor  Capacitor
      plates     plates

At rest:
  Mass centered, capacitors balanced

Under acceleration:
  Mass shifts → capacitance changes → voltage changes
  
Measures: Force per unit mass (F/m = a)
  F = ma → a = F/m

Accelerometer measures:
  a_measured = a_true + g

Where g = gravity (9.81 m/s²)

When stationary:
  a_measured = g (only gravity!)
  This is how we find "down" direction
```

### 3-Axis Accelerometer

```
MPU6050 has 3 accelerometers (X, Y, Z):

         Z (up)
         │
         │
         ◄────► Y (forward)
        ╱
       ╱
      X (right)

Each axis measures acceleration along that direction.

When level and stationary:
  AccelX = 0 g   (no sideways acceleration)
  AccelY = 0 g   (no forward acceleration)
  AccelZ = 1 g   (gravity pulling down)

When tilted (pitch 30°):
  AccelX = 0 g
  AccelY = sin(30°) × 1g = 0.5 g
  AccelZ = cos(30°) × 1g = 0.87 g
```

### MEMS Gyroscope

```
┌─────────────────────────────────────────────────────────────────┐
│               MEMS GYROSCOPE PRINCIPLE                           │
└─────────────────────────────────────────────────────────────────┘

Structure: Vibrating proof mass (Coriolis effect)

        Drive
      oscillation
    ────────────►
    │           │
    │   MASS    │
    │           │
    └───────────┘
          │
      Sense motion
        (when rotating)

Coriolis force:
  F_c = 2m(ω × v)

Where:
- m: Mass
- ω: Angular velocity (what we want to measure)
- v: Linear velocity (driven oscillation)

When rotating:
  Mass vibrates in drive direction
  Rotation causes perpendicular force (Coriolis)
  Perpendicular motion detected
  Proportional to rotation rate

Measures: Angular velocity (°/s or rad/s)
  ω = dθ/dt

To get angle:
  θ = ∫ ω dt  (integrate over time)
```

### Sensor Ranges and Resolution

```
MPU6050 Accelerometer:
──────────────────────
Selectable ranges: ±2g, ±4g, ±8g, ±16g
We use: ±2g (most sensitive)

Resolution: 16-bit ADC
  Range: -32768 to +32767 (65536 steps)
  Scale factor: 32768 / 2g = 16384 LSB/g
  
Sensitivity:
  1 LSB = 1/16384 g ≈ 0.061 mg
  At 2g range: 0.061 mg resolution ✓

Noise:
  Typical: 400 µg/√Hz
  At 100 Hz bandwidth: 400µg × √100 = 4 mg RMS

MPU6050 Gyroscope:
──────────────────
Selectable ranges: ±250, ±500, ±1000, ±2000 °/s
We use: ±250 °/s (most sensitive)

Resolution: 16-bit ADC
  Scale factor: 32768 / 250 = 131 LSB/(°/s)
  
Sensitivity:
  1 LSB = 1/131 °/s ≈ 0.0076 °/s
  Very sensitive! ✓

Noise:
  Typical: 0.005 °/s/√Hz
  At 100 Hz: 0.005 × √100 = 0.05 °/s RMS
```

---

## Accelerometer-Based Angles

### Mathematical Derivation

```
Goal: Find pitch and roll from accelerometer readings

Assumptions:
1. Aircraft is NOT accelerating (only gravity acts)
2. Gravity magnitude = 1g
3. We know gravity direction in sensor frame

Geometry:

         Z                    
         │  ╱                 Gravity vector g = [gx, gy, gz]
         │ ╱ g                Points "down" in world frame
         │╱                   
         ◄────── Y            Measured as acceleration in sensor frame
        ╱│
       ╱ │
      X  │

When level:
  g = [0, 0, -1]  (down is negative Z)

When pitched θ (nose up):
  g rotates in YZ plane
  g = [0, sin(θ), -cos(θ)]
  
When rolled φ (right wing down):
  g rotates in XZ plane
  g = [sin(φ), 0, -cos(φ)]

General case (pitch AND roll):
  gx = -sin(φ) × cos(θ)
  gy = sin(θ)
  gz = -cos(φ) × cos(θ)
```

### Pitch Angle

```
From geometry:
  gy = sin(θ)
  gz = -cos(φ) × cos(θ)
  
  tan(θ) = gy / √(gx² + gz²)
  
  θ = atan2(gy, √(gx² + gz²))

Why use atan2?
- Handles all quadrants correctly
- Returns -π to +π (or -180° to +180°)
- Works when denominator is zero

In code:
```cpp
pitch = atan2(AccelY, sqrt(AccelX*AccelX + AccelZ*AccelZ)) × 57.2957795;
                                                             └─ Radians to degrees
```

Example:
  AccelY = 0.5g, AccelX = 0g, AccelZ = 0.87g
  
  pitch = atan2(0.5, sqrt(0 + 0.87²))
        = atan2(0.5, 0.87)
        = atan(0.5/0.87)
        = atan(0.575)
        = 0.524 rad
        = 30° ✓
```

### Roll Angle

```
From geometry:
  gx = -sin(φ) × cos(θ)
  gy = sin(θ)
  gz = -cos(φ) × cos(θ)
  
For small θ, cos(θ) ≈ 1:
  gx ≈ -sin(φ)
  gz ≈ -cos(φ)
  
  tan(φ) = -gx / gz
  φ = atan2(-gx, gz)

But we want symmetry with pitch, so:
  φ = atan2(-gx, √(gy² + gz²))

In code:
```cpp
roll = atan2(-AccelX, sqrt(AccelY*AccelY + AccelZ*AccelZ)) × 57.2957795;
```

Note the negative sign on AccelX!
This defines positive roll as right wing down.

Example:
  AccelX = -0.5g, AccelY = 0g, AccelZ = 0.87g
  
  roll = atan2(-(-0.5), sqrt(0 + 0.87²))
       = atan2(0.5, 0.87)
       = 30° ✓ (right wing down)
```

### Limitations

```
1. ONLY WORKS WHEN STATIONARY
   ────────────────────────────
   During acceleration:
     a_measured = a_motion + g
     
   Accelerometer can't distinguish!
   
   Example: 1g forward acceleration
     AccelY = 0g (gravity) + 1g (thrust) = 1g
     Looks like 45° pitch up!
     But aircraft might be level ✗

2. SENSITIVE TO VIBRATION
   ───────────────────────
   Vibration → high-frequency acceleration
   Appears as noise on angle measurements
   Solution: Low-pass filter

3. NO YAW INFORMATION
   ───────────────────
   Gravity is always "down"
   Can't determine heading from accelerometer alone
   Need magnetometer or GPS for yaw

4. GIMBAL LOCK AT 90°
   ───────────────────
   At pitch = ±90° (vertical):
     gz ≈ 0
     Denominator in atan2 → 0
     Roll becomes undefined
   For aircraft, limit to ±45° to avoid this
```

---

## Low-Pass Filtering

### Why Filter?

```
Raw accelerometer signal (example):

Angle (°)
  10 │     ╱╲     ╱╲     ╱╲
     │    ╱  ╲   ╱  ╲   ╱  ╲    ← High-frequency noise
   5 │   ╱    ╲ ╱    ╲ ╱    ╲     (vibration, motor)
     │  ╱      ╳      ╳      ╲
   0 ├─╱──────╱─╲────╱─╲──────╲─► Time
     │╱      ╱   ╲  ╱   ╲      
  -5 │      ╱     ╲╱     ╲      ← True signal
     │                          (aircraft motion)

Goals of filtering:
✓ Remove high-frequency noise (>10 Hz)
✓ Preserve low-frequency motion (<5 Hz)
✓ Low computational cost
✓ No phase lag (or minimal)
```

### First-Order IIR Filter

```
┌─────────────────────────────────────────────────────────────────┐
│           FIRST-ORDER LOW-PASS FILTER (IIR)                      │
└─────────────────────────────────────────────────────────────────┘

Equation:
  y(n) = (1 - B) × y(n-1) + B × x(n)

Where:
- y(n): Filtered output at time n
- y(n-1): Previous filtered output
- x(n): New raw input
- B: Filter coefficient (0 < B < 1)

Characteristics:
- IIR: Infinite Impulse Response (uses past outputs)
- Recursive: Each output depends on previous output
- Memory: Only needs to store one previous value
- Efficient: 2 multiplications, 1 addition per sample

Transfer function (Z-domain):
         B
H(z) = ─────────
       1 - (1-B)z⁻¹

Cutoff frequency:
  f_c = f_sample × B / (2π × (1 - B))
```

### Implementation

```cpp
// Accelerometer filter (B = 0.14):
AccelX = (1.0 - B_ACCEL) × AccelX_prev + B_ACCEL × AccelX_raw;
AccelX_prev = AccelX;

// Expanded:
AccelX = 0.86 × AccelX_prev + 0.14 × AccelX_raw;

Interpretation:
- 86% of output comes from previous (smoothed) value
- 14% comes from new (noisy) raw measurement
- Acts as moving average with exponential weighting
```

### Frequency Response

```
For B_ACCEL = 0.14, f_sample = 100 Hz:

Cutoff frequency:
  f_c = 100 × 0.14 / (2π × 0.86)
      = 14 / 5.40
      = 2.6 Hz

Magnitude response:
Gain
(dB)
  0 ─┐
     │╲
     │ ╲
 -3 ─┤  ●────── Cutoff (2.6 Hz)
     │   ╲╲
     │    ╲╲
-20 ─┤     ╲╲___
     │       ╲╲╲_____
-40 ─┤          ╲╲╲╲╲______
     └──────────────────────────► Frequency (Hz)
     0  2.6  5   10   20  50

At 2.6 Hz: -3 dB (50% amplitude)
At 26 Hz: -20 dB (10% amplitude)
At 100 Hz: -40 dB (1% amplitude)

Aircraft motion: 0-5 Hz → passes through ✓
Vibration: 10-50 Hz → strongly attenuated ✓
```

### Gyroscope Filtering

```
Gyroscope is cleaner than accelerometer:
  B_GYRO = 0.10 (less aggressive filtering)
  
Cutoff frequency:
  f_c = 100 × 0.10 / (2π × 0.90)
      = 10 / 5.65
      = 1.8 Hz

Why lower cutoff?
- Gyro already quite clean
- Want to preserve fast rotation information
- Used for PID derivative term (needs bandwidth)

Trade-off:
- Lower B: More filtering, slower response
- Higher B: Less filtering, faster response but noisier
```

### Filter Tuning

```
Choose B based on:

1. SAMPLE RATE
   f_c = f_sample × B / (2π(1-B))
   
   Higher sample rate → can use smaller B for same f_c

2. NOISE LEVEL
   More noise → use smaller B (more filtering)
   
3. SIGNAL BANDWIDTH
   Faster aircraft motion → need higher f_c → larger B

4. COMPUTATIONAL COST
   IIR filter: Always cheap (fixed cost)
   Can afford low B values

Typical ranges:
  B = 0.05-0.15 for accelerometers
  B = 0.08-0.12 for gyroscopes

Our values:
  B_ACCEL = 0.14 (moderate filtering)
  B_GYRO = 0.10 (light filtering)
```

---

## Sensor Fusion (Not Implemented)

### Complementary Filter

```
Idea: Combine accelerometer and gyroscope

Accelerometer: Good low-frequency, bad high-frequency
Gyroscope: Good high-frequency, bad low-frequency (drift)

Complementary filter:
  angle = α × (angle + gyro × dt) + (1 - α) × accel_angle
          └──── Integrate gyro ─────┘  └─ Tilt from accel ──┘
  
Where:
- α ≈ 0.98 (high-pass filter on gyro, low-pass on accel)
- dt: Time step
- gyro: Angular velocity (°/s)
- accel_angle: Angle from accelerometer

Block diagram:
                ┌────────────┐
  gyro ─────────┤ Integrate  ├───┐
   (°/s)        │   × dt     │   │    ┌─────┐
                └────────────┘   ├────┤ α   ├───┐
                                 │    └─────┘   │
                                 │              (+)─── angle
                ┌────────────┐   │    ┌─────┐   │      (°)
  accel ────────┤atan2(...)  ├───┴────┤1 - α├───┘
  (g)           │ angles     │        └─────┘
                └────────────┘

Advantages:
✓ Corrects gyro drift (using accel)
✓ Reduces accel noise (using gyro)
✓ Works during acceleration (gyro dominates short-term)
✓ Simple to implement

Why we don't use it:
- Accel-only is sufficient for bench testing
- No sustained acceleration (aircraft on ground)
- Simpler code
- One less tuning parameter
```

### Kalman Filter

```
Optimal sensor fusion (statistically)

State space model:
  x = [angle, angular_velocity]ᵀ
  
Process model:
  angle(n) = angle(n-1) + gyro(n) × dt
  gyro(n) = gyro(n-1) + w_gyro
  
Measurement model:
  accel_angle(n) = angle(n) + w_accel
  gyro_measured(n) = gyro(n) + w_gyro_sensor

Kalman filter algorithm:
  1. Predict: Use gyro to predict next angle
  2. Update: Correct prediction using accel measurement
  3. Compute Kalman gain (optimal weighting)
  
Advantages:
✓ Optimal (minimum variance)
✓ Handles sensor uncertainties
✓ Can model complex dynamics

Disadvantages:
❌ Complex to implement
❌ Needs tuning (process/measurement noise)
❌ Computational cost (matrix operations)
❌ Overkill for simple attitude estimation

When to use:
- Inertial navigation (position tracking)
- High-performance flight control
- Research/advanced applications
```

### Madgwick / Mahony Filters

```
Quaternion-based attitude estimation

Advantages:
✓ No gimbal lock
✓ Efficient (quaternion math)
✓ 3D orientation (including yaw with magnetometer)
✓ Used in commercial drones

Disadvantages:
❌ Complex (quaternion algebra)
❌ Needs magnetometer for yaw
❌ More tuning parameters

Not used because:
- We only need pitch and roll (2D)
- No magnetometer in our system
- Simple atan2 is sufficient
```

---

## Calibration

### Sensor Biases

```
All sensors have BIAS (constant offset):

Accelerometer bias:
  a_measured = a_true + a_bias
  
Typical values:
  a_bias = ±0.05g (varies with temperature, aging)

Gyroscope bias (drift):
  ω_measured = ω_true + ω_bias
  
Typical values:
  ω_bias = ±2 °/s (much larger!)
  Temperature dependent!
  Changes over time (minutes)

Why calibrate?
- Angle error = bias
- For accel: 0.05g → ~3° error!
- For gyro: 2°/s → 2° error after 1 second of integration
```

### Calibration Process

```cpp
void calibrateMPU6050() {
    const int numSamples = 200;  // 2 seconds at 100 Hz
    
    // Arrays to accumulate samples
    float pitch_sum = 0, roll_sum = 0;
    float gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
    
    // Collect samples (aircraft must be LEVEL and STATIONARY)
    for (int i = 0; i < numSamples; i++) {
        // Read sensors
        read_MPU6050();
        
        // Calculate angles
        float pitch_sample = atan2(AccelY, sqrt(...)) × 57.3;
        float roll_sample = atan2(-AccelX, sqrt(...)) × 57.3;
        
        // Accumulate
        pitch_sum += pitch_sample;
        roll_sum += roll_sample;
        gyroX_sum += GyroX;
        gyroY_sum += GyroY;
        gyroZ_sum += GyroZ;
        
        delay(10);  // 100 Hz
    }
    
    // Calculate average (these are the biases!)
    pitch_offset = pitch_sum / numSamples;
    roll_offset = roll_sum / numSamples;
    GyroX_offset = gyroX_sum / numSamples;
    GyroY_offset = gyroY_sum / numSamples;
    GyroZ_offset = gyroZ_sum / numSamples;
}
```

### Applying Calibration

```cpp
// After reading sensors and computing angles:

pitch -= pitch_offset;  // Remove accelerometer tilt bias
roll -= roll_offset;

GyroX -= GyroX_offset;  // Remove gyroscope drift
GyroY -= GyroY_offset;
GyroZ -= GyroZ_offset;

Now:
  pitch = 0° when level (by definition!)
  roll = 0° when level
  GyroX/Y/Z = 0°/s when stationary
```

### Calibration Accuracy

```
With 200 samples:

Standard error of mean = σ / √N

For accelerometer (σ ≈ 4 mg = 0.23°):
  SE = 0.23° / √200 = 0.016° ✓ (very accurate!)

For gyroscope (σ ≈ 0.05 °/s):
  SE = 0.05 / √200 = 0.0035 °/s ✓

Calibration precision: ~0.02° for angles
This is MUCH better than sensor noise!
```

### When to Recalibrate

```
Recalibrate if:
1. Temperature changes significantly (>10°C)
2. After impact/crash (sensor mounting shifted)
3. Periodic (every hour for gyro drift)
4. Sensor readings look wrong (visible offset)

Automatic recalibration:
- Can calibrate gyro during flight (when stationary hover)
- Cannot calibrate accel during flight (unknown acceleration)
```

---

## Noise Analysis

### Sources of Noise

```
1. QUANTIZATION NOISE
   ─────────────────
   ADC resolution: 16-bit
   Noise floor: 1 LSB = 0.061 mg
   
   Effect: Negligible (very small)

2. THERMAL NOISE
   ─────────────
   Electronics generate random noise
   Gaussian distribution
   
   Typical: 400 µg/√Hz (accelerometer)
   At 100 Hz BW: 4 mg RMS

3. FLICKER NOISE (1/f noise)
   ─────────────────────────
   Low-frequency noise
   Dominates at < 1 Hz
   
   Causes gyro drift over time

4. VIBRATION
   ──────────
   Mechanical vibration couples to sensor
   Typically 10-100 Hz (motor frequency)
   
   Can be 100+ mg amplitude!
   MUST filter this

5. ELECTROMAGNETIC INTERFERENCE (EMI)
   ─────────────────────────────────
   Motors, servos generate EMI
   Can couple into sensor wires
   
   Solution: Keep wires short, add ferrite beads
```

### Noise Spectrum

```
Power Spectral Density (PSD):

PSD
(mg²/Hz)
  100 ─┤                       Motor vibration
       │                       ▲▲▲▲
   10 ─┤              ▲▲▲▲    ││││
       │         ▲▲▲▲││││    ││││
    1 ─┤    ▲▲▲▲││││││││    ││││
       │ ▲▲▲││││││││││││────┴┴┴┴─── Thermal noise
  0.1 ─┤▲│││││││││││││
       │││││││││││││                Flicker noise
 0.01 ─┴┴┴┴┴┘
       └───────────────────────────► Frequency (Hz)
       0.1  1   10  50 100

Aircraft motion: 0-5 Hz
Our filter cutoff: 2.6 Hz
Vibration: 10-100 Hz → Strongly attenuated ✓
```

### Allan Variance

```
Tool for analyzing gyroscope drift:

Allan Deviation (°/s):
  
  10 ─┤   ╲                   
       │    ╲╲                 
   1 ─┤      ╲╲    ╱           Rate random walk
       │       ╲╲  ╱            (long-term drift)
  0.1 ─┤   Angle╲╲╱             
       │   random╲╲             
 0.01 ─┤   walk  ╲╲             
       │          ╲╲___         Bias instability
       │              ╲╲╲       (minimum drift)
0.001 ─┤               ╲╲╲___   
       └────────────────────────► Integration time (s)
       1   10  100 1000

Minimum drift: ~0.005 °/s at τ ≈ 10 sec
For our application: Negligible (no long integrations)
```

---

## Performance Optimization

### Computational Cost

```
Per sensor reading (100 Hz):

Operation                  Time     % of loop
──────────────────────────────────────────────
I2C read (14 bytes)        1.0 ms   10%
Scale (6 divisions)        0.01 ms  0.1%
Filter (6 multiplications) 0.02 ms  0.2%
Angle calc (2 atan2)       0.08 ms  0.8%
Calibration (6 subtract)   0.01 ms  0.1%
──────────────────────────────────────────────
TOTAL                      1.12 ms  11.2%

Bottleneck: I2C communication (89% of sensor processing time)

Optimization opportunities:
1. Increase I2C speed to 400 kHz → reduce to ~0.3ms
2. Use DMP (on-chip processing) → no atan2 needed
3. Fixed-point math → faster on non-FPU chips (not needed on ESP32)
```

### Memory Optimization

```
Variables needed per axis:

float AccelX, AccelX_prev;    // 8 bytes
float GyroX, GyroX_prev;      // 8 bytes
float pitch, roll;            // 8 bytes
float pitch_offset, roll_offset;  // 8 bytes
float GyroX/Y/Z_offset;       // 12 bytes
──────────────────────────────
Total: ~44 bytes

For 3 axes: ~100 bytes total

This is TINY (< 0.1% of ESP32 RAM)
No optimization needed!
```

### Sampling Rate Optimization

```
Current: 100 Hz (10ms)

Nyquist theorem: Sample at > 2× highest frequency of interest

Aircraft dynamics: ~5 Hz max
Minimum sampling: 10 Hz (Nyquist)
Recommended: 50-100 Hz (safety margin)

Trade-offs:

Higher rate (e.g., 200 Hz):
✓ Lower latency
✓ Better noise averaging
✗ More CPU usage
✗ More power consumption
✗ Diminishing returns

Lower rate (e.g., 50 Hz):
✓ Less CPU usage
✓ Lower power
✗ Higher latency (20ms vs 10ms)
✗ Less margin above Nyquist

100 Hz is OPTIMAL for our application ✓
```

---

## Advanced Topics

### Temperature Compensation

```
Sensor biases vary with temperature:

Typical drift:
  Accel: ±0.01g per °C
  Gyro: ±0.05°/s per °C

Temperature compensation:
```cpp
float temp = readTemperature();  // From MPU6050
float temp_delta = temp - calibration_temp;

// Compensate biases
float gyro_drift_comp = GyroX_offset + temp_coeff × temp_delta;
GyroX -= gyro_drift_comp;
```

Requires:
- Temperature sensor (MPU6050 has one built-in!)
- Temperature coefficients (characterize each sensor)
- More complex calibration
```

### Sensor Alignment

```
Problem: Sensor axes not aligned with aircraft body

Real mounting:
  Sensor X ≈ Aircraft X (but rotated ~5°)
  Sensor Y ≈ Aircraft Y (but rotated ~5°)
  
Solution: Rotation matrix
```cpp
// Rotation matrix (pre-calibrated)
float R[3][3] = {
    {cos(θ), -sin(θ), 0},
    {sin(θ),  cos(θ), 0},
    {0,       0,      1}
};

// Rotate sensor readings to body frame
AccelX_body = R[0][0]*AccelX + R[0][1]*AccelY + R[0][2]*AccelZ;
AccelY_body = R[1][0]*AccelX + R[1][1]*AccelY + R[1][2]*AccelZ;
AccelZ_body = R[2][0]*AccelX + R[2][1]*AccelY + R[2][2]*AccelZ;
```

Usually not necessary (mount sensor accurately instead!)
```

### Adaptive Filtering

```
Adjust filter coefficient based on conditions:

```cpp
// During motion: Less filtering (faster response)
if (abs(GyroX) > 10 || abs(GyroY) > 10) {
    B_ACCEL = 0.20;  // More responsive
} else {
    B_ACCEL = 0.10;  // More filtering
}
```

Or use motion detection:
```cpp
float accel_magnitude = sqrt(AccelX² + AccelY² + AccelZ²);
if (abs(accel_magnitude - 1.0) > 0.2) {
    // Under acceleration
    trust_accel = false;  // Don't update angle
} else {
    // Stationary
    trust_accel = true;   // Use accel for angle
}
```

This is stepping towards Kalman filtering!
```

---

## Summary

Our sensor processing pipeline:

```
┌─────────────────────────────────────────────────────────────────┐
│                   SENSOR PROCESSING PIPELINE                     │
└─────────────────────────────────────────────────────────────────┘

1. READ MPU6050 (I2C)
   ├─ Accelerometer: 3 axes (X, Y, Z) in g
   └─ Gyroscope: 3 axes (X, Y, Z) in °/s

2. SCALE TO PHYSICAL UNITS
   ├─ Accel: Raw / 16384.0 → g
   └─ Gyro: Raw / 131.0 → °/s

3. LOW-PASS FILTER
   ├─ Accel: B = 0.14 (f_c = 2.6 Hz)
   └─ Gyro: B = 0.10 (f_c = 1.8 Hz)

4. CALCULATE ANGLES (Accelerometer-based)
   ├─ Pitch: atan2(AccelY, √(AccelX² + AccelZ²))
   └─ Roll: atan2(-AccelX, √(AccelY² + AccelZ²))

5. APPLY CALIBRATION
   ├─ Pitch -= pitch_offset
   ├─ Roll -= roll_offset
   └─ GyroX/Y/Z -= gyro_offset

6. CONSTRAIN
   ├─ Pitch/Roll: -45° to +45°
   └─ (Prevent unrealistic values)

7. USE IN CONTROL
   ├─ PID uses: pitch, roll (for error)
   └─ PID uses: GyroX, GyroY (for derivative)

Total latency: ~1.1 ms (excellent!)
Update rate: 100 Hz (smooth!)
```

**Key takeaways:**
- ✅ Accelerometer gives absolute angle (but noisy, fails during acceleration)
- ✅ Gyroscope gives rate (clean, works during acceleration, but drifts)
- ✅ Low-pass filter removes vibration noise
- ✅ Calibration removes sensor biases
- ✅ Our implementation: Accel-only (sufficient for ground testing)
- ⚠️ Future: Add complementary or Kalman filter for flight

---

**END OF SENSOR FUSION AND FILTERING DOCUMENTATION**

