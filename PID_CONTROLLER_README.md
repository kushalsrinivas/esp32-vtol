# PID Controller Implementation for Servo Control

## Overview

This implementation adds a **PID (Proportional-Integral-Derivative) controller** to the MPU6050 servo control system, based on the dRehmFlight controller architecture. The PID controller provides smooth, stable, and accurate servo positioning based on IMU tilt measurements.

## What Changed from Simple Mapping

### Before (Direct Mapping)
```cpp
// Simple direct mapping - no feedback control
int servoAngle = map(AccelX * 1000, -1000, 1000, 0, 180);
servo.write(servoAngle);
```

**Problems with direct mapping:**
- ❌ No damping - servos overshoot and oscillate
- ❌ Sensitive to sensor noise
- ❌ No compensation for servo lag
- ❌ Unpredictable behavior with fast movements

### After (PID Control)
```cpp
// PID control - smooth, stable, accurate
computePID_Servo1();  // Calculate optimal servo command
servo1.write(servo1_PID);
```

**Benefits of PID control:**
- ✅ Smooth motion with minimal overshoot
- ✅ Damped response (D term uses gyro rate)
- ✅ Reaches exact target position (I term eliminates error)
- ✅ Predictable, tunable behavior
- ✅ Based on proven flight controller design (dRehmFlight)

## How It Works

### PID Components

#### 1. **Proportional (P) Term**
```cpp
error = desired_position - actual_position
P_output = Kp * error
```
- **What it does:** Provides correction proportional to how far off you are
- **Effect:** Larger error = stronger correction
- **Tuning:** Higher Kp = faster response, but can cause oscillation

#### 2. **Integral (I) Term**
```cpp
integral = integral_prev + error * dt
I_output = Ki * integral
```
- **What it does:** Accumulates error over time to eliminate steady-state error
- **Effect:** Ensures servo reaches exact target position
- **Tuning:** Higher Ki = faster elimination of small errors, but can cause slow oscillations
- **Safety:** Limited by `i_limit` to prevent windup

#### 3. **Derivative (D) Term**
```cpp
derivative = GyroRate  // Uses actual gyro measurement
D_output = Kd * derivative
```
- **What it does:** Opposes rapid changes, providing damping
- **Effect:** Reduces overshoot and oscillation
- **Tuning:** Higher Kd = more damping, but can make response sluggish
- **Key feature:** Uses gyro rate directly (cleaner than differentiating error)

### Final Output
```cpp
servo_output = Kp * error + Ki * integral - Kd * gyro_rate
```

## Key Features from dRehmFlight

### 1. **Integral Anti-Windup**
```cpp
integral = constrain(integral, -i_limit, i_limit);
```
Prevents the integral term from accumulating excessively, which could cause overshoot or instability.

### 2. **Gyro-Based Derivative**
```cpp
derivative_servo1 = GyroY;  // Direct gyro measurement
```
Instead of differentiating the error (which amplifies noise), we use the actual gyro rate. This is a key technique from flight controllers that provides cleaner damping.

### 3. **Accurate Timing**
```cpp
dt = (current_time - prev_time) / 1000.0;
```
Proper time-step calculation ensures PID gains work consistently regardless of loop rate variations.

### 4. **Low-Pass Filtering**
```cpp
AccelX = (1.0 - B_ACCEL) * AccelX_prev + B_ACCEL * AccelX;
GyroX = (1.0 - B_GYRO) * GyroX_prev + B_GYRO * GyroX;
```
Sensor data is filtered before entering the PID controller, reducing noise impact.

## Tuning Guide

### Step-by-Step Tuning Process

#### **Phase 1: Proportional Only**
1. Set `Ki = 0` and `Kd = 0`
2. Start with `Kp = 20`
3. Tilt the MPU6050 and observe servo response:
   - **Too low:** Servo moves slowly, doesn't reach target
   - **Too high:** Servo oscillates back and forth
   - **Just right:** Servo moves quickly to target with small overshoot
4. Adjust Kp until you get good responsiveness
5. **Recommended range:** 30-60

#### **Phase 2: Add Derivative**
1. Keep Kp from Phase 1
2. Start with `Kd = 5`
3. Observe the response:
   - **Effect:** Should reduce/eliminate oscillations
   - **Too low:** Still oscillates
   - **Too high:** Servo moves very slowly, feels "sticky"
4. Increase Kd until oscillations are damped
5. **Recommended range:** 5-15

#### **Phase 3: Add Integral**
1. Keep Kp and Kd from previous phases
2. Start with `Ki = 2`
3. Tilt to a position and hold steady:
   - **Effect:** Servo should creep to exact target position
   - **Too low:** Small steady-state error remains
   - **Too high:** Slow oscillations or overshoots appear
4. Adjust Ki for precise positioning without oscillation
5. **Recommended range:** 1-10

#### **Phase 4: Fine Tuning**
- Adjust all three gains together for optimal performance
- Test with different tilt speeds (slow, medium, fast)
- Test with different tilt angles (small, large)
- Watch Serial Plotter to see desired vs actual positions

### Current Settings

```cpp
// Default values (good starting point)
float Kp_servo1 = 45.0;   // Proportional gain
float Ki_servo1 = 5.0;    // Integral gain
float Kd_servo1 = 8.0;    // Derivative gain
float i_limit = 25.0;     // Integral saturation limit
```

## Monitoring with Serial Plotter

Open **Tools → Serial Plotter** to visualize:

| Signal | Description |
|--------|-------------|
| `TiltX`, `TiltY` | Current tilt angles from accelerometer (degrees) |
| `Servo1_Des`, `Servo2_Des` | Desired servo positions (setpoints) |
| `Servo1_Actual`, `Servo2_Actual` | Actual servo positions (PID output) |
| `GyroX`, `GyroY` | Rotation rates used for derivative term (deg/s) |

### What to Look For

**Good tuning:**
- Actual position follows desired position closely
- Minimal overshoot when tilting
- Smooth, damped motion
- Reaches exact target position

**Bad tuning (oscillation):**
- Actual position bounces around desired
- **Fix:** Reduce Kp or increase Kd

**Bad tuning (sluggish):**
- Actual position lags far behind desired
- **Fix:** Increase Kp or reduce Kd

**Bad tuning (steady-state error):**
- Actual position gets close but not exact
- **Fix:** Increase Ki

## Code Structure

### Global Variables
```cpp
// PID gains (tunable parameters)
float Kp_servo1, Ki_servo1, Kd_servo1;

// PID state variables
float error_servo1, integral_servo1, derivative_servo1;
float servo1_PID;  // Final output

// Timing
float dt;  // Time since last loop
```

### Main Loop Flow
```cpp
1. Calculate dt (time step)
2. Read and filter IMU data
3. Calculate tilt angles from accelerometer
4. Map tilt to desired servo position (setpoint)
5. Run PID controller → computePID_Servo1()
6. Apply PID output to servo
7. Send data to Serial Plotter
```

### PID Function
```cpp
void computePID_Servo1() {
    // 1. Calculate error
    error = desired - actual;
    
    // 2. Integrate error
    integral = integral_prev + error * dt;
    integral = constrain(integral, -i_limit, i_limit);
    
    // 3. Get derivative (gyro rate)
    derivative = GyroY;
    
    // 4. Compute PID output
    output = Kp * error + Ki * integral - Kd * derivative;
    
    // 5. Constrain and save
    output = constrain(output, 0, 180);
    integral_prev = integral;
}
```

## Comparison to dRehmFlight

### Similarities
✅ PID structure with P, I, D terms  
✅ Integral anti-windup with saturation limits  
✅ Gyro-based derivative for damping  
✅ Low-pass filtering on sensor inputs  
✅ Accurate dt calculation  
✅ Constrained outputs  

### Differences
- **dRehmFlight:** Controls motor speeds for flight stabilization
- **This code:** Controls servo positions for tilt tracking
- **dRehmFlight:** Uses Madgwick filter for attitude estimation
- **This code:** Uses simple accelerometer tilt calculation (adequate for servo control)
- **dRehmFlight:** Cascaded control (outer angle loop, inner rate loop)
- **This code:** Single-level control (direct position control)

## Troubleshooting

### Issue: Servos jitter or oscillate
**Cause:** Kp too high or Kd too low  
**Fix:** Reduce Kp by 20-30% or increase Kd by 50%

### Issue: Servos move too slowly
**Cause:** Kp too low or Kd too high  
**Fix:** Increase Kp by 20-30% or reduce Kd by 30%

### Issue: Servos don't reach exact position
**Cause:** Ki too low or integral windup  
**Fix:** Increase Ki slightly, check `i_limit` setting

### Issue: Servos overshoot and take long to settle
**Cause:** Kd too low or Ki too high  
**Fix:** Increase Kd for more damping, reduce Ki

### Issue: Erratic behavior with fast movements
**Cause:** Sensor noise, filter settings  
**Fix:** Increase `B_ACCEL` and `B_GYRO` for more filtering (slower response)

### Issue: Different behavior on different axes
**Solution:** Tune Servo1 and Servo2 PID gains independently

## Advanced Modifications

### 1. Adjust Sensitivity
```cpp
float max_tilt_angle = 45.0;  // Increase for less sensitive
                               // Decrease for more sensitive
```

### 2. Different Gains per Axis
```cpp
// If X and Y axes behave differently
Kp_servo1 = 45.0;  // X-axis
Kp_servo2 = 50.0;  // Y-axis (different value)
```

### 3. Rate Limiting
Add maximum servo speed limits:
```cpp
float max_servo_rate = 180.0;  // deg/s
float servo_rate = (servo1_PID - servo1_prev) / dt;
if (abs(servo_rate) > max_servo_rate) {
    // Limit rate
}
```

### 4. Deadband
Ignore small errors to reduce jitter:
```cpp
if (abs(error_servo1) < 2.0) {
    error_servo1 = 0;
}
```

## References

- **dRehmFlight:** Flight controller by Nicholas Rehm
  - GitHub: [nickrehm/dRehmFlight](https://github.com/nickrehm/dRehmFlight)
  - Proven PID structure for stabilization
  - Key techniques: integral anti-windup, gyro-based derivative

- **PID Theory:**
  - Wikipedia: [PID controller](https://en.wikipedia.org/wiki/PID_controller)
  - Ziegler-Nichols tuning method

## License

This implementation is based on concepts from dRehmFlight (open source) and adapted for servo control applications.

---

**Created for:** MPU6050 Diagnostic Tool with Servo Control  
**Based on:** dRehmFlight by Nicholas Rehm  
**Date:** 2025  

