# PID Tuning Quick Reference

## Current Settings (lines 48-55 in .ino file)

```cpp
float Kp_servo1 = 45.0;    // Proportional gain for servo1 (X-axis)
float Ki_servo1 = 5.0;     // Integral gain for servo1
float Kd_servo1 = 8.0;     // Derivative gain for servo1

float Kp_servo2 = 45.0;    // Proportional gain for servo2 (Y-axis)
float Ki_servo2 = 5.0;     // Integral gain for servo2
float Kd_servo2 = 8.0;     // Derivative gain for servo2

float i_limit = 25.0;      // Integrator saturation limit
float max_tilt_angle = 45.0; // Maximum tilt angle for mapping
```

## Quick Tuning Guide

### Problem → Solution Matrix

| Symptom | Likely Cause | Action |
|---------|--------------|--------|
| 🔴 Servo oscillates/bounces | Kp too high | ⬇️ Reduce Kp by 20-30% |
| 🔴 Servo oscillates/bounces | Kd too low | ⬆️ Increase Kd by 50% |
| 🟡 Servo moves too slowly | Kp too low | ⬆️ Increase Kp by 20-30% |
| 🟡 Servo feels "sticky" | Kd too high | ⬇️ Reduce Kd by 30% |
| 🟡 Doesn't reach target exactly | Ki too low | ⬆️ Increase Ki by 1-2 |
| 🔴 Overshoots then slow oscillation | Ki too high | ⬇️ Reduce Ki by 1-2 |
| 🟢 Too sensitive to tilts | max_tilt_angle too low | ⬆️ Increase to 60-90° |
| 🟢 Not sensitive enough | max_tilt_angle too high | ⬇️ Reduce to 30-45° |

## 3-Step Tuning Process

### Step 1: Tune P (Responsiveness)
```
Set: Ki = 0, Kd = 0
Start: Kp = 30
```
1. Tilt MPU6050 to one side
2. Watch how fast servo responds
3. If too slow → increase Kp
4. If oscillates → decrease Kp
5. **Goal:** Fast response with small overshoot
6. **Typical range:** 30-60

### Step 2: Add D (Damping)
```
Keep: Kp from Step 1
Start: Kd = 5
```
1. Tilt and watch for oscillations
2. Increase Kd until oscillations stop
3. Don't increase too much or servo will feel sluggish
4. **Goal:** Smooth motion, no oscillation
5. **Typical range:** 5-15

### Step 3: Add I (Precision)
```
Keep: Kp and Kd from previous steps
Start: Ki = 2
```
1. Tilt and hold at an angle
2. Watch if servo reaches exact target
3. Increase Ki if there's a small offset
4. Watch for slow oscillations (sign of too much Ki)
5. **Goal:** Exact positioning without oscillation
6. **Typical range:** 1-10

## Test Scenarios

### Test 1: Slow Tilt
- Tilt slowly from center to ~30°
- **Good:** Smooth tracking, reaches target
- **Bad:** Jerky motion, oscillates at target

### Test 2: Fast Tilt
- Quickly tilt to ~30° and hold
- **Good:** Fast response, small overshoot, settles quickly
- **Bad:** Large overshoot, oscillates, takes long to settle

### Test 3: Return to Center
- Tilt and return to flat
- **Good:** Returns to exactly 90° (center)
- **Bad:** Stops at 88° or 92° (steady-state error)

### Test 4: Extreme Tilt
- Tilt to maximum angle (~45°)
- **Good:** Servo reaches end position smoothly
- **Bad:** Servo maxes out and jitters

## Serial Plotter Interpretation

### What to Watch
```
Open Tools → Serial Plotter
Watch these signals:
```

| Signal | What It Means |
|--------|---------------|
| `TiltX`, `TiltY` | Actual tilt angle from sensor |
| `Servo1_Des`, `Servo2_Des` | Where you WANT the servo to be |
| `Servo1_Actual`, `Servo2_Actual` | Where the servo ACTUALLY is |
| `GyroX`, `GyroY` | Rate of rotation (damping feedback) |

### Good Tuning Example
```
Servo_Des:    ___/‾‾‾‾‾\___
Servo_Actual: __/‾‾‾‾‾‾\__

✅ Actual follows Desired closely
✅ Minimal overshoot
✅ Smooth curves
```

### Bad Tuning Example (Oscillation)
```
Servo_Des:    ___/‾‾‾‾‾\___
Servo_Actual: _/\/\/\/\/\_

❌ Actual bounces around Desired
❌ Action: Reduce Kp or increase Kd
```

### Bad Tuning Example (Lag)
```
Servo_Des:    ___/‾‾‾‾‾\___
Servo_Actual: ____/‾‾‾‾\___

❌ Actual lags behind Desired
❌ Action: Increase Kp or reduce Kd
```

## Preset Configurations

### Conservative (Smooth, slow)
```cpp
Kp = 30.0, Ki = 3.0, Kd = 10.0
```
- Use for: Delicate applications, slow movements
- Behavior: Very smooth, minimal overshoot, slower response

### Balanced (Default)
```cpp
Kp = 45.0, Ki = 5.0, Kd = 8.0
```
- Use for: General purpose, moderate speed
- Behavior: Good balance of speed and smoothness

### Aggressive (Fast, snappy)
```cpp
Kp = 60.0, Ki = 7.0, Kd = 5.0
```
- Use for: Fast response applications
- Behavior: Quick response, may have small overshoot

### Precise (Exact positioning)
```cpp
Kp = 40.0, Ki = 10.0, Kd = 12.0
```
- Use for: When exact position is critical
- Behavior: Slower but reaches exact target

## Sensitivity Adjustment

Control how much tilt affects servo position:

```cpp
// Less sensitive (larger tilts needed)
float max_tilt_angle = 60.0;  // Need to tilt more

// More sensitive (smaller tilts needed)
float max_tilt_angle = 30.0;  // Small tilts move servo a lot

// Default
float max_tilt_angle = 45.0;  // Balanced
```

## When Axes Behave Differently

If X and Y axes need different tuning:

```cpp
// Servo 1 (X-axis) - maybe more responsive
Kp_servo1 = 50.0;
Ki_servo1 = 5.0;
Kd_servo1 = 7.0;

// Servo 2 (Y-axis) - maybe needs more damping
Kp_servo2 = 40.0;
Ki_servo2 = 5.0;
Kd_servo2 = 10.0;
```

## Emergency Reset

If servos go crazy and won't stop oscillating:

```cpp
// Ultra-conservative values
Kp_servo1 = 20.0;
Ki_servo1 = 0.0;   // Disable I
Kd_servo1 = 15.0;  // Max damping

Kp_servo2 = 20.0;
Ki_servo2 = 0.0;
Kd_servo2 = 15.0;
```

Then gradually increase Kp and add Ki back in.

## Advanced: Understanding the Math

### P Term (Error)
```
P = Kp × (desired - actual)
```
- Large error → large correction
- Small error → small correction

### I Term (Accumulated Error)
```
I = Ki × Σ(error × time)
```
- Builds up over time
- Eliminates steady-state error
- Can cause windup (prevented by i_limit)

### D Term (Rate of Change)
```
D = Kd × gyro_rate
```
- Opposes rapid changes
- Provides "viscous damping"
- Uses gyro directly (cleaner than error derivative)

### Final Output
```
servo_output = P + I - D
             = Kp×error + Ki×integral - Kd×gyro_rate
```

## Checklist Before Flying/Using

- [ ] Servos center at 90° when MPU6050 is flat
- [ ] Tilting left/right moves servo1 smoothly
- [ ] Tilting forward/back moves servo2 smoothly
- [ ] No oscillation when held at an angle
- [ ] Returns to center (90°) when released
- [ ] Responds to fast tilts without excessive overshoot
- [ ] Serial Plotter shows Actual following Desired

## Need Help?

1. **Start with defaults:** Kp=45, Ki=5, Kd=8
2. **If unstable:** Reduce Kp by 50%, increase Kd by 50%
3. **Tune one axis at a time**
4. **Make small changes:** ±10-20% at a time
5. **Test after each change**
6. **Use Serial Plotter:** Visual feedback is key

---

**Remember:** There's no "perfect" tuning - it depends on your servos, power supply, mechanical load, and desired behavior. The values above are starting points. Experiment!

