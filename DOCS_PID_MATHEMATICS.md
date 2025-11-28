# PID Controller - Deep Mathematical Analysis

## Table of Contents

1. [Mathematical Foundation](#mathematical-foundation)
2. [Proportional Term (P)](#proportional-term-p)
3. [Integral Term (I)](#integral-term-i)
4. [Derivative Term (D)](#derivative-term-d)
5. [Complete PID Equation](#complete-pid-equation)
6. [Discrete Implementation](#discrete-implementation)
7. [Anti-Windup Protection](#anti-windup-protection)
8. [Gyro-Based Derivative](#gyro-based-derivative)
9. [Tuning Mathematics](#tuning-mathematics)

---

## Mathematical Foundation

### Control System Block Diagram

```
                           ┌─────────────────┐
    Setpoint (r) ────(+)───┤                 │
                      │-   │  PID Controller │──► Control Signal (u)
                      └────┤                 │
                           └─────────────────┘
                                    ▲
                                    │
                            Sensor Feedback (y)
```

### Error Signal

The fundamental quantity in PID control is the **error signal**:

```
e(t) = r(t) - y(t)
```

Where:

- `e(t)` = error at time t
- `r(t)` = desired setpoint (reference)
- `y(t)` = measured process variable (feedback)

**In our system:**

- `r(t)` = `pitch_des` or `roll_des` (desired angle, typically 0° for stabilization)
- `y(t)` = `pitch` or `roll` (measured angle from MPU6050)
- `e(t)` = `error_pitch` or `error_roll`

---

## Proportional Term (P)

### Mathematical Definition

```
P(t) = Kp × e(t)
```

Where:

- `Kp` = Proportional gain (unitless)
- `e(t)` = Current error (degrees)

### Physical Interpretation

The proportional term creates an output **directly proportional** to the current error.

**Example:** If aircraft is tilted 10° and `Kp = 35.0`:

```
P = 35.0 × 10° = 350° of correction
```

(This gets constrained to ±60° in practice)

### Frequency Response

The P term responds **instantaneously** to error:

- No phase lag
- Bandwidth: ∞ (theoretically)
- Gain: Constant across all frequencies

### Advantages

- ✅ Simple and intuitive
- ✅ Fast response
- ✅ Proportional to disturbance magnitude

### Disadvantages

- ❌ **Steady-state error**: Cannot drive error exactly to zero
- ❌ **Limited by stability**: High Kp causes oscillation
- ❌ **Sensitive to noise**: Amplifies measurement noise

### Stability Analysis

For a system with gain Kp, the characteristic equation becomes:

```
1 + Kp × G(s) × H(s) = 0
```

Where G(s) is the plant transfer function and H(s) is feedback.

**Critical gain** (Kp_crit): The value where system becomes unstable (begins oscillating)

**Rule of thumb:** Set `Kp ≈ 0.5 × Kp_crit` for good stability margin

---

## Integral Term (I)

### Mathematical Definition (Continuous)

```
I(t) = Ki × ∫₀ᵗ e(τ) dτ
```

Where:

- `Ki` = Integral gain (1/seconds)
- `τ` = Integration variable
- The integral accumulates error over time

### Discrete Implementation

```cpp
integral(n) = integral(n-1) + e(n) × Δt

I(n) = Ki × integral(n)
```

Where:

- `Δt` = Time step (seconds)
- `n` = Current sample number
- `n-1` = Previous sample

### Physical Interpretation

The integral term **accumulates** past errors and grows until error reaches zero.

**Example timeline:**

```
t=0:   e=5°,  integral=0°·s      → I = 0
t=0.1: e=5°,  integral=0.5°·s    → I = Ki × 0.5
t=0.2: e=5°,  integral=1.0°·s    → I = Ki × 1.0
t=0.3: e=5°,  integral=1.5°·s    → I = Ki × 1.5
...
t=5.0: e=0°,  integral=25°·s     → I = Ki × 25
t=5.1: e=-1°, integral=24.9°·s   → I = Ki × 24.9 (starts decreasing)
```

### Why Integral Eliminates Steady-State Error

Suppose system settles with small error `e_ss`:

```
If e_ss ≠ 0 → integral keeps growing → I(t) increases → correction increases
→ error reduces → eventually e_ss = 0
```

The integral term is the **only** term that can drive steady-state error to exactly zero.

### Advantages

- ✅ **Eliminates steady-state error**
- ✅ Compensates for constant disturbances
- ✅ Accounts for modeling errors

### Disadvantages

- ❌ **Integral windup**: Can accumulate excessively during saturation
- ❌ **Slow response**: Takes time to accumulate
- ❌ **Can cause overshoot**: Integral keeps pushing past zero
- ❌ **Reduces stability**: Adds phase lag

### Frequency Response

```
H_I(s) = Ki/s
```

- **Magnitude**: Increases as frequency decreases (1/f)
- **Phase**: -90° phase lag at all frequencies
- **Effect**: Very strong at low frequencies (DC), weak at high frequencies

This is why I term eliminates **steady-state** (DC) error but doesn't help with fast transients.

---

## Derivative Term (D)

### Mathematical Definition (Continuous)

```
D(t) = Kd × de(t)/dt
```

Where:

- `Kd` = Derivative gain (seconds)
- `de/dt` = Rate of change of error

### Discrete Implementation (Traditional)

```cpp
derivative(n) = (e(n) - e(n-1)) / Δt

D(n) = Kd × derivative(n)
```

### Problems with Traditional Derivative

**Noise amplification:**

```
High frequency noise → Large de/dt → Huge D term
```

**Example:** Measurement noise of ±0.1° at 100Hz:

```
Worst case: e jumps from +0.1° to -0.1° in 10ms
de/dt = 0.2° / 0.01s = 20°/s

With Kd = 8.0:
D = 8.0 × 20 = 160° of correction!
```

This is why traditional derivative is problematic.

---

## Gyro-Based Derivative

### The Key Insight

For angle control:

```
de/dt = d(r - y)/dt = dr/dt - dy/dt
```

If setpoint is constant: `dr/dt = 0`

Therefore:

```
de/dt = -dy/dt = -ω
```

Where `ω` = angular velocity (measured by gyroscope!)

### Implementation

```cpp
derivative_pitch = GyroY;  // Pitch rate (°/s)
derivative_roll = GyroX;   // Roll rate (°/s)

// In PID equation:
D = -Kd × derivative  // Note the negative sign!
```

### Why This is Better

1. **Direct measurement**: Gyro directly measures rate (no differentiation needed)
2. **Clean signal**: Gyro is pre-filtered by sensor
3. **No noise amplification**: No numerical derivative
4. **Physical meaning**: Damping opposes motion
5. **Standard practice**: Used in all flight controllers

### Physical Interpretation

The D term opposes **velocity**, providing damping:

```
If aircraft is rotating quickly → Large GyroY → Large D term
→ Strong correction opposing the motion → Slows down rotation
→ Reduces overshoot and oscillation
```

### Advantages

- ✅ **Reduces overshoot**
- ✅ **Damps oscillations**
- ✅ **Improves stability**
- ✅ **Clean signal** (when using gyro)

### Disadvantages

- ❌ **Noise sensitivity** (if differentiating error)
- ❌ **Can slow response** (too much damping)
- ❌ **No effect on steady-state**

### Frequency Response

```
H_D(s) = Kd × s
```

- **Magnitude**: Increases with frequency (f)
- **Phase**: +90° phase lead
- **Effect**: Strong at high frequencies, zero at DC

This is why D term helps with fast transients and oscillations but doesn't affect steady-state.

---

## Complete PID Equation

### Standard Form (Continuous)

```
u(t) = Kp × e(t) + Ki × ∫₀ᵗ e(τ)dτ + Kd × de(t)/dt
```

### Our Implementation (Discrete, Gyro-Based)

```cpp
// Pitch PID:
error_pitch = pitch_des - pitch;                              // Error (°)
integral_pitch = integral_pitch_prev + error_pitch × dt;      // Accumulate (°·s)
integral_pitch = constrain(integral_pitch, -i_limit, i_limit); // Anti-windup
derivative_pitch = GyroY;                                     // Rate (°/s)

pitch_PID = Kp_pitch × error_pitch
          + Ki_pitch × integral_pitch
          - Kd_pitch × derivative_pitch;

pitch_PID = constrain(pitch_PID, -ELEVON_RANGE, ELEVON_RANGE); // ±60°
```

### Term-by-Term Analysis

**Current values:** Kp=35.0, Ki=3.0, Kd=8.0

**Example scenario:** Aircraft tilted 10° nose up, rotating at 5°/s

```
P term: 35.0 × 10°     = 350°     → Constrained to 60°
I term: 3.0 × 0°·s     = 0°       → (assuming no accumulated error)
D term: -8.0 × 5°/s    = -40°     → Opposes rotation

Total (before constraint): 60° + 0° - 40° = 20°
Final output: 20° of servo deflection
```

### Time Evolution

```
t=0.0s:  e=10°,  I=0,      ω=0°/s   → PID=60° (max P)
t=0.1s:  e=8°,   I=0.24°·s, ω=20°/s → PID=60-160=-100° → -60° (damping limits)
t=0.5s:  e=2°,   I=3°·s,   ω=5°/s  → PID=70+9-40=39°
t=1.0s:  e=0.5°, I=8°·s,   ω=1°/s  → PID=17.5+24-8=33.5°
t=2.0s:  e=0°,   I=15°·s,  ω=0°/s  → PID=0+45+0=45°
t=2.5s:  e=-1°,  I=14.5°·s, ω=-2°/s → PID=-35+43.5+16=24.5°
t=3.0s:  e=0°,   I=14.5°·s, ω=0°/s  → PID=0+43.5+0=43.5° (settling...)
```

Eventually integral winds down to zero and system settles perfectly at `e=0°`.

---

## Discrete Implementation

### Sample Rate Considerations

**Our system:**

- Main loop: 100 Hz (10ms interval)
- MPU6050 update: 100 Hz
- PID computation: 100 Hz
- Servo update: 100 Hz
- Telemetry: 50 Hz (20ms interval)

### Timing Accuracy

```cpp
current_time = millis();
dt = (current_time - prev_time) / 1000.0;  // Convert ms to seconds
prev_time = current_time;
```

**Critical:** Accurate `dt` ensures PID gains work consistently.

If loop rate varies:

```
dt = 10ms → integral grows normally
dt = 20ms → integral grows 2x faster (same behavior with scaled time)
dt = 5ms  → integral grows slower
```

This is why we calculate actual `dt` each loop instead of assuming 0.01s.

### Discretization Method

**Rectangular integration** (simplest):

```cpp
integral(n) = integral(n-1) + e(n) × dt
```

**Trapezoidal integration** (more accurate):

```cpp
integral(n) = integral(n-1) + (e(n) + e(n-1))/2 × dt
```

We use rectangular because:

- Simpler
- dt is small (10ms) so error is negligible
- Trapezoidal only provides ~1% improvement

### Z-Transform

The discrete transfer function becomes:

```
       b₀ + b₁z⁻¹ + b₂z⁻²
H(z) = ───────────────────
       1 - z⁻¹
```

Where:

```
b₀ = Kp + Ki×dt + Kd/dt
b₁ = -Kp - 2×Kd/dt
b₂ = Kd/dt
```

This can be used for stability analysis in the Z-domain.

---

## Anti-Windup Protection

### The Windup Problem

**Scenario:** Servo reaches physical limit (150°) but error still exists (aircraft still tilted)

```
Error exists → Integral keeps growing → I term becomes huge
→ Even when error reverses, large I term keeps pushing same direction
→ Large overshoot → Instability
```

### Our Implementation

```cpp
integral_pitch = constrain(integral_pitch, -i_limit, i_limit);
```

With `i_limit = 15.0°·s`:

**Maximum integral contribution:**

```
I_max = Ki × i_limit = 3.0 × 15.0 = 45°
```

This prevents integral from dominating the control signal.

### Choosing i_limit

**Rule of thumb:**

```
i_limit ≈ ELEVON_RANGE / Ki
i_limit ≈ 60° / 3.0 = 20°·s
```

We use 15°·s to be more conservative.

### Alternative Anti-Windup Methods

**Conditional integration** (stop integrating when saturated):

```cpp
if (output < OUTPUT_MAX && output > OUTPUT_MIN) {
    integral += error × dt;
}
```

**Back-calculation** (reduce integral when saturated):

```cpp
if (output_saturated) {
    integral -= (output_actual - output_desired) / Tt;
}
```

We use simple clamping because it's robust and easy to tune.

---

## Gyro-Based Derivative

### Comparison Table

| Method                  | Formula                 | Noise     | Accuracy | Lag     |
| ----------------------- | ----------------------- | --------- | -------- | ------- |
| **Error derivative**    | `(e(n)-e(n-1))/dt`      | Very high | Poor     | None    |
| **Filtered derivative** | `LPF((e(n)-e(n-1))/dt)` | Medium    | Medium   | Yes     |
| **Gyroscope**           | `-GyroY`                | Low       | High     | Minimal |

### Mathematical Equivalence

For angle `θ` and angular velocity `ω`:

```
dθ/dt = ω
```

For error `e = θ_des - θ` with constant setpoint:

```
de/dt = d(θ_des - θ)/dt = 0 - dθ/dt = -ω
```

Therefore:

```
Kd × de/dt = Kd × (-ω) = -Kd × ω
```

### Gyro Filtering

Raw gyro has noise. We apply low-pass filter:

```cpp
GyroY = (1.0 - B_GYRO) × GyroY_prev + B_GYRO × GyroY_raw;
```

With `B_GYRO = 0.10`:

**Transfer function:**

```
H(z) = B / (1 - (1-B)z⁻¹)
```

**Cutoff frequency:**

```
f_c = f_sample × B / (2π(1-B))
f_c = 100 Hz × 0.10 / (2π × 0.90)
f_c ≈ 1.77 Hz
```

This removes high-frequency noise while preserving motion information.

---

## Tuning Mathematics

### Ziegler-Nichols Method

**Step 1:** Find ultimate gain (Ku) and period (Tu)

- Set Ki=0, Kd=0
- Increase Kp until system oscillates continuously
- Ku = that critical Kp value
- Tu = oscillation period

**Step 2:** Calculate gains

```
Kp = 0.6 × Ku
Ki = 2 × Kp / Tu
Kd = Kp × Tu / 8
```

### Root Locus Method

Plot closed-loop poles as gains vary:

- Poles in left half-plane: Stable
- Poles on imaginary axis: Marginally stable (oscillation)
- Poles in right half-plane: Unstable

### Gain Margin and Phase Margin

**Gain margin:** How much can gain increase before instability?

```
GM = 20 log₁₀(1/|G(jω_pc)|) dB
```

**Phase margin:** How much phase lag before instability?

```
PM = 180° + ∠G(jω_gc)
```

**Safe values:**

- GM > 6 dB
- PM > 30°

### Empirical Tuning (What We Use)

**Phase 1: Proportional** (Kp)

```
Start: Kp = 30
Test: Tilt aircraft, observe response
If sluggish: Kp += 10
If oscillates: Kp -= 10
Goal: Fast response, slight overshoot
Typical: 30-50
```

**Phase 2: Derivative** (Kd)

```
Start: Kd = 5
Test: Observe overshoot and oscillation
If oscillates: Kd += 2
If too slow: Kd -= 2
Goal: Smooth, damped motion
Typical: 5-12
```

**Phase 3: Integral** (Ki)

```
Start: Ki = 2
Test: Hold at angle, check if reaches exact target
If steady error: Ki += 1
If slow oscillation: Ki -= 1
Goal: Zero steady-state error, no oscillation
Typical: 2-5
```

### Current Values Analysis

```cpp
Kp_pitch = 35.0;  // Moderate-strong response
Ki_pitch = 3.0;   // Light integral action
Kd_pitch = 8.0;   // Moderate damping
```

**Closed-loop bandwidth estimate:**

```
ω_n ≈ sqrt(Kp) ≈ sqrt(35) ≈ 5.9 rad/s ≈ 0.94 Hz
```

**Damping ratio estimate:**

```
ζ ≈ Kd / (2√Kp) ≈ 8 / (2×5.9) ≈ 0.68
```

This is **slightly underdamped** (ζ < 1), which gives fast response with small overshoot.

**Ideal damping:** ζ ≈ 0.7 (critical damping for fast response)

Our system is well-tuned!

---

## Performance Metrics

### Rise Time (tr)

Time to reach 90% of final value:

```
tr ≈ 1.8 / ω_n ≈ 1.8 / 5.9 ≈ 0.3 seconds
```

### Settling Time (ts)

Time to stay within 2% of final value:

```
ts ≈ 4 / (ζ × ω_n) ≈ 4 / (0.68 × 5.9) ≈ 1.0 seconds
```

### Overshoot (Mp)

```
Mp = exp(-π × ζ / sqrt(1 - ζ²))
Mp ≈ exp(-π × 0.68 / 0.73) ≈ 0.05 = 5%
```

These are excellent characteristics for a stabilization system!

---

## Summary

| Term  | Mathematical Role     | Physical Effect          | Frequency Response       |
| ----- | --------------------- | ------------------------ | ------------------------ |
| **P** | Proportional to error | Immediate correction     | All frequencies equally  |
| **I** | Accumulates error     | Eliminates DC offset     | Strong at low freq only  |
| **D** | Rate of change        | Opposes motion (damping) | Strong at high freq only |

**Together:** Fast response (P) + Zero steady-state error (I) + Smooth motion (D) = Stable control!
