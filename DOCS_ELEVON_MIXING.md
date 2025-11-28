# Elevon Mixing Mathematics - Deep Dive

## Table of Contents
1. [Introduction to Elevons](#introduction-to-elevons)
2. [Control Surface Theory](#control-surface-theory)
3. [Mixing Mathematics](#mixing-mathematics)
4. [Geometric Analysis](#geometric-analysis)
5. [Servo Mechanics](#servo-mechanics)
6. [Practical Implementation](#practical-implementation)
7. [Troubleshooting](#troubleshooting)

---

## Introduction to Elevons

### What are Elevons?

**Elevons** = **Elev**ators + Ailer**ons** (portmanteau)

Control surfaces that provide both:
- **Pitch control** (like elevators on conventional aircraft)
- **Roll control** (like ailerons on conventional aircraft)

```
Conventional Aircraft:
                    
     ┌────────┬────────┐
     │Aileron │Aileron │  ← Roll control
     └────────┴────────┘
          │   │
          │   │ Fuselage
          │   │
          │   │
        ┌─┴───┴─┐
        │Elevator│  ← Pitch control
        └────────┘

Flying Wing / Tailsitter:
                    
     ┌────────┬────────┐
     │ELEVON  │ELEVON  │  ← BOTH pitch AND roll!
     └────────┴────────┘
          │   │
          │   │ (No tail)
          └───┘
```

### Where Used?

```
1. FLYING WINGS
   - No horizontal or vertical tail
   - All control from wing trailing edge
   - Example: B-2 bomber, RC flying wings

2. DELTA WINGS
   - Triangular wing planform
   - Elevons on trailing edge
   - Example: Concorde, many RC planes

3. TAILSITTER VTOL
   - Takes off/lands vertically
   - Flies horizontally
   - Elevons control both modes
   - Our application! ✓
```

---

## Control Surface Theory

### Conventional Control (3-Axis)

```
Traditional airplane has 3 control axes:

1. PITCH (Nose up/down)
   ────────────────────────
   Control surface: Elevator (on tail)
   Motion: Rotation around lateral axis
   
        ┌─────────┐    ▲ Nose up
        │ Elevator│    │ (positive pitch)
        │   UP    │    │
        └─────────┘    │
                      ╱
                     ╱  Rotation
                    ◄   axis
   
2. ROLL (Wing up/down)
   ────────────────────────
   Control surface: Ailerons (on wings)
   Motion: Rotation around longitudinal axis
   
     ┌────────┐         ┌────────┐
     │ Aileron│         │Aileron │
     │  DOWN  │         │   UP   │
     └────────┘         └────────┘
          Right wing            Left wing
          goes DOWN             goes UP
              ╲                 ╱
               ╲               ╱
                ◄─────────────►
                Roll axis
   
3. YAW (Nose left/right)
   ────────────────────────
   Control surface: Rudder (on tail)
   Motion: Rotation around vertical axis
   (NOT used in elevon systems)
```

### Elevon System (2-Axis)

```
Elevons combine pitch and roll into 2 surfaces:

         LEFT ELEVON    RIGHT ELEVON
              │              │
              ▼              ▼
        ┌─────────┐    ┌─────────┐
        │         │    │         │
        │    L    │    │    R    │
        │         │    │         │
        └─────────┘    └─────────┘

PITCH COMMAND:
  Both move TOGETHER
  L = pitch    R = pitch
  
ROLL COMMAND:
  Move OPPOSITE (differential)
  L = -roll    R = +roll
  
COMBINED:
  L = pitch - roll
  R = pitch + roll
```

---

## Mixing Mathematics

### Standard Mixing Formula

```
The fundamental elevon mixing equations:

Right Elevon = Pitch + Roll
Left Elevon  = Pitch - Roll

Where:
- Pitch: Desired pitch correction (°)
- Roll: Desired roll correction (°)
- Sign convention: Positive = trailing edge down
```

### Our Modified Formula

```cpp
// We use NEGATIVE pitch due to servo mounting:
right_elevon_angle = ELEVON_CENTER - pitch_PID + roll_PID;
left_elevon_angle = ELEVON_CENTER - pitch_PID - roll_PID;
```

**Why negative pitch?**
- Servos mounted underneath aircraft
- Servo linkage reverses direction
- Positive pitch PID needs negative servo movement

### Vector Representation

```
Think of elevons as a 2D vector system:

Pitch axis:    [1,  1]   (both move together)
Roll axis:     [1, -1]   (move opposite)

Output = Center + pitch × [1, 1] + roll × [1, -1]

Example:
  pitch_PID = 10°
  roll_PID = 5°
  
  Right = 90 + 10×1 + 5×1  = 105°
  Left  = 90 + 10×1 + 5×(-1) = 95°
```

### Matrix Form

```
Mixing can be expressed as matrix multiplication:

┌────────┐   ┌──────────┐   ┌───────┐
│ Right  │   │  1    1  │   │ Pitch │
│        │ = │          │ × │       │  + Center
│ Left   │   │  1   -1  │   │ Roll  │
└────────┘   └──────────┘   └───────┘
             Mixing Matrix

This is a LINEAR transformation:
- Orthogonal basis vectors
- Invertible (can unmix)
- Preserves control authority
```

### Inverse Mixing (Unmixing)

```
Given elevon positions, compute pitch and roll:

Pitch = (Right + Left) / 2
Roll  = (Right - Left) / 2

Example:
  Right = 105°, Left = 95°
  
  Pitch = (105 + 95) / 2 = 100° from center
  Roll  = (105 - 95) / 2 = 5° differential
```

---

## Geometric Analysis

### Pure Pitch Movement

```
Pitch Up Command:
─────────────────

   Aircraft         Elevon Position        Effect
   
     ┌───┐           LEFT    RIGHT
     │ ▲ │           DOWN    DOWN          Nose
     │ │ │            │        │           rises
     │ │ │            ▼        ▼             ▲
  ───┴─┴─┴───     =========  =========       │
                                             │
  BEFORE          ╲═══════╱  ╲═══════╱    AFTER
                   deflect    deflect

Both elevons deflect DOWN (trailing edge down)
→ Increases lift at tail
→ Nose pitches UP

Math:
  pitch_PID = -30° (need nose up correction)
  roll_PID = 0°
  
  Right = 90 - (-30) + 0 = 120° (down)
  Left  = 90 - (-30) - 0 = 120° (down)
  
  Both servos at 120° → trailing edges down → nose up ✓
```

### Pure Roll Movement

```
Roll Right Command:
───────────────────

   Aircraft         Elevon Position        Effect
   
     ┌───┐           LEFT    RIGHT
     │   │─►          UP     DOWN         Right wing
     │   │            ▲        │          goes down
     │   │            │        ▼            ↓
  ───┴───┴───     =========  =========   
                  
  BEFORE          ╱═══════╲  ╲═══════╱    AFTER
                   deflect    deflect       (rolled)

Left elevon UP, Right elevon DOWN
→ Left wing: less lift (rises)
→ Right wing: more lift (descends)
→ Aircraft rolls RIGHT

Math:
  pitch_PID = 0°
  roll_PID = 30° (need right roll correction)
  
  Right = 90 - 0 + 30 = 120° (down)
  Left  = 90 - 0 - 30 = 60°  (up)
  
  Differential deflection → roll right ✓
```

### Combined Movement

```
Pitch Up + Roll Right:
──────────────────────

   Aircraft         Elevon Position        Effect
   
     ┌───┐           LEFT    RIGHT
     │ ▲ │─►         DOWN    DOWN         Nose up
     │ │ │           (less)  (more)       + roll right
     │ │ │            │        │             ▲  ╲
  ───┴─┴─┴───     =========  =========       │   ╲

  BEFORE          ╲══════╲  ╲═══════╱╱    AFTER
                  small     large
                  deflect   deflect

Math:
  pitch_PID = -30° (nose up)
  roll_PID = 15° (roll right)
  
  Right = 90 - (-30) + 15 = 135° (more down)
  Left  = 90 - (-30) - 15 = 105° (less down)
  
  Both down (pitch up) + differential (roll right) ✓
```

---

## Servo Mechanics

### Servo Angle Convention

```
Standard Servo Angles:
0° ─────── 90° ─────── 180°
MIN      CENTER       MAX

Physical position:
0°:   Full left/down
90°:  Centered (neutral)
180°: Full right/up

Our constrained range:
30° ──── 90° ──── 150°
MIN    CENTER    MAX
(±60° from center)
```

### Mechanical Advantage

```
Servo arm and control horn geometry affects response:

         Servo arm           Control surface
              │                    │
    ╔═════════╧═════════╗    ┌─────▼─────┐
    ║  SERVO            ║────┤  ELEVON   │
    ╚═══════════════════╝    └───────────┘
              ║
         Servo angle θ
         
Deflection angle φ = (L_arm / L_horn) × θ

Example:
  Servo arm: 20mm
  Control horn: 15mm
  Servo angle: 30°
  
  Deflection: (20/15) × 30° = 40° surface deflection

Typical ratios:
  1:1  → Direct drive (most common)
  2:1  → More deflection (less precise)
  1:2  → Less deflection (more precise)
```

### Linkage Geometry

```
Pushrod vs Pull-pull cable:

PUSHROD (rigid):
  Servo ═══════O══════ Elevon
          (rigid rod)
  
  Pros:
  + No slack
  + Precise
  + Bi-directional
  
  Cons:
  - Can bend under load
  - Friction in bends
  - Needs straight path

PULL-PULL (cable):
  Servo ─────────────╮
         (cable)     │  Elevon
  Servo ─────────────╯
  
  Pros:
  + Lightweight
  + No binding
  + Can route anywhere
  
  Cons:
  - Needs tensioning
  - Can stretch
  - Unidirectional
```

---

## Practical Implementation

### Our Code

```cpp
// ========== ELEVON MIXING ==========
// PITCH REVERSED: Servos mounted sideways, need negative pitch
// Right Elevon = -Pitch + Roll
// Left Elevon  = -Pitch - Roll
right_elevon_angle = ELEVON_CENTER - pitch_PID + roll_PID;
left_elevon_angle = ELEVON_CENTER - pitch_PID - roll_PID;

// Constrain to safe range (30° to 150°, ±60° from center)
right_elevon_angle = constrain(right_elevon_angle, ELEVON_MIN, ELEVON_MAX);
left_elevon_angle = constrain(left_elevon_angle, ELEVON_MIN, ELEVON_MAX);

// Write to servos
right_elevon.write((int)right_elevon_angle);
left_elevon.write((int)left_elevon_angle);
```

### Range Limiting

```
Why limit to 30°-150° instead of 0°-180°?

PHYSICAL CONSTRAINTS:
┌────────────────────────────────────────┐
│                                        │
│  30°              90°             150° │
│   │                │                │  │
│   └────────────────┼────────────────┘  │
│    SAFE RANGE      │   SAFE RANGE      │
│                    │                   │
│ 0°-30°: May hit structure              │
│ 150°-180°: May hit structure           │
└────────────────────────────────────────┘

Benefits:
✓ Prevents mechanical binding
✓ Avoids servo stall
✓ Reduces wear
✓ Safety margin
```

### Saturation Handling

```
What happens when PID output exceeds limits?

Scenario: Large pitch command (pitch_PID = -80°)
  
Unconstrained:
  Right = 90 - (-80) + 0 = 170° ← EXCEEDS 150° max!
  Left  = 90 - (-80) - 0 = 170° ← EXCEEDS 150° max!

After constraint:
  Right = constrain(170, 30, 150) = 150°
  Left  = constrain(170, 30, 150) = 150°

Result:
  Servos saturate at 150°
  Further pitch commands have NO effect
  Control authority is LOST

Prevention:
- Limit PID output BEFORE mixing
```cpp
pitch_PID = constrain(pitch_PID, -ELEVON_RANGE, ELEVON_RANGE);
                               // ±60°
```
- This ensures total range (pitch + roll) stays within limits
```

### Asymmetric Saturation

```
Problem: One servo saturates, other doesn't

Scenario: pitch_PID = -55°, roll_PID = 25°

Right = 90 - (-55) + 25 = 170° → 150° (saturated)
Left  = 90 - (-55) - 25 = 120° (OK)

Result:
  Right servo at limit
  Left servo still has authority
  → Uneven response
  → Unintended roll!

Solution:
  Pre-calculate combined worst-case:
```cpp
float max_combined = max(abs(pitch_PID + roll_PID), 
                         abs(pitch_PID - roll_PID));
if (max_combined > ELEVON_RANGE) {
    float scale = ELEVON_RANGE / max_combined;
    pitch_PID *= scale;
    roll_PID *= scale;
}
```
  Now both commands scaled proportionally
```

---

## Control Authority

### Maximum Deflection

```
With our ±60° range:

Pure pitch: ±60° on BOTH elevons
Pure roll:  ±60° differential (120° total between them)

Maximum combined:
  If pitch_PID = -60° and roll_PID = 60°:
    Right = 90 - (-60) + 60 = 210° → 150° (saturated!)
    Left  = 90 - (-60) - 60 = 30°  (at limit)
  
  This shows: Can't have max pitch AND max roll simultaneously
```

### Authority Envelope

```
Graph of available control authority:

Roll (°)
  ▲
60│       ╱│╲         Saturation boundary
  │      ╱ │ ╲        (pitch + roll > 60°)
  │     ╱  │  ╲
  │    ╱   │   ╲
  │   ╱    │    ╲
  │  ╱     │     ╲
  │ ╱      │      ╲
  ├────────┼────────┤────► Pitch (°)
 -60      -60  0   60
  │╲       │      ╱│
  │ ╲      │     ╱ │
  │  ╲     │    ╱  │
  │   ╲    │   ╱   │
  │    ╲   │  ╱    │
  │     ╲  │ ╱     │
  │      ╲ │╱      │
-60       ╲│       │

Inside diamond: Full authority
On edge: One servo at limit
Outside: Saturated (clipped to edge)
```

### Control Coupling

```
Elevons inherently couple pitch and roll:

Pure pitch command:
  ✓ Both elevons down → pitch up
  ✗ If not exactly equal → unintended roll

Pure roll command:
  ✓ Differential → roll
  ✗ Average deflection ≠ 0 → unintended pitch

Mitigation:
1. Accurate servo centering (calibration)
2. Matched servos (same speed/torque)
3. Tight mechanical linkage (no play)
4. PID tuning to compensate
```

---

## Coordinate Systems

### Body Frame

```
Aircraft body-fixed coordinate system:

         X (forward)
         │
         │
         ◄────► Y (right)
        ╱
       ╱
      Z (down)

Rotations:
- Roll (φ):  Rotation around X-axis
- Pitch (θ): Rotation around Y-axis
- Yaw (ψ):   Rotation around Z-axis (not controlled by elevons)

Our sensors measure:
- MPU6050 pitch = rotation around Y-axis
- MPU6050 roll = rotation around X-axis
```

### Control Frame

```
Elevon control coordinate system:

Symmetric (pitch):
  Both elevons move together
  Axis: [1, 1]
  
Antisymmetric (roll):
  Elevons move opposite
  Axis: [1, -1]

These form an ORTHOGONAL basis:
  [1, 1] · [1, -1] = 1×1 + 1×(-1) = 0 ✓

Any elevon position can be expressed as:
  Position = center + pitch×[1,1]/2 + roll×[1,-1]/2
```

### Transformation

```
Body frame → Control frame:

┌────────┐   ┌──────────┐   ┌───────┐
│ Right  │   │  1    1  │   │ Pitch │
│        │ = │          │ × │       │
│ Left   │   │  1   -1  │   │ Roll  │
└────────┘   └──────────┘   └───────┘

Determinant = |1×(-1) - 1×1| = |-2| = 2 ≠ 0
→ Matrix is invertible ✓
→ One-to-one mapping ✓
→ No information lost ✓
```

---

## Aerodynamic Principles

### Lift Equation

```
Lift = ½ × ρ × V² × S × CL

Where:
- ρ: Air density (kg/m³)
- V: Airspeed (m/s)
- S: Wing area (m²)
- CL: Lift coefficient (depends on angle of attack)

Elevon deflection changes CL:
  ΔCL = CL_α × δ
  
Where:
- CL_α: Lift curve slope (≈ 2π per radian, for thin airfoil)
- δ: Elevon deflection angle (radians)

Example:
  Elevon deflection: 10° = 0.174 rad
  CL_α ≈ 6.28 rad⁻¹
  ΔCL = 6.28 × 0.174 = 1.09
  
  Lift increases by factor of 1 + 1.09 = 2.09 ✓
```

### Pitching Moment

```
Pitching moment about center of gravity:

M = L × d

Where:
- L: Lift from elevon
- d: Distance from CG to elevon (moment arm)

Elevon effectiveness increases with:
✓ Larger deflection angle
✓ Longer moment arm (elevon far from CG)
✓ Higher airspeed (more air over surface)
✓ Larger elevon area
```

### Roll Rate

```
Roll rate (p) induced by elevons:

p = (L_right - L_left) × b / (2 × I_xx)

Where:
- L_right, L_left: Lift from right/left elevons
- b: Wing span
- I_xx: Roll moment of inertia

Example:
  Right elevon: 120° (down) → more lift
  Left elevon:  60° (up)   → less lift
  Differential lift creates rolling moment
```

---

## Troubleshooting

### Direction Reversal

```
Problem: Elevons move opposite to intended

Pitch command → Nose goes DOWN instead of up
Roll command → Aircraft rolls LEFT instead of right

Solutions:

1. Swap servo signals:
```cpp
right_elevon.attach(LEFT_ELEVON_PIN);   // Swap
left_elevon.attach(RIGHT_ELEVON_PIN);   // Swap
```

2. Reverse mixing signs:
```cpp
right_elevon_angle = ELEVON_CENTER + pitch_PID - roll_PID;  // Change signs
left_elevon_angle = ELEVON_CENTER + pitch_PID + roll_PID;
```

3. Reverse PID signs (already done for pitch):
```cpp
right_elevon_angle = ELEVON_CENTER - pitch_PID + roll_PID;  // Negative pitch
```
```

### Unequal Response

```
Problem: One elevon responds more than the other

Causes:
1. Servo speed mismatch
   - Different servo models
   - One servo worn out
   - Solution: Use matched pair

2. Mechanical binding
   - Linkage friction
   - Control horn interference
   - Solution: Free up linkage, add lubricant

3. Servo calibration
   - Different servo centers
   - Non-linear response curves
   - Solution: Calibrate trim or use servo programmer

4. PID coupling
   - Gyro axes not aligned with aircraft
   - Sensor mounting angle off
   - Solution: Re-calibrate MPU6050
```

### Flutter

```
Problem: Elevons oscillate rapidly (flutter)

Causes:
1. PID gains too high
   - Over-correction
   - Solution: Reduce Kp, increase Kd

2. Mechanical slack
   - Play in linkage
   - Loose servo arm
   - Solution: Tighten all connections

3. Servo jitter
   - Noisy PWM signal
   - Power supply ripple
   - Solution: Add capacitor, filter signal

4. Aerodynamic instability
   - Control surface too large
   - CG too far back
   - Solution: Reduce deflection limits, move CG forward
```

### Limited Authority

```
Problem: Servos don't deflect enough

Causes:
1. PID gains too low
   - Weak correction
   - Solution: Increase Kp

2. Saturation
   - Commands exceed limits
   - Solution: Check constraints, increase limits if safe

3. Mechanical limits
   - Servo hitting structure
   - Linkage at max extension
   - Solution: Adjust linkage geometry

4. Low voltage
   - Servos underpowered
   - Slow response
   - Solution: Check battery, use BEC
```

---

## Advanced Topics

### Elevon Gain Tuning

```
Independent pitch and roll gains:

Traditional:
```cpp
float elevon_gain = 1.0;  // Same for both axes
```

Advanced:
```cpp
float pitch_gain = 1.0;
float roll_gain = 0.8;   // Reduce roll sensitivity

pitch_PID *= pitch_gain;
roll_PID *= roll_gain;
```

Why different gains?
- Aircraft may be more sensitive in roll than pitch
- Center of gravity affects pitch authority
- Wing dihedral affects roll stability
```

### Dynamic Mixing Ratios

```
Adjust mixing based on flight conditions:

```cpp
// At high speed: Reduce deflections (more effective)
float speed_factor = constrain(airspeed / 20.0, 0.5, 1.0);
pitch_PID *= speed_factor;
roll_PID *= speed_factor;

// At high angle of attack: Increase deflections (less effective)
float aoa_factor = 1.0 + 0.5 * abs(pitch) / 45.0;
pitch_PID *= aoa_factor;
```

### Non-Linear Mixing

```
For extreme maneuvers:

```cpp
// Soft saturation (exponential)
float saturate_soft(float x, float limit) {
    return limit * tanh(x / limit);
}

pitch_PID = saturate_soft(pitch_PID, ELEVON_RANGE);
roll_PID = saturate_soft(roll_PID, ELEVON_RANGE);
```

This provides smooth transition near limits rather than hard clipping.
```

---

## Summary

Elevon mixing combines **pitch and roll** control into **two surfaces**:

```
Key equations:
  Right = Center - Pitch + Roll
  Left  = Center - Pitch - Roll

Key concepts:
✓ Linear transformation (matrix)
✓ Orthogonal control axes
✓ Saturation must be handled carefully
✓ Geometric constraints limit combined authority
✓ Servo mechanics affect response
✓ Aerodynamics depend on airspeed and angle

Implementation checklist:
☑ Correct mixing signs (test each axis)
☑ Proper range limits (prevent binding)
☑ Saturation handling (avoid asymmetry)
☑ Matched servos (equal response)
☑ Tight linkages (no slack)
☑ Calibrated center positions
☑ Appropriate PID gains
```

---

**END OF ELEVON MIXING DOCUMENTATION**

