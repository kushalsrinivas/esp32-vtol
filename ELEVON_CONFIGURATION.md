# Elevon Configuration for Tailsitter Aircraft

## Overview

This flight controller has been configured for a **tailsitter aircraft with elevon control surfaces**. The servos are mounted underneath the aircraft and use standard elevon mixing for pitch and roll control.

## Hardware Configuration

### Servo Assignments

- **GPIO 25** → **RIGHT Elevon** (mounted underneath aircraft)
- **GPIO 26** → **LEFT Elevon** (mounted underneath aircraft)

### Servo Range Limits

- **Center Position**: 90°
- **Minimum Angle**: 30° (90° - 60°)
- **Maximum Angle**: 150° (90° + 60°)
- **Total Range**: ±60° from center

This limited range prevents the servos from hitting the aircraft structure.

## Elevon Mixing Logic

### Standard Elevon Mixing Formula

```
Right Elevon = Center + Pitch PID + Roll PID
Left Elevon  = Center + Pitch PID - Roll PID
```

### Control Behavior

#### Pitch Control (Both elevons move together)

- **Nose Up** (positive pitch): Both elevons deflect up
- **Nose Down** (negative pitch): Both elevons deflect down

#### Roll Control (Elevons move differentially)

- **Roll Right** (positive roll): Right elevon down, left elevon up
- **Roll Left** (negative roll): Right elevon up, left elevon down

## PID Controllers

### Pitch PID

- Controls overall pitch angle
- Uses gyroscope Y-axis for derivative term
- Output range: ±60° (limited to ELEVON_RANGE)

### Roll PID

- Controls roll angle
- Uses gyroscope X-axis for derivative term
- Output range: ±60° (limited to ELEVON_RANGE)

### Current PID Values

```
Pitch: Kp=45.0, Ki=5.0, Kd=8.0
Roll:  Kp=45.0, Ki=5.0, Kd=8.0
Integrator limit: 25.0
```

## Code Changes Summary

### Key Modifications

1. **Renamed variables** from servo1/servo2 to right_elevon/left_elevon
2. **Separated PID controllers** into pitch and roll (instead of X/Y axes)
3. **Implemented elevon mixing** after PID computation
4. **Added range constraints** (30° to 150°) to prevent mechanical binding
5. **Updated telemetry** to reflect pitch/roll and elevon positions
6. **Updated serial output** for better debugging of elevon system

### Safety Features

- Hard limits on elevon angles (30° - 150°)
- PID output constrained to ±60°
- Integral anti-windup protection
- Constrained pitch/roll input angles (±45° max)

## Testing Recommendations

1. **Power off test**: Manually move elevons through full range to verify no binding
2. **Bench test**: With aircraft secured, verify elevon movements:
   - Tilt nose up → both elevons should deflect up
   - Tilt nose down → both elevons should deflect down
   - Roll right → right elevon down, left elevon up
   - Roll left → right elevon up, left elevon down
3. **Range check**: Ensure servos don't hit limits during extreme movements
4. **PID tuning**: May need adjustment based on aircraft response

## Telemetry Data

The following data is transmitted via ESP-NOW:

- Pitch angle (degrees)
- Roll angle (degrees)
- Right elevon position (degrees)
- Left elevon position (degrees)
- Pitch rate (gyro)
- Roll rate (gyro)
- Accelerometer X, Y, Z
- Timestamp

## Notes

- Servos initialize to center position (90°) on startup
- Elevon mixing happens after PID computation for better control
- If servo directions are reversed, you may need to invert the mixing formula
- The current setup assumes standard servo orientation (underneath aircraft)

## Future Adjustments

If elevon directions are incorrect during flight testing:

1. Swap the mixing formula (change + to - and vice versa)
2. Or reverse individual servo connections physically
3. Can also adjust gyro axis assignments if needed
