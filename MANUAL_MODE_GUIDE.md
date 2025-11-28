# Manual Mode Feature Guide

## Overview
The Manual Mode feature allows you to control the transmitter's servos directly using the receiver's joystick, bypassing the automatic PID stabilization.

## Features Added

### Receiver (receiver_esp32.ino)
- **New Menu Item**: "Manual Mode" (7th item in main menu)
- **Joystick Control**: 
  - Y-axis → Pitch control (up/down movement)
  - X-axis → Roll control (left/right movement)
- **Real-time Display**: Shows joystick position, servo angles, and transmission status
- **Auto-exit**: Automatically sends "resume auto" command when leaving Manual Mode screen

### Transmitter (transmitter_esp32.ino)
- **Bidirectional Communication**: Now receives control commands from receiver
- **Manual Mode Logic**: 
  - Disables PID control when in manual mode
  - Applies direct joystick commands to servos
  - Resets PID integrators to prevent windup
- **Safety Timeout**: Automatically resumes auto-stabilization if no command received for 500ms
- **Status Reporting**: Serial monitor shows mode (MANUAL/AUTO) and relevant data

## Setup Instructions

### Step 1: Update MAC Addresses

**In receiver_esp32.ino (line ~90):**
```cpp
uint8_t transmitterAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // UPDATE THIS!
```

**To find the transmitter's MAC address:**
1. Upload `transmitter_esp32.ino` to your transmitter ESP32
2. Open Serial Monitor (115200 baud)
3. Look for: "Transmitter MAC Address: XX:XX:XX:XX:XX:XX"
4. Copy this address to the `transmitterAddress[]` array in receiver code

**Example:**
If transmitter shows: `A1:B2:C3:D4:E5:F6`
Update receiver to:
```cpp
uint8_t transmitterAddress[] = {0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6};
```

### Step 2: Upload Code
1. Upload **transmitter_esp32.ino** to transmitter ESP32 first
2. Note the MAC address from serial monitor
3. Update receiver code with transmitter MAC address
4. Upload **receiver_esp32.ino** to receiver ESP32

### Step 3: Verify Setup
**On Transmitter Serial Monitor:**
- Should see: "✓ ESP-NOW initialized successfully"
- Should see: "SYSTEM READY - Starting Main Loop"
- In AUTO mode: Shows pitch/roll angles and servo positions

**On Receiver Serial Monitor:**
- Should see: "✓ Transmitter peer added successfully"
- Should see: "Can send control data to: XX:XX:XX:XX:XX:XX"

## How to Use Manual Mode

### Entering Manual Mode
1. From main menu, navigate with joystick UP/DOWN to "Manual Mode"
2. Press joystick button (center push) to enter
3. Display shows joystick control interface

### Controlling Servos
- **Push joystick UP**: Pitch up (both elevons move together)
- **Push joystick DOWN**: Pitch down (both elevons move together)
- **Push joystick LEFT**: Roll left (elevons move differentially)
- **Push joystick RIGHT**: Roll right (elevons move differentially)
- **Deadzone**: Small movements near center are ignored (prevents drift)

### Display Elements
- **Joystick Position**: Visual indicator showing stick position
- **Pitch/Roll Values**: Numeric readout (-1.00 to +1.00)
- **Servo Angles**: Real-time elevon positions (R: right, L: left)
- **TX Status**: Shows if control commands are being sent successfully

### Exiting Manual Mode
- **Press joystick button**: Return to main menu
- **Navigate UP/DOWN**: Switch to another screen
- Transmitter automatically resumes PID auto-stabilization

## Control Mapping

### Joystick to Servo Conversion
```
Joystick Y (Pitch): -1.0 (down) to +1.0 (up)
Joystick X (Roll):  -1.0 (left) to +1.0 (right)

Servo Range: 30° to 150° (±60° from 90° center)

Right Elevon = 90° + (pitchCmd * 60°) + (rollCmd * 60°)
Left Elevon  = 90° + (pitchCmd * 60°) - (rollCmd * 60°)
```

**Example:**
- Joystick full up (pitch=+1.0, roll=0.0):
  - Right Elevon: 90 + 60 + 0 = 150°
  - Left Elevon: 90 + 60 - 0 = 150°
  - Result: Both elevons up → nose pitch up

- Joystick full right (pitch=0.0, roll=+1.0):
  - Right Elevon: 90 + 0 + 60 = 150°
  - Left Elevon: 90 + 0 - 60 = 30°
  - Result: Right up, left down → roll right

## Safety Features

### Connection Loss Protection
- **Timeout**: If no control command received for 500ms, transmitter automatically:
  1. Exits manual mode
  2. Resumes PID auto-stabilization
  3. Prints warning to serial monitor

### Servo Range Limiting
- All servo commands are constrained to 30°-150° range
- Prevents mechanical binding and servo damage

### PID Integrator Reset
- When entering manual mode, PID integrators are zeroed
- Prevents sudden jumps when switching back to auto mode

## Troubleshooting

### Manual Mode Not Working
**Symptom**: Can't send control commands, "TX: FAIL" shown

**Solutions:**
1. Verify transmitter MAC address is correct in receiver code
2. Check that transmitter ESP32 is powered on and running
3. Ensure both ESP32s are close enough (ESP-NOW range ~200m line-of-sight)
4. Check serial monitor on receiver for peer add errors

### Servos Not Moving in Manual Mode
**Symptom**: Joystick moves but servos don't respond

**Solutions:**
1. Check transmitter serial monitor - should show "MANUAL MODE: Pitch=X.XX Roll=X.XX"
2. Verify servo connections and power supply (servos need external 5V)
3. Test servos in auto mode first to ensure they work
4. Check joystick calibration (center should be ~2048)

### Servo Movement Reversed
**Symptom**: Joystick up makes nose go down, or roll direction is wrong

**Solutions:**
1. In transmitter code, line ~453, adjust the signs:
   ```cpp
   // If pitch is reversed, change:
   float manual_pitch_deflection = -manualPitchCmd * ELEVON_RANGE;
   // to:
   float manual_pitch_deflection = manualPitchCmd * ELEVON_RANGE;
   
   // If roll is reversed, change:
   float manual_roll_deflection = manualRollCmd * ELEVON_RANGE;
   // to:
   float manual_roll_deflection = -manualRollCmd * ELEVON_RANGE;
   ```

### Transmitter Won't Resume Auto Mode
**Symptom**: After exiting manual mode, aircraft doesn't stabilize

**Solutions:**
1. Power cycle the transmitter ESP32
2. Check serial monitor - should see "⚠ Manual mode timeout - resuming auto stabilization"
3. Verify PID parameters haven't been changed accidentally

## Technical Details

### Data Structures

**ControlData** (Receiver → Transmitter):
```cpp
{
    bool manualMode;       // true = manual, false = auto
    float pitchCommand;    // -1.0 to +1.0
    float rollCommand;     // -1.0 to +1.0
    uint32_t timestamp;    // millis()
}
```

### Communication Protocol
- **Update Rate**: ~50Hz (20ms interval) when in manual mode screen
- **Timeout**: 500ms (if no command, resume auto)
- **Protocol**: ESP-NOW (low latency, connectionless)

### Performance
- **Latency**: ~10-30ms from joystick input to servo movement
- **Range**: Up to 200m line-of-sight (ESP-NOW limitation)
- **Bandwidth**: Minimal (~24 bytes per control packet)

## FAQ

**Q: Can I use manual mode while flying?**
A: Yes, but be cautious! If you lose connection or exit manual mode, the transmitter will resume auto-stabilization which may cause abrupt movements.

**Q: Does manual mode affect telemetry display?**
A: No, telemetry continues to flow from transmitter to receiver. Other screens still show sensor data.

**Q: Can I adjust PID gains in manual mode?**
A: PID is completely disabled in manual mode. Gains only affect auto-stabilization mode.

**Q: What happens if I switch screens while in manual mode?**
A: The receiver automatically sends an "exit manual mode" command, and the transmitter resumes auto-stabilization.

**Q: Can I use manual mode without the MPU6050?**
A: No, the transmitter still needs the MPU6050 for telemetry data even in manual mode. The sensor data is just not used for control.

## Future Enhancements (Not Yet Implemented)

Possible improvements for future versions:
- Toggle manual mode with a button (stays active across screens)
- Exponential control curves for smoother inputs
- Trim adjustments for manual mode
- Rate vs. Angle mode selection
- Record and playback maneuvers
- Dual-rate switches (high/low sensitivity)

---

**Created**: November 28, 2025  
**Compatible with**: ESP32 Arduino Core v3.x+  
**Tested on**: ESP32 DevKit v1 with ILI9341 display

