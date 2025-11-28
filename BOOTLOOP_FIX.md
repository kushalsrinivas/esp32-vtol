# RECEIVER BOOTLOOP & WHITE SCREEN FIX

## Problem Diagnosis

Your receiver ESP32 was stuck in a bootloop (repeatedly restarting) and showing a flickering white screen. This was caused by **calling TFT display functions inside the ESP-NOW interrupt callback**.

### Root Cause

The ESP-NOW callback `OnDataRecv()` runs in an **ISR (Interrupt Service Routine) context**, which has strict limitations:

1. ❌ **Cannot call SPI functions** (TFT display uses SPI)
2. ❌ **Cannot use printf/Serial.print** safely
3. ❌ **Cannot call delay()**
4. ❌ **Cannot allocate memory**
5. ✅ **Can only**: Set flags, copy data, update simple variables

### What Was Happening

```
Transmitter sends data
    ↓
ESP-NOW interrupt fires
    ↓
OnDataRecv() calls updateDisplay()
    ↓
updateDisplay() calls tft.printf() and other TFT functions
    ↓
TFT library tries to use SPI in ISR context
    ↓
💥 CRASH → ESP32 reboots → White screen → Repeat
```

## The Fix

### Changes Made:

1. **Added a flag variable** (`volatile bool dataReady = false`)
   - Set to `true` in the ISR when new data arrives
   - Checked and cleared in the main loop

2. **Moved all display operations to main loop**
   - ISR only copies data and sets the flag
   - Main loop does all the heavy lifting (graphs, display updates)

3. **Kept ISR minimal and fast**
   - Only essential operations in the callback
   - All time-consuming operations in main loop

### Code Flow After Fix:

```
Transmitter sends data
    ↓
ESP-NOW interrupt fires
    ↓
OnDataRecv() copies data and sets dataReady = true
    ↓
ISR exits quickly ✓
    ↓
Main loop sees dataReady flag
    ↓
Main loop calls updateGraphs() and updateDisplay()
    ↓
Display updates smoothly ✓
```

## How to Test

### 1. Upload the Fixed Code

Upload the corrected `receiver_esp32.ino` to your receiver ESP32.

### 2. Check Serial Monitor

You should see:
```
╔════════════════════════════════════════╗
║  ESP32 RECEIVER - TFT Display         ║
║  with ESP-NOW Telemetry                ║
╚════════════════════════════════════════╝

→ Initializing TFT display...
✓ TFT display initialized

→ Initializing ESP-NOW...
  Receiver MAC Address: FC:E8:C0:E0:D2:F4
✓ ESP-NOW initialized successfully

╔════════════════════════════════════════╗
║  SYSTEM READY - Waiting for data...   ║
╚════════════════════════════════════════╝
```

**No more bootloops!** 🎉

### 3. Check TFT Display

The display should show:
- ✅ Stable startup screen
- ✅ "WAITING..." status (if transmitter not running)
- ✅ Smooth graph updates when transmitter is active
- ✅ No white screen flickering
- ✅ No random resets

### 4. Start Transmitter

When you power on the transmitter:
- Connection indicator should turn GREEN
- Graphs should start scrolling smoothly
- Servo values should update in real-time
- Update rate should show ~20 Hz

## Expected Behavior Now

### Before Data Arrives:
```
┌─────────────────────────────────────────┐
│           TELEMETRY                     │
├─────────────────────────────────────────┤
│ TX: [─────────────────] 0.0°            │
│ TY: [─────────────────] 0.0°            │
├─────────────────────────────────────────┤
│ S1: 0/0   S2: 0/0   GX: 0.0   GY: 0.0   │
│ Rate: 0.0 Hz          Pkts: 0           │
│ Status: 🔴 WAITING...                   │
└─────────────────────────────────────────┘
```

### After Connection:
```
┌─────────────────────────────────────────┐
│           TELEMETRY                     │
├─────────────────────────────────────────┤
│ TX: [~~~Wave~~~] -12.5°                 │
│ TY: [~~~Wave~~~] +8.3°                  │
├─────────────────────────────────────────┤
│ S1: 85/90  S2: 95/90  GX: +2.1  GY: -1.5│
│ Rate: 20.0 Hz          Pkts: 1234       │
│ Status: 🟢 CONNECTED                    │
└─────────────────────────────────────────┘
```

## Technical Details

### What Changed in the Code

**Before (BROKEN):**
```cpp
void OnDataRecv(...) {
    // ... receive data ...
    updateGraphs();      // ❌ SPI calls in ISR!
    updateDisplay();     // ❌ TFT printf in ISR!
}

void loop() {
    // ... only check connection ...
}
```

**After (FIXED):**
```cpp
volatile bool dataReady = false;  // ✓ ISR-safe flag

void OnDataRecv(...) {
    // ... receive data ...
    dataReady = true;    // ✓ Only set flag!
}

void loop() {
    if (dataReady) {
        dataReady = false;
        updateGraphs();     // ✓ Safe in main loop
        updateDisplay();    // ✓ Safe in main loop
    }
    // ... check connection ...
}
```

### ISR Best Practices Applied

✅ **Minimal processing** - Copy data and exit quickly  
✅ **Volatile variables** - Ensures proper access between ISR and main loop  
✅ **Flag-based signaling** - Standard ISR-to-main-loop communication  
✅ **Deferred work** - Heavy operations moved to main loop  
✅ **No blocking calls** - All delays/waits in main loop only  

## Troubleshooting

### If still showing white screen:

1. **Check power supply**
   - TFT needs stable 3.3V
   - Use good quality power source

2. **Verify wiring**
   ```
   TFT Pin → ESP32 Pin
   VCC     → 3.3V
   GND     → GND
   MOSI    → GPIO 23
   SCK     → GPIO 18
   CS      → GPIO 5
   DC      → GPIO 21
   RST     → GPIO 4
   LED     → 3.3V
   ```

3. **Test TFT independently**
   - Upload a simple Adafruit GFX test sketch
   - Verify display works before ESP-NOW

### If bootloop continues:

1. **Check Serial Monitor** for error messages
2. **Verify ESP32 board package** is up to date
3. **Check memory usage** - compile and look for warnings
4. **Try different ESP32 board** - could be hardware issue

## Summary

The fix was simple but critical:
- **Don't do complex operations in interrupt callbacks**
- **Use flags to signal main loop**
- **Keep ISRs fast and minimal**

This is a common embedded systems pattern and essential for stable ESP32 operation!

---
**Status:** ✅ FIXED - Receiver should now run stably without bootloops or flickering



