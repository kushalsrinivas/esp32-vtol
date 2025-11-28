# ESP-NOW Telemetry System Setup Guide

## 📡 System Overview

This system uses **two ESP32 boards** to transmit MPU6050 telemetry data wirelessly:

- **Transmitter ESP32**: Reads MPU6050 sensor, controls servos with PID, sends telemetry via ESP-NOW
- **Receiver ESP32**: Receives telemetry data, displays real-time graphs on ILI9341 TFT screen

**Key Features:**

- Real-time wireless telemetry (20Hz update rate)
- Low latency (ESP-NOW is faster than WiFi/Bluetooth)
- No router required (direct ESP32-to-ESP32 communication)
- Range: Up to 250 meters (line of sight)
- Live scrolling graphs and numerical data display

---

## 🔧 Hardware Requirements

### Transmitter ESP32

- 1x ESP32 Development Board
- 1x MPU6050 IMU Sensor
- 2x Servo Motors (e.g., SG90, MG90S)
- External 5V power supply for servos (REQUIRED)
- Jumper wires
- Breadboard (optional)

### Receiver ESP32

- 1x ESP32 Development Board
- 1x ILI9341 2.4" TFT LCD Display (320x240)
- Jumper wires

---

## 📐 Wiring Diagrams

### Transmitter ESP32 Wiring

```
┌─────────────┐
│   MPU6050   │
├─────────────┤
│ VCC → 3.3V  │
│ GND → GND   │
│ SDA → GPIO21│
│ SCL → GPIO22│
└─────────────┘

┌──────────────────┐
│   SERVO 1 (X)    │
├──────────────────┤
│ Signal → GPIO 25 │
│ VCC → 5V (EXT)   │◄── IMPORTANT: Use external 5V supply!
│ GND → GND        │
└──────────────────┘

┌──────────────────┐
│   SERVO 2 (Y)    │
├──────────────────┤
│ Signal → GPIO 26 │
│ VCC → 5V (EXT)   │◄── IMPORTANT: Use external 5V supply!
│ GND → GND        │
└──────────────────┘
```

**⚠️ CRITICAL:** Never power servos from ESP32 3.3V! Use external 5V power supply.

### Receiver ESP32 Wiring

```
┌─────────────────────┐
│   ILI9341 TFT LCD   │
├─────────────────────┤
│ VCC → 3.3V          │
│ GND → GND           │
│ SDA (MOSI) → GPIO23 │
│ SCK → GPIO 18       │
│ CS → GPIO 5         │
│ DC (A0) → GPIO 21   │
│ RST → GPIO 4        │
│ LED → 3.3V          │
└─────────────────────┘
```

---

## 🚀 Step-by-Step Setup Instructions

### Step 1: Install Required Libraries

Open Arduino IDE and install these libraries via **Library Manager**:

1. **ESP32Servo** by Kevin Harrington

   - Sketch → Include Library → Manage Libraries
   - Search: "ESP32Servo"
   - Install latest version

2. **Adafruit GFX Library** by Adafruit

   - Search: "Adafruit GFX"
   - Install latest version

3. **Adafruit ILI9341** by Adafruit

   - Search: "Adafruit ILI9341"
   - Install latest version

4. **ESP32 Board Support** (REQUIRED: v3.0.0 or newer)
   - File → Preferences
   - Add to "Additional Board Manager URLs":
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Tools → Board → Boards Manager
   - Search: "ESP32"
   - Install "esp32" by Espressif Systems (version 3.0.0+)
   - ⚠️ **Important:** This code requires ESP32 core v3.x or newer due to ESP-NOW API changes

---

### Step 2: Upload Receiver Code FIRST

**Why first?** You need the receiver's MAC address to configure the transmitter.

1. **Wire up the Receiver ESP32** according to the wiring diagram above

2. **Open** `receiver_esp32.ino` in Arduino IDE

3. **Select Board:**

   - Tools → Board → ESP32 Arduino → ESP32 Dev Module

4. **Select Port:**

   - Tools → Port → (Select your ESP32's COM port)

5. **Upload the code**

6. **Open Serial Monitor** (115200 baud)

7. **Copy the MAC address** displayed in the serial monitor

   - Example: `A4:CF:12:34:56:78`

8. **Keep the receiver running** - you'll see it waiting for data

---

### Step 3: Configure and Upload Transmitter Code

1. **Wire up the Transmitter ESP32** according to the wiring diagram above

2. **Open** `transmitter_esp32.ino` in Arduino IDE

3. **⚠️ IMPORTANT: Update the MAC address**

   Find this line around line 54:

   ```cpp
   uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
   ```

   Replace it with your receiver's MAC address from Step 2:

   ```cpp
   // Example: If receiver MAC is A4:CF:12:34:56:78
   uint8_t receiverAddress[] = {0xA4, 0xCF, 0x12, 0x34, 0x56, 0x78};
   ```

4. **Upload the transmitter code**

5. **Open Serial Monitor** (115200 baud) to verify transmission

---

### Step 4: Verify System Operation

**What you should see:**

**Receiver TFT Display:**

```
┌─────────────────────────────────────────┐
│           TELEMETRY                     │
├─────────────────────────────────────────┤
│ TX: [=====Graph TiltX=====] -12.5°      │ ← Cyan scrolling graph
│                                          │
│                                          │
├─────────────────────────────────────────┤
│ TY: [=====Graph TiltY=====] +8.3°       │ ← Magenta scrolling graph
│                                          │
│                                          │
├─────────────────────────────────────────┤
│ S1: 85/90  S2: 95/90  GX: +2.1  GY: -1.5│ ← Real-time values
│ Rate: 20.0 Hz          Pkts: 1234       │
│ Status: ● CONNECTED                     │ ← Green = good
└─────────────────────────────────────────┘
```

**Transmitter Serial Monitor:**

```
TX→ TiltX:-5.2 TiltY:12.1 S1:78 S2:105 GyroX:1.3 GyroY:-0.8
TX→ TiltX:-5.1 TiltY:12.0 S1:79 S2:104 GyroX:1.2 GyroY:-0.9
...
```

**Physical Behavior:**

- Tilt the MPU6050 sensor
- Servos should respond smoothly with PID control
- TFT display graphs should update in real-time
- Connection indicator should be green

---

## 🎨 Display Legend

| Element      | Color        | Meaning                         |
| ------------ | ------------ | ------------------------------- |
| **TX**       | Cyan         | Tilt X (Roll) angle in degrees  |
| **TY**       | Magenta      | Tilt Y (Pitch) angle in degrees |
| **S1/S2**    | Green/Yellow | Servo positions: Actual/Desired |
| **GX/GY**    | Orange       | Gyroscope rates (°/s)           |
| **Rate**     | Green        | Data update frequency (Hz)      |
| **Status ●** | Green        | Connected / Red = Disconnected  |

---

## 🔧 Troubleshooting

### Problem: Receiver shows "WAITING..." / No connection

**Solutions:**

1. **Check MAC address** in transmitter code - most common issue!
2. **Verify both ESP32s are powered on** and code is uploaded
3. **Check distance** - keep within 10m for initial testing
4. **Re-upload receiver code** and get fresh MAC address
5. **Check Serial Monitor** on transmitter for error messages

### Problem: TFT display is blank or garbled

**Solutions:**

1. **Verify wiring** - double-check all TFT connections
2. **Check power** - ensure 3.3V is stable
3. **Try different SPI speed** - add this in receiver setup():
   ```cpp
   tft.begin(20000000);  // 20MHz instead of default
   ```
4. **Test with example sketch** from Adafruit_ILI9341 library first

### Problem: Servos not responding or jittery

**Solutions:**

1. **Check external 5V power** - servos MUST have dedicated supply
2. **Common ground** - ensure ESP32 GND and servo power GND are connected
3. **Check MPU6050** - verify it's responding (transmitter serial monitor)
4. **Calibrate MPU6050** - place flat and level during startup

### Problem: "ESP-NOW Init Failed" message

**Solutions:**

1. **Update ESP32 board package** to latest version (v3.0.0+)
2. **Check WiFi region** - some boards have restrictions
3. **Try different ESP32 board** - clone boards may have issues
4. **Re-flash ESP32** with complete erase:
   ```
   esptool.py --chip esp32 erase_flash
   ```

### Problem: Compilation error "invalid conversion... esp_now_recv_cb_t"

**This means you're using an old ESP32 Arduino Core version (v2.x).**

**Solutions:**

1. **Update ESP32 board package** (RECOMMENDED):

   - Tools → Board → Boards Manager
   - Search "ESP32"
   - Update to version 3.0.0 or newer

2. **OR modify receiver code** if you must use v2.x:
   - In `receiver_esp32.ino`, find line ~109
   - Change from:
     ```cpp
     void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
     ```
   - To:
     ```cpp
     void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
     ```

### Problem: Low update rate (< 15 Hz)

**Solutions:**

1. **Reduce distance** between ESP32s
2. **Check for interference** - keep away from WiFi routers
3. **Reduce serial prints** in transmitter code for faster loop
4. **Use better antenna** or external antenna on ESP32

### Problem: MAC Address shows as 00:00:00:00:00:00

**This means WiFi wasn't properly initialized before reading MAC.**

**Solutions:**

1. **Already fixed in updated code** - includes `WiFi.disconnect()` and `delay(100)`
2. **If still seeing zeros:**
   - Press the **RESET button** on the ESP32
   - Re-upload the code
   - Try a different USB cable (some cables are power-only)
3. **Check if ESP32 is genuine** - clone boards sometimes have MAC issues
4. **Verify ESP32 board selection** in Arduino IDE (should be "ESP32 Dev Module")

---

## ⚙️ Advanced Configuration

### Adjusting Telemetry Rate

In `transmitter_esp32.ino`, modify this line:

```cpp
const unsigned long TELEMETRY_INTERVAL = 50;  // 50ms = 20Hz
```

Options:

- `25` = 40Hz (faster, more CPU usage)
- `50` = 20Hz (recommended, good balance)
- `100` = 10Hz (slower, less network traffic)

### Customizing Display Colors

In `receiver_esp32.ino`, modify these defines:

```cpp
#define COLOR_TILTX    0x07FF  // Cyan
#define COLOR_TILTY    0xF81F  // Magenta
// ... etc
```

Use [RGB565 color picker](http://www.barth-dev.de/online/rgb565-color-picker/) for custom colors.

### PID Tuning

See `PID_TUNING_QUICK_REFERENCE.md` for detailed PID tuning instructions.

Quick adjustments in `transmitter_esp32.ino`:

```cpp
float Kp_servo1 = 45.0;  // ↑ for faster response
float Ki_servo1 = 5.0;   // ↑ to eliminate steady-state error
float Kd_servo1 = 8.0;   // ↑ to reduce overshoot
```

---

## 📊 Performance Specifications

| Parameter         | Value                                    |
| ----------------- | ---------------------------------------- |
| Update Rate       | 20 Hz (50ms interval)                    |
| Latency           | < 20ms typical                           |
| Range             | Up to 250m (line of sight)               |
| Data Packet Size  | 40 bytes                                 |
| Display Refresh   | Real-time (on data receive)              |
| Power Consumption | ~200mA (transmitter) + ~150mA (receiver) |

---

## 📝 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    TRANSMITTER ESP32                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  MPU6050 ──► Low-Pass ──► Tilt Calc ──► PID ──► Servos    │
│   (I2C)       Filter       (Angles)     Control    (PWM)   │
│                               │                             │
│                               ▼                             │
│                          Telemetry                          │
│                            Packet                           │
│                               │                             │
│                               ▼                             │
│                          ESP-NOW TX                         │
└─────────────────────────────┬───────────────────────────────┘
                              │
                    ══════════╧══════════  Wireless (2.4GHz)
                              │
┌─────────────────────────────▼───────────────────────────────┐
│                     RECEIVER ESP32                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ESP-NOW RX ──► Parse Data ──► Update ──► ILI9341 TFT     │
│                                 Graphs      Display (SPI)   │
│                                                             │
│  • Real-time scrolling graphs                              │
│  • Numerical value display                                 │
│  • Connection status indicator                             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 🔒 Security Notes

- ESP-NOW transmissions are **unencrypted** by default
- For encrypted communication, enable `peerInfo.encrypt = true` and set encryption keys
- This basic implementation prioritizes speed over security
- Suitable for hobbyist projects, not critical applications

---

## 📚 Additional Resources

- [ESP-NOW Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [ESP32Servo Library](https://github.com/madhephaestus/ESP32Servo)
- [Adafruit ILI9341 Guide](https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

---

## 🎯 What's Next?

Possible enhancements:

1. **Add SD card logging** on receiver for data recording
2. **Implement auto-calibration** for MPU6050 on startup
3. **Add battery monitoring** and low-battery warnings
4. **Create mobile app** for remote monitoring
5. **Add multiple transmitters** to one receiver (sensor network)
6. **Implement bi-directional communication** (send commands back to transmitter)

---

## 📄 License & Credits

- Original PID implementation inspired by dRehmFlight
- ESP-NOW examples from Espressif
- Display code adapted from Adafruit libraries

---

**Questions or Issues?**
Check the troubleshooting section or refer to the documentation files in this directory.

Happy Flying! 🚁
