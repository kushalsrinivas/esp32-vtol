# Receiver Architecture - Deep Dive

## Table of Contents

1. [System Overview](#system-overview)
2. [Software Architecture](#software-architecture)
3. [Display System](#display-system)
4. [User Interface](#user-interface)
5. [Data Reception](#data-reception)
6. [Screen Layouts](#screen-layouts)
7. [Performance Analysis](#performance-analysis)

---

## System Overview

### Purpose

The receiver is the **ground station** that displays real-time telemetry from the aircraft. Its main responsibilities are:

1. Receive telemetry data via ESP-NOW
2. Display data on ST7735 LCD screen
3. Provide multi-screen interface for different views
4. Monitor connection status
5. Log data (in some modes)
6. Allow user navigation between screens

### High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                   RECEIVER SYSTEM ARCHITECTURE                   │
└─────────────────────────────────────────────────────────────────┘

      INPUT              PROCESSING             OUTPUT

  ┌──────────┐        ┌──────────┐         ┌──────────┐
  │ Aircraft │        │          │         │ ST7735   │
  │Transmits │        │   ESP32  │         │   LCD    │
  │          │ESP-NOW │          │   SPI   │          │
  │Telemetry ├───────►│  Receive │────────►│ Display  │
  │          │ 2.4GHz │  Decode  │         │          │
  └──────────┘        │  Render  │         └──────────┘
                      │          │
  ┌──────────┐        │  Menu    │
  │ Joystick │ Analog │  System  │
  │ Button   ├───────►│          │
  │          │        └──────────┘
  └──────────┘

```

---

## Software Architecture

### Module Organization

```
┌───────────────────────────────────────────────────────────────┐
│                    SOFTWARE MODULE STRUCTURE                   │
└───────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────┐
│                      APPLICATION LAYER                         │
├───────────────────────────────────────────────────────────────┤
│  - Main Loop (updates display based on screen mode)           │
│  - Menu Navigation Handler                                    │
│  - Connection Monitor                                         │
└───────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌───────────────────────────────────────────────────────────────┐
│                         UI LAYER                              │
├───────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐          │
│  │ Orientation │  │    Gyro     │  │   Accel     │          │
│  │   Screen    │  │   Screen    │  │   Screen    │          │
│  └─────────────┘  └─────────────┘  └─────────────┘          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐          │
│  │   Servo     │  │   Logger    │  │    Menu     │          │
│  │   Screen    │  │   Screen    │  │   Screen    │          │
│  └─────────────┘  └─────────────┘  └─────────────┘          │
└───────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌───────────────────────────────────────────────────────────────┐
│                      GRAPHICS LAYER                           │
├───────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐       │
│  │ Draw Text    │  │ Draw Shapes  │  │ Draw Gauges  │       │
│  │ (Adafruit)   │  │ (Lines, etc) │  │ (Custom)     │       │
│  └──────────────┘  └──────────────┘  └──────────────┘       │
└───────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌───────────────────────────────────────────────────────────────┐
│                      HARDWARE LAYER                           │
├───────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────┐    │
│  │ SPI Bus  │  │ ESP-NOW │  │ ADC     │  │ Timer/Clock │    │
│  │ (Display)│  │ (Radio) │  │(Joystk) │  │  (millis)   │    │
│  └──────────┘  └─────────┘  └─────────┘  └─────────────┘    │
└───────────────────────────────────────────────────────────────┘
```

### File Structure

```
receiver_esp32_st7735.ino
├── Pin Definitions (lines 61-72)
├── Data Structures (lines 73-100)
│   ├── TelemetryData struct
│   └── Display dimensions
├── Display Objects (lines 92-100)
│   └── Adafruit_ST7735 tft
├── Global Variables (lines 102-130)
│   ├── Connection status
│   ├── Screen mode
│   ├── Menu state
│   └── Update rates
├── Callback Functions (lines 132-150)
│   └── ESP-NOW receive callback
├── Setup Function (lines 152-250)
│   └── Initialize display, WiFi, ESP-NOW
├── Main Loop (lines 252-280)
│   └── Handle joystick, update screen
├── Screen Drawing Functions (lines 282-900)
│   ├── drawOrientationScreen()
│   ├── drawGyroScreen()
│   ├── drawAccelScreen()
│   ├── drawServoScreen()
│   ├── drawLoggerScreen()
│   └── drawMenuScreen()
└── Helper Functions (lines 902-1043)
    ├── drawConnectionStatus()
    ├── readJoystick()
    └── Various drawing utilities
```

---

## Display System

### ST7735 LCD Specifications

```
┌─────────────────────────────────────────────────────────────────┐
│                     ST7735 DISPLAY DETAILS                       │
└─────────────────────────────────────────────────────────────────┘

Physical:
- Size: 1.8" diagonal
- Resolution: 128 x 160 pixels
- Color depth: 18-bit (262,144 colors)
- Pixel size: 0.22mm x 0.22mm

Orientation Modes:
┌────────────────┬──────────┬──────────┐
│ Mode           │ Width    │ Height   │
├────────────────┼──────────┼──────────┤
│ Portrait       │ 128 px   │ 160 px   │
│ Landscape      │ 160 px   │ 128 px   │ ← We use this
└────────────────┴──────────┴──────────┘

Controller: ST7735R chip
Interface: SPI (4-wire + control lines)
```

### SPI Communication

```
SPI Configuration:
- Clock speed: 40 MHz (very fast!)
- Mode: SPI_MODE0 (CPOL=0, CPHA=0)
- Bit order: MSB first
- Data width: 8-bit

Pin assignments:
┌──────────┬──────────┬────────────────────────┐
│ Signal   │ ESP32    │ Purpose                │
├──────────┼──────────┼────────────────────────┤
│ SCK      │ GPIO 18  │ SPI clock              │
│ MOSI     │ GPIO 23  │ Data (Master → Slave)  │
│ CS       │ GPIO 5   │ Chip select (active L) │
│ DC       │ GPIO 21  │ Data/Command select    │
│ RST      │ GPIO 4   │ Reset (active LOW)     │
└──────────┴──────────┴────────────────────────┘

Communication Protocol:
1. Assert CS (LOW) → Select display
2. Set DC:
   - LOW → Send command
   - HIGH → Send data
3. Clock out 8 bits on MOSI
4. Deassert CS (HIGH) → End transaction

Example: Set pixel color
  CS = LOW
  DC = LOW,  send: 0x2C (RAMWR command)
  DC = HIGH, send: R, G, B (pixel data)
  CS = HIGH
```

### Display Memory

```
Frame Buffer:
- Location: On ST7735 chip (not in ESP32 RAM!)
- Size: 128 × 160 × 18-bit = 46,080 bytes
- Format: RGB666 (6 bits per color channel)

No ESP32 frame buffer:
- Saves 46 KB of RAM
- Drawback: Can't read back pixels
- Must redraw entire screen for updates
```

### Color System

```cpp
// 16-bit RGB565 format (used by library):
// RRRRRGGGGGGBBBBB (5-6-5 bits)

Color Palette:
ST7735_BLACK    = 0x0000  // RGB(0, 0, 0)
ST7735_WHITE    = 0xFFFF  // RGB(255, 255, 255)
ST7735_RED      = 0xF800  // RGB(255, 0, 0)
ST7735_GREEN    = 0x07E0  // RGB(0, 255, 0)
ST7735_BLUE     = 0x001F  // RGB(0, 0, 255)
ST7735_CYAN     = 0x07FF  // RGB(0, 255, 255)
ST7735_MAGENTA  = 0xF81F  // RGB(255, 0, 255)
ST7735_YELLOW   = 0xFFE0  // RGB(255, 255, 0)
ST7735_ORANGE   = 0xFC00  // RGB(255, 128, 0)

Custom colors:
uint16_t myColor = ((R & 0xF8) << 8) |
                   ((G & 0xFC) << 3) |
                   (B >> 3);
```

### Drawing Primitives

```cpp
// Adafruit_GFX library provides:

// Basic shapes:
tft.drawPixel(x, y, color);
tft.drawLine(x0, y0, x1, y1, color);
tft.drawRect(x, y, w, h, color);
tft.fillRect(x, y, w, h, color);
tft.drawCircle(x, y, r, color);
tft.fillCircle(x, y, r, color);
tft.drawTriangle(x0,y0, x1,y1, x2,y2, color);
tft.fillTriangle(x0,y0, x1,y1, x2,y2, color);

// Text:
tft.setCursor(x, y);
tft.setTextColor(color);
tft.setTextSize(size);  // 1-5 (pixel multiplication)
tft.print("Hello");
tft.println(value);

// Screen:
tft.fillScreen(color);  // Clear entire screen
tft.setRotation(rot);   // 0-3 (portrait/landscape)
```

### Text Rendering

```
Font Sizes (built-in Adafruit font):
┌──────┬────────┬────────┬─────────────────┐
│ Size │ Width  │ Height │ Characters/line │
├──────┼────────┼────────┼─────────────────┤
│  1   │ 6 px   │ 8 px   │ 26 chars        │
│  2   │ 12 px  │ 16 px  │ 13 chars        │
│  3   │ 18 px  │ 24 px  │ 8 chars         │
│  4   │ 24 px  │ 32 px  │ 6 chars         │
│  5   │ 30 px  │ 40 px  │ 5 chars         │
└──────┴────────┴────────┴─────────────────┘

(Based on 160px width in landscape mode)

We use:
- Size 1: Labels, small text
- Size 2: Main data values
- Size 3: Large headings (rare, too big for 160px)
```

---

## User Interface

### Screen Modes

The receiver has **6 different screens**:

```
┌────────────────────────────────────────────────────────────┐
│                    SCREEN NAVIGATION                        │
└────────────────────────────────────────────────────────────┘

        ┌──────────────┐
        │   MENU       │ ← Start here
        └──────┬───────┘
               │
       ┌───────┴───────┬──────────┬──────────┬──────────┐
       ▼               ▼          ▼          ▼          ▼
┌────────────┐  ┌───────────┐ ┌────────┐ ┌────────┐ ┌────────┐
│Orientation │  │   Gyro    │ │ Accel  │ │ Servo  │ │ Logger │
│   View     │  │   View    │ │  View  │ │  View  │ │  View  │
└────────────┘  └───────────┘ └────────┘ └────────┘ └────────┘

Navigation: Use joystick UP/DOWN to select, CENTER to activate
```

### Joystick Interface

````
5-Way Joystick Module:
         UP
          │
    LEFT ─┼─ RIGHT
          │
        DOWN

    CENTER (button press)

Hardware:
- VRX, VRY: Analog potentiometers (0-4095 ADC value)
- SW: Digital button (active LOW)
- Center position: ~2048 (12-bit ADC)
- Edges: 0 (one side), 4095 (other side)

Software Implementation:
```cpp
int vrx = analogRead(JOYSTICK_VRX);  // 0-4095
int vry = analogRead(JOYSTICK_VRY);

// Direction detection:
if (vry < 1500) direction = UP;
else if (vry > 2500) direction = DOWN;
else if (vrx < 1500) direction = LEFT;
else if (vrx > 2500) direction = RIGHT;

bool pressed = (digitalRead(JOYSTICK_SW) == LOW);
````

### Menu System

```
Menu Structure:

┌────────────────────────────────────────┐
│         TELEMETRY RECEIVER             │
│                                        │
│  ►  Orientation View                   │
│     Gyro View                          │
│     Accelerometer View                 │
│     Servo View                         │
│     Data Logger                        │
│                                        │
│  Use ↑↓ to navigate, press to select  │
└────────────────────────────────────────┘

Selected item: Orange text with ► marker
Other items: White text
Navigation: UP/DOWN moves selection
Action: CENTER button activates selected screen
```

---

## Data Reception

### ESP-NOW Receive Process

```
┌─────────────────────────────────────────────────────────────────┐
│                  ESP-NOW RECEIVE PIPELINE                        │
└─────────────────────────────────────────────────────────────────┘

1. RADIO HARDWARE
   - 2.4 GHz receiver (ESP32 WiFi module)
   - Packet arrives over-the-air
   - Hardware CRC check
   │
   ▼
2. ESP-NOW STACK (Core 1, background)
   - Packet decryption (if enabled, we don't use)
   - MAC address filtering
   - Callback triggered
   │
   ▼
3. OnDataRecv() CALLBACK
   - Runs on Core 1 (interrupt context)
   - Copies data to receivedData struct
   - Sets dataReceived flag
   - Updates lastDataTime
   │
   ▼
4. MAIN LOOP (Core 0)
   - Checks dataReceived flag
   - Displays data on screen
   - Clears flag

Latency: ~3-5ms from TX to RX callback
```

### Callback Implementation

```cpp
void OnDataRecv(const esp_now_recv_info_t *recv_info,
                const uint8_t *incomingData, int len)
{
    // This runs in INTERRUPT context!
    // Keep it SHORT and FAST

    // Copy received data
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // Set flag for main loop
    dataReceived = true;
    lastDataTime = millis();

    // Update counter
    packetsReceived++;

    // NO Serial.print() here! (too slow)
    // NO heavy processing! (blocks radio)
}
```

### Connection Monitoring

````
Connection Status Logic:

Connected: Received packet within last 1000ms
Disconnected: No packet for > 1000ms

Visual Indicator:
┌──────────────────────────────────────┐
│ ●CONN  50Hz                          │ ← Green dot = connected
└──────────────────────────────────────┘

┌──────────────────────────────────────┐
│ ●LOST  --Hz                          │ ← Red dot = disconnected
└──────────────────────────────────────┘

Code:
```cpp
unsigned long timeSinceLastData = millis() - lastDataTime;
bool connected = (timeSinceLastData < 1000);

if (connected) {
    tft.fillCircle(10, 10, 4, ST7735_GREEN);
    tft.print("CONN");
} else {
    tft.fillCircle(10, 10, 4, ST7735_RED);
    tft.print("LOST");
}
````

### Data Rate Calculation

```cpp
// Calculate packets per second:
unsigned long now = millis();
if (now - lastRateCalc >= 1000) {  // Every 1 second
    dataRate = packetsReceived - lastPacketCount;
    lastPacketCount = packetsReceived;
    lastRateCalc = now;
}

// Display:
tft.print(dataRate);
tft.print("Hz");

// Typical values:
// Good connection: 45-50 Hz
// Marginal: 20-40 Hz
// Poor: < 20 Hz
```

---

## Screen Layouts

### 1. Orientation Screen

```
┌────────────────────────────────────────┐
│ ●CONN  50Hz           [ORIENTATION]    │
│                                        │
│  PITCH: 12.5°                          │
│  ┌────────────┐                        │
│  │     ▓▓     │  Graphical indicator   │
│  │   ▓▓▓▓▓▓   │  (pitch angle bar)     │
│  │ ▓▓▓▓▓▓▓▓▓▓ │                        │
│  └────────────┘                        │
│                                        │
│  ROLL: -5.2°                           │
│  ┌────────────┐                        │
│  │ ▓▓▓▓▓▓▓▓   │  Graphical indicator   │
│  │   ▓▓▓▓▓▓   │  (roll angle bar)      │
│  │     ▓▓     │                        │
│  └────────────┘                        │
│                                        │
│  Press for Menu                        │
└────────────────────────────────────────┘

Features:
- Large text for angles
- Visual bars (like old attitude indicator)
- Color coding: Cyan bars
- Real-time update
```

### 2. Gyro Screen

```
┌────────────────────────────────────────┐
│ ●CONN  50Hz              [GYRO]        │
│                                        │
│  GYRO PITCH:                           │
│    12.5 °/s                            │
│                                        │
│  GYRO ROLL:                            │
│    -3.2 °/s                            │
│                                        │
│  GYRO YAW:                             │
│    0.8 °/s                             │
│                                        │
│  (Angular velocities)                  │
│                                        │
│  Press for Menu                        │
└────────────────────────────────────────┘

Features:
- Shows rotation rates
- Useful for tuning D term
- Color: Cyan for values
```

### 3. Accelerometer Screen

```
┌────────────────────────────────────────┐
│ ●CONN  50Hz             [ACCEL]        │
│                                        │
│  ACCEL X:                              │
│    0.15 g                              │
│                                        │
│  ACCEL Y:                              │
│    0.03 g                              │
│                                        │
│  ACCEL Z:                              │
│    1.02 g                              │
│                                        │
│  (Raw accelerations)                   │
│                                        │
│  Press for Menu                        │
└────────────────────────────────────────┘

Features:
- Shows linear accelerations
- Z ≈ 1.0g when level (gravity)
- Useful for vibration analysis
```

### 4. Servo Screen

```
┌────────────────────────────────────────┐
│ ●CONN  50Hz             [SERVO]        │
│                                        │
│  RIGHT ELEVON:                         │
│    Des:   90°                          │
│    Act:   92°                          │
│                                        │
│  LEFT ELEVON:                          │
│    Des:   90°                          │
│    Act:   88°                          │
│                                        │
│  (Elevon positions)                    │
│                                        │
│  Press for Menu                        │
└────────────────────────────────────────┘

Features:
- Desired vs Actual positions
- Helps debug mixing
- Color: Green for actual
```

### 5. Data Logger Screen

```
┌────────────────────────────────────────┐
│ ●CONN  50Hz            [LOGGER]        │
│                                        │
│  Logging: ACTIVE                       │
│  Packets: 12458                        │
│                                        │
│  P: 5.2° R:-2.1° GY:1.2                │
│  P: 5.1° R:-2.0° GY:1.1                │
│  P: 5.0° R:-1.9° GY:1.0                │
│  P: 4.9° R:-1.8° GY:0.9                │
│  P: 4.8° R:-1.7° GY:0.8                │
│                                        │
│  (Scrolling log)                       │
│                                        │
│  Press for Menu                        │
└────────────────────────────────────────┘

Features:
- Scrolling text log
- Compact format (fits more data)
- Good for recording flights
```

### 6. Menu Screen

See [Menu System](#menu-system) section above.

---

## Screen Update Strategy

### Partial Update vs Full Redraw

````
Problem: Full screen redraw is SLOW (50-100ms)
- Would cause flicker
- Limits update rate to ~10 Hz

Solution: Partial updates
- Only redraw changed values
- Keep static elements (labels, borders)

Implementation:
```cpp
// First draw (full):
void drawOrientationScreen() {
    if (firstDraw) {
        tft.fillScreen(ST7735_BLACK);  // Clear all
        tft.setTextColor(ST7735_WHITE);
        tft.setCursor(10, 30);
        tft.print("PITCH:");  // Static label
        firstDraw = false;
    }

    // Every frame (partial):
    tft.fillRect(70, 30, 80, 16, ST7735_BLACK);  // Erase old value
    tft.setCursor(70, 30);
    tft.setTextColor(ST7735_CYAN);
    tft.print(pitch, 1);  // Draw new value
}
````

### Timing Analysis

```
Screen Update Performance:

Full redraw:
  fillScreen()      20 ms
  Draw labels       10 ms
  Draw values       5 ms
  Draw graphics     15 ms
  ────────────────────────
  Total:            50 ms  (20 Hz max)

Partial update:
  Erase old values  2 ms
  Draw new values   5 ms
  ────────────────────────
  Total:            7 ms   (140 Hz capable!)

Actual update rate: Limited by data rate (50 Hz)
Screen refresh: Also 50 Hz (matches data rate)
```

### Anti-Flicker Techniques

```
1. Don't clear entire screen
   ❌ tft.fillScreen(BLACK);  // Every frame
   ✓  Clear only changed regions

2. Use bounding boxes
   tft.fillRect(x, y, width, height, BLACK);
   tft.setCursor(x, y);
   tft.print(newValue);

3. Draw background color text (not used, slower)
   tft.setTextColor(CYAN, BLACK);  // Text + background
   tft.print(value);  // Erases old automatically

4. Double-buffering (not possible on ST7735)
   - Would need 46KB RAM for frame buffer
   - ESP32 doesn't have enough spare RAM
```

---

## Performance Analysis

### CPU Load

```
ESP32 Dual-core:
Core 0: Main application, display updates
Core 1: WiFi/ESP-NOW stack

Core 0 utilization:
  Screen refresh: 7 ms every 20 ms = 35%
  Joystick read: < 1 ms = < 5%
  Idle: ~60%

Core 1 utilization:
  ESP-NOW receive: ~10-15% (background)

Overall system load: ~50% (comfortable)
```

### Memory Usage

```
RAM Usage:
┌─────────────────────┬─────────┐
│ Component           │ Size    │
├─────────────────────┼─────────┤
│ TelemetryData       │ 48 B    │
│ Display object      │ ~2 KB   │
│ WiFi/ESP-NOW stack  │ ~60 KB  │
│ Adafruit_GFX        │ ~5 KB   │
│ Global variables    │ ~200 B  │
│ Stack               │ ~8 KB   │
├─────────────────────┼─────────┤
│ Total               │ ~75 KB  │
│ Available           │ ~255 KB │
└─────────────────────┴─────────┘

Memory usage: 23% (plenty of room)
```

### Display Bandwidth

```
SPI Clock: 40 MHz
Bits per pixel: 16 (RGB565)
Screen size: 160 × 128 = 20,480 pixels

Full screen transfer:
  20,480 px × 16 bits = 327,680 bits
  327,680 bits ÷ 40 MHz = 8.2 ms

Theoretical max FPS: 122 Hz
Practical: ~70 Hz (with overhead)
Our rate: 50 Hz (data-limited) ✓
```

### Latency Budget

```
Transmitter sends → Receiver displays

TX: Sensor read      1 ms
TX: Processing       0.5 ms
TX: ESP-NOW send     3 ms
────────────────────────────
Air time             4.5 ms

RX: ESP-NOW recv     0.1 ms
RX: Callback         0.05 ms
RX: Main loop        0-20 ms (worst case: waiting for prev frame)
RX: Screen update    7 ms
────────────────────────────
Display time         27 ms

Total latency: 4.5 + 27 = 31.5 ms (typical)
Worst case: 4.5 + 20 + 7 = 51.5 ms

This is acceptable for telemetry display!
(Human reaction time: ~200ms)
```

---

## Error Handling

### Connection Loss

```
Scenario: Transmitter goes out of range or powers off

Detection:
  if (millis() - lastDataTime > 1000) {
      // Connection lost!
  }

Response:
- Update status indicator (red dot)
- Show "LOST" text
- Data rate shows "--Hz"
- Values on screen freeze (last known value)
- No error message (to avoid clutter)

Recovery:
- Automatic when packets resume
- No user intervention needed
```

### Invalid Data

```
Scenario: Corrupted packet (rare with ESP-NOW CRC)

Detection:
- ESP-NOW has built-in CRC check
- Corrupted packets are discarded
- We don't receive callback

Response:
- Packet simply not counted
- Previous data still displayed
- Connection timeout triggers if persistent

No additional checking needed!
```

### Display Errors

```
Scenario: SPI communication failure (very rare)

Symptoms:
- Garbled display
- No response to commands
- Random colors

Response (current):
- System continues (may look bad)
- No automatic recovery

Improvement (future):
- Add init check in setup()
- Detect SPI timeouts
- Attempt re-initialization
```

---

## Configuration Options

### Compile-Time Options

```cpp
// Display orientation:
tft.setRotation(1);  // 0-3 (portrait/landscape)

// Screen dimensions (must match rotation):
#define SCREEN_WIDTH 160   // X-axis
#define SCREEN_HEIGHT 128  // Y-axis

// Joystick thresholds:
#define THRESHOLD_LOW 1500   // Trigger UP/LEFT
#define THRESHOLD_HIGH 2500  // Trigger DOWN/RIGHT

// Connection timeout:
#define TIMEOUT_MS 1000  // Consider lost after 1 sec

// Update rates:
// (No defines, but could add)
#define DISPLAY_UPDATE_MS 20  // 50 Hz
```

### Runtime Configuration

```
Currently: No runtime config (no EEPROM, no menu settings)

Could add:
- Screen brightness (adjust backlight)
- Color schemes (themes)
- Auto-rotate screens
- Data logging enable/disable
- Alert thresholds
```

---

## Future Enhancements

### Possible Improvements

1. **Graphical Instruments**

   - Artificial horizon (pitch/roll)
   - Circular gauges (like analog instruments)
   - Bar graphs for servo positions
   - Trend lines (history plots)

2. **Data Logging to SD Card**

   - Add SD card module
   - Log to CSV file
   - Replay logged flights
   - Export to computer

3. **Alerts and Warnings**

   - Low battery warning (if voltage sent)
   - Excessive tilt alarm
   - High gyro rate warning
   - Servo saturation indicator

4. **Multiple Transmitters**

   - Switch between aircraft
   - Show TX ID on screen
   - Store separate settings per TX

5. **Touchscreen**

   - Replace joystick with touch
   - Easier menu navigation
   - Interactive controls
   - On-screen buttons

6. **Better Graphics**
   - Use TFT_eSPI library (faster)
   - Smooth animations
   - Fade transitions between screens
   - Anti-aliased text

---

## Debugging Tips

### Serial Monitor Output

```cpp
// Enable verbose debugging:
#define DEBUG_MODE

#ifdef DEBUG_MODE
    Serial.printf("Received: P=%.1f R=%.1f\n", pitch, roll);
#endif
```

### Common Issues

| Problem          | Cause                | Solution                     |
| ---------------- | -------------------- | ---------------------------- |
| Blank screen     | Wrong wiring or init | Check SPI pins, power        |
| White screen     | No init command      | Check tft.initR() call       |
| Garbled display  | Wrong rotation       | Try tft.setRotation(0-3)     |
| No data received | Wrong MAC address    | Verify TX has correct RX MAC |
| Slow updates     | SPI speed too low    | Check library settings       |
| Flickering       | Full redraws         | Use partial updates          |

### Performance Monitoring

```cpp
// Add timing measurements:
unsigned long startTime = millis();
drawOrientationScreen();
unsigned long elapsed = millis() - startTime;
Serial.printf("Draw time: %lu ms\n", elapsed);
```

---

## Summary

The receiver is a **real-time telemetry display system** that:

- Receives 50 Hz data via ESP-NOW
- Displays on 160×128 pixel color LCD
- Provides 6 different view screens
- Has menu-based navigation
- Monitors connection status
- Updates display at 50 Hz

**Key strengths:**

- ✅ Low latency (30-50ms)
- ✅ Fast SPI (40 MHz)
- ✅ Multiple view modes
- ✅ User-friendly interface
- ✅ Reliable connection monitoring

**Potential improvements:**

- ⚠️ Could add data logging
- ⚠️ Could add graphical instruments
- ⚠️ Could add alerts/warnings
- ⚠️ Could support touchscreen

---

**END OF RECEIVER ARCHITECTURE**
