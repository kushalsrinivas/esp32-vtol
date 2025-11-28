# Hardware Wiring and Circuit Diagrams

## Table of Contents
1. [System Overview](#system-overview)
2. [Transmitter Circuit](#transmitter-circuit)
3. [Receiver Circuit](#receiver-circuit)
4. [Component Specifications](#component-specifications)
5. [Power Distribution](#power-distribution)
6. [Signal Levels](#signal-levels)
7. [Wiring Best Practices](#wiring-best-practices)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                      COMPLETE SYSTEM DIAGRAM                     │
└─────────────────────────────────────────────────────────────────┘

                    TRANSMITTER                    RECEIVER
                 (On Aircraft)              (Ground Station)
                                                    
┌──────────────────────────────┐         ┌──────────────────────────┐
│         ESP32 #1             │         │        ESP32 #2          │
│    (Transmitter Module)      │         │   (Receiver Module)      │
│                              │         │                          │
│  ┌────────┐                 │         │                          │
│  │MPU6050 │---I2C---[GPIO]  │         │  [GPIO]---ST7735 LCD     │
│  │Sensor  │         [21/22] │         │  [Multiple pins]         │
│  └────────┘                 │         │                          │
│                              │         │                          │
│            [GPIO25]---Servo1 │         │                          │
│            [GPIO26]---Servo2 │         │                          │
│                              │         │                          │
│           [WiFi/ESP-NOW]     │◄───────►│  [WiFi/ESP-NOW]          │
│                              │ 2.4GHz  │                          │
│  [3.3V]  [GND]  [5V Servo]  │         │  [5V USB]  [GND]         │
└──────────────────────────────┘         └──────────────────────────┘
       │       │        │                       │       │
       │       │        │                       │       │
    3.7V Li  GND   5V BEC                   USB Power  GND
     Battery         (for servos)
```

---

## Transmitter Circuit

### Full Schematic

```
┌────────────────────────────────────────────────────────────────────┐
│                    TRANSMITTER ESP32 CIRCUIT                        │
└────────────────────────────────────────────────────────────────────┘

                        ESP32 DEVKIT (30-pin)
                    ┌─────────────────────────┐
                    │                         │
         MPU6050    │  3V3 ───┐               │
           ┌────────┼─ GND    │               │
           │        │  EN     │               │
           │        │  VP     ├ COMMON        │
           │        │  VN     │  GROUND       │    SERVO POWER
           │        │  34     │  BUS          │    SUPPLY
           │        │  35     │               │    ┌──────┐
           │        │  32     │               │    │ 5V   │
           │        │  33     │               │    │ BEC  │
┌──────┐  │        │  25 ────┼───────────────┼────┤ OUT  │
│ VCC  ├──┘        │  26 ────┼──────────┐    │    └──────┘
│      │           │  27     │          │    │       │
│ GND  ├───────────┼─────────┘          │    │       │
│      │           │  14                │    │       │
│ SCL  ├───────────┼─ 22 (SCL)         │    │       │
│      │           │  21 (SDA)          │    │       │
│ SDA  ├───────────┼──┘                 │    │       │
│      │           │  TX                │    │       │
│ AD0  ├─(to GND)  │  RX                │    │       │
│      │           │  GND ───┬──────────┼────┼───────┤
└──────┘           │  ───────┤          │    │       │
 MPU6050           │  ───────┤          │    │       │
                   │         │          │    │       │
                   └─────────┼──────────┼────┼───────┘
                             │          │    │
                            GND         │    │
                                        │    │
                         ┌──────────────┘    │
                         │                   │
                    ┌────▼────┐         ┌────▼────┐
                    │  SERVO  │         │  SERVO  │
                    │  RIGHT  │         │  LEFT   │
                    │ ELEVON  │         │ ELEVON  │
                    └─────────┘         └─────────┘
                      GPIO 25            GPIO 26
                    (PWM Signal)       (PWM Signal)
                    
                    Servo Wiring:
                    ├─ Brown/Black: GND (common bus)
                    ├─ Red: 5V (from BEC)
                    └─ Orange/Yellow: Signal (from ESP32)
```

### Pin Mapping Table (Transmitter)

| Component | Pin Name | ESP32 GPIO | Pin Type | Voltage | Purpose |
|-----------|----------|------------|----------|---------|---------|
| **MPU6050** | VCC | 3.3V | Power | 3.3V | Sensor power |
| | GND | GND | Ground | 0V | Common ground |
| | SCL | GPIO 22 | I2C Clock | 3.3V | I2C clock line |
| | SDA | GPIO 21 | I2C Data | 3.3V | I2C data line |
| | AD0 | GND | Address | 0V | I2C addr = 0x68 |
| **Right Elevon** | Signal | GPIO 25 | PWM Output | 3.3V | Servo control |
| | Power | 5V BEC | Power | 5.0V | Servo motor power |
| | Ground | GND | Ground | 0V | Common ground |
| **Left Elevon** | Signal | GPIO 26 | PWM Output | 3.3V | Servo control |
| | Power | 5V BEC | Power | 5.0V | Servo motor power |
| | Ground | GND | Ground | 0V | Common ground |
| **ESP32 Power** | VIN | 5V or USB | Power | 5.0V | ESP32 main power |
| | GND | GND | Ground | 0V | Common ground |

### Detailed MPU6050 Connection

```
MPU6050 Breakout Board          ESP32 DevKit
┌───────────────────┐           ┌──────────┐
│                   │           │          │
│  [VCC]  ●─────────┼───────────┤ 3.3V     │  (3.3V regulated)
│  [GND]  ●─────────┼───────────┤ GND      │  (Common ground)
│  [SCL]  ●─────────┼───────────┤ GPIO 22  │  (I2C Clock - 100kHz)
│  [SDA]  ●─────────┼───────────┤ GPIO 21  │  (I2C Data)
│  [XDA]  ●         │           │          │  (Leave unconnected)
│  [XCL]  ●         │           │          │  (Leave unconnected)
│  [AD0]  ●─────────┼───────────┤ GND      │  (Sets I2C addr to 0x68)
│  [INT]  ●         │           │          │  (Leave unconnected)
│                   │           │          │
└───────────────────┘           └──────────┘

I2C Bus Characteristics:
- Protocol: I2C (Two-Wire Interface)
- Clock Speed: 100 kHz (Standard Mode)
- Voltage Level: 3.3V (ESP32 native)
- Pull-up Resistors: Built into MPU6050 breakout (typically 4.7kΩ)
- Address: 0x68 (when AD0 = LOW)
```

### Detailed Servo Connection

```
Right Elevon Servo                 ESP32 + Power
┌──────────────────┐              ┌────────────┐
│                  │              │            │
│  [BROWN/BLACK]   ├──────────────┤ GND        │ ◄── Common ground
│  [RED]           ├──────────┐   │            │
│  [ORANGE/YELLOW] ├──────────┼───┤ GPIO 25    │ ◄── PWM signal
│                  │          │   │            │
└──────────────────┘          │   └────────────┘
                              │
                              │   5V BEC/Regulator
                              │   ┌────────────┐
                              └───┤ +5V OUT    │
                                  │ GND        ├─── Common GND
                                  │ VIN        ├─── Battery (7.4V-12V)
                                  └────────────┘

PWM Signal Characteristics:
- Frequency: 50 Hz (20ms period)
- Pulse Width: 500µs - 2500µs
  * 500µs  = 0° (minimum)
  * 1500µs = 90° (center)
  * 2500µs = 180° (maximum)
- Voltage: 3.3V logic (compatible with 5V servos)
- Our range: 30° to 150° (mechanical limits)
```

---

## Receiver Circuit

### Full Schematic

```
┌────────────────────────────────────────────────────────────────────┐
│                     RECEIVER ESP32 CIRCUIT                          │
└────────────────────────────────────────────────────────────────────┘

                        ESP32 DEVKIT (30-pin)
                    ┌─────────────────────────┐
                    │                         │
         ST7735     │  3V3 ────┐              │
           ┌────────┼─ GND     │              │
           │        │  EN      │              │
           │        │  VP      ├ COMMON       │
           │        │  VN      │  GROUND      │
           │        │  34      │  BUS         │
           │        │  35      │              │
           │        │  32      │              │
           │        │  33      │              │
┌──────┐  │        │  25      │              │
│ VCC  ├──┘        │  26      │              │
│      │           │  27      │              │
│ GND  ├───────────┼──────────┘              │
│      │           │  14      │              │
│ SCL  ├───────────┼─ 18 (SCK)               │
│      │           │  19 (MISO) (unused)     │
│ SDA  ├───────────┼─ 23 (MOSI/SDA)          │
│      │           │  TX                     │
│ CS   ├───────────┼─ 15 (CS)                │
│      │           │  RX                     │
│ RST  ├───────────┼─ 4  (RST)               │
│      │           │  GND ────┬               │
│ DC   ├───────────┼─ 2  (DC) │               │
│      │           │  ───────┐│               │
│ LED  ├───────────┼─ 3V3    ││               │
│      │           │         ││               │
└──────┘           └─────────┼┼───────────────┘
 ST7735                      ││
  LCD                        GND
                    (Backlight to 3.3V)

USB Connection:
┌──────────┐
│   USB    │
│  Cable   ├───► ESP32 VIN (5V)
│          ├───► GND
└──────────┘
```

### Pin Mapping Table (Receiver)

| Component | Pin Name | ESP32 GPIO | Pin Type | Voltage | Purpose |
|-----------|----------|------------|----------|---------|---------|
| **ST7735 LCD** | VCC | 3.3V | Power | 3.3V | Display power |
| | GND | GND | Ground | 0V | Common ground |
| | SCL/SCK | GPIO 18 | SPI Clock | 3.3V | SPI clock |
| | SDA/MOSI | GPIO 23 | SPI Data | 3.3V | SPI data out |
| | CS | GPIO 15 | Chip Select | 3.3V | SPI chip select |
| | RST | GPIO 4 | Reset | 3.3V | Display reset |
| | DC | GPIO 2 | Data/Cmd | 3.3V | Data/command select |
| | LED | 3.3V | Backlight | 3.3V | Backlight power |
| **ESP32 Power** | VIN | USB 5V | Power | 5.0V | ESP32 main power |
| | GND | USB GND | Ground | 0V | Common ground |

### Detailed ST7735 Connection

```
ST7735 LCD Display             ESP32 DevKit
┌───────────────────┐          ┌──────────┐
│                   │          │          │
│  [VCC]  ●─────────┼──────────┤ 3.3V     │  Display power
│  [GND]  ●─────────┼──────────┤ GND      │  Common ground
│  [SCL]  ●─────────┼──────────┤ GPIO 18  │  SPI clock
│  [SDA]  ●─────────┼──────────┤ GPIO 23  │  SPI data (MOSI)
│  [CS]   ●─────────┼──────────┤ GPIO 15  │  Chip select (active LOW)
│  [RST]  ●─────────┼──────────┤ GPIO 4   │  Reset (active LOW)
│  [DC]   ●─────────┼──────────┤ GPIO 2   │  Data(HIGH)/Command(LOW)
│  [LED]  ●─────────┼──────────┤ 3.3V     │  Backlight (always on)
│                   │          │          │
│   128x160 pixels  │          │          │
│   1.8" diagonal   │          │          │
└───────────────────┘          └──────────┘

SPI Bus Characteristics:
- Protocol: SPI (Serial Peripheral Interface)
- Mode: SPI Mode 0 (CPOL=0, CPHA=0)
- Clock Speed: 40 MHz (fast!)
- Bit Order: MSB first
- Data Lines: MOSI only (display doesn't send data back)
- Chip Select: Active LOW (pulled HIGH when not in use)
```

---

## Component Specifications

### ESP32 DevKit V1

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 SPECIFICATIONS                      │
└─────────────────────────────────────────────────────────────┘

Microcontroller: ESP32-WROOM-32
CPU: Dual-core Tensilica Xtensa LX6 @ 240 MHz
RAM: 520 KB SRAM
Flash: 4 MB
WiFi: 802.11 b/g/n (2.4 GHz)
Bluetooth: v4.2 BR/EDR and BLE

GPIO Pins: 30 (some have restrictions)
ADC: 18 channels, 12-bit (0-4095)
DAC: 2 channels, 8-bit
PWM: All GPIO pins (16 channels, 16-bit resolution)
I2C: 2 interfaces (any GPIO via software)
SPI: 4 interfaces (we use VSPI)
UART: 3 interfaces

Operating Voltage: 3.3V (internal)
Input Voltage: 5V via USB or VIN pin
Max Current per GPIO: 12 mA (use caution!)
Max Total Current: 200 mA (all GPIO combined)

Power Consumption:
- Active (WiFi TX): 160-260 mA
- Active (WiFi RX): 80-90 mA
- Modem sleep: 20 mA
- Deep sleep: 10 µA
```

### MPU6050 IMU Sensor

```
┌─────────────────────────────────────────────────────────────┐
│                  MPU6050 SPECIFICATIONS                      │
└─────────────────────────────────────────────────────────────┘

Manufacturer: InvenSense (TDK)
Package: QFN-24 (breakout board available)

Accelerometer:
- 3-axis (X, Y, Z)
- Ranges: ±2g, ±4g, ±8g, ±16g (we use ±2g)
- Resolution: 16-bit (0-65535)
- Scale Factor: 16,384 LSB/g (at ±2g range)
- Noise: 400 µg/√Hz

Gyroscope:
- 3-axis (X, Y, Z)
- Ranges: ±250, ±500, ±1000, ±2000 °/s (we use ±250°/s)
- Resolution: 16-bit
- Scale Factor: 131 LSB/(°/s) (at ±250°/s)
- Noise: 0.005 °/s/√Hz

Temperature Sensor:
- Range: -40°C to +85°C
- Sensitivity: 340 LSB/°C
- Offset: -521 at 35°C

Interface: I2C (up to 400 kHz Fast Mode)
I2C Address: 0x68 (AD0=LOW) or 0x69 (AD0=HIGH)

Operating Voltage: 2.375V - 3.46V (3.3V ideal)
Current Consumption:
- Normal mode: 3.9 mA
- Sleep mode: 10 µA

Digital Motion Processor (DMP):
- Built-in (not used in our code)
- Can compute quaternions, Euler angles
- We do calculations in ESP32 for simplicity
```

### Standard Servo Motors

```
┌─────────────────────────────────────────────────────────────┐
│              STANDARD SERVO SPECIFICATIONS                   │
└─────────────────────────────────────────────────────────────┘

Common Models: SG90, MG90S, Futaba S3003

Operating Voltage: 4.8V - 6V (we use 5V)
Operating Current:
- Idle: 10 mA
- Moving: 100-200 mA
- Stall: 500-800 mA (max)

Control Signal:
- Type: PWM (Pulse Width Modulation)
- Frequency: 50 Hz (20ms period)
- Pulse Width Range:
  * Minimum (0°):   ~500 µs  (may vary)
  * Center (90°):   1500 µs
  * Maximum (180°): ~2500 µs (may vary)

Physical Range: 180° (some servos: 160° or 200°)
Our Limited Range: 30° to 150° (±60° from center)

Speed: ~0.12 sec/60° (at 4.8V)
Torque: 1.8 kg·cm (at 4.8V) - varies by model

Deadband: ±1-2 µs (won't move for tiny changes)
Jitter: Some vibration at hold (depends on quality)

Wire Colors (standard):
- Brown/Black: Ground (GND)
- Red: Power (4.8-6V)
- Orange/Yellow/White: Signal (PWM)
```

### ST7735 LCD Display

```
┌─────────────────────────────────────────────────────────────┐
│                ST7735 LCD SPECIFICATIONS                     │
└─────────────────────────────────────────────────────────────┘

Display Type: TFT LCD (Thin Film Transistor)
Controller: ST7735R or ST7735S chip
Size: 1.8" diagonal (some versions: 1.44")
Resolution: 128 x 160 pixels
Color Depth: 18-bit (262,144 colors)

Interface: SPI (4-wire)
- Supported speeds: up to 40 MHz
- Data lines: MOSI only (no MISO needed)
- Requires: CS, DC, RST control lines

Operating Voltage: 3.3V or 5V (depending on board)
- Logic Level: 3.3V (ESP32 compatible)
- Current: 20-40 mA (display on), 80-100mA (backlight)

Viewing Angle: 60-120° (varies by quality)
Backlight: LED (usually 3 white LEDs in parallel)

Pixel Pitch: 0.22mm x 0.22mm
Active Area: 28.03mm x 35.04mm

Common Uses:
- Arduino projects
- Raspberry Pi displays
- Embedded systems
- Telemetry displays (our use!)
```

---

## Power Distribution

### Transmitter Power System

```
┌──────────────────────────────────────────────────────────────┐
│              TRANSMITTER POWER DISTRIBUTION                   │
└──────────────────────────────────────────────────────────────┘

                    Main Battery
                   (LiPo 2S - 7.4V)
                   ┌─────────────┐
                   │   7.4V      │
                   │   1000mAh   │
                   └──────┬──────┘
                          │
                 ┌────────┴────────┐
                 │                 │
            ┌────▼────┐      ┌─────▼─────┐
            │  5V BEC │      │  ESP32    │
            │ (Servo) │      │ Regulator │
            └────┬────┘      └─────┬─────┘
                 │ 5V              │ 3.3V
                 │                 │
        ┌────────┴──────┐    ┌─────┴──────────┐
        │               │    │                │
    ┌───▼───┐      ┌────▼──┐ │  ┌──────┐     │
    │ RIGHT │      │ LEFT  │ │  │ESP32 │     │
    │ELEVON │      │ELEVON │ │  │ CORE │     │
    └───────┘      └───────┘ │  └──────┘     │
                              │                │
                              │  ┌──────────┐ │
                              └──┤ MPU6050  │◄┘
                                 └──────────┘

Power Budget:
┌─────────────────┬────────┬─────────┬────────────┐
│ Component       │ Voltage│ Current │ Power      │
├─────────────────┼────────┼─────────┼────────────┤
│ ESP32 (active)  │ 3.3V   │ 160 mA  │ 528 mW     │
│ MPU6050         │ 3.3V   │ 4 mA    │ 13 mW      │
│ Right Servo     │ 5.0V   │ 100 mA  │ 500 mW     │
│ Left Servo      │ 5.0V   │ 100 mA  │ 500 mW     │
├─────────────────┼────────┼─────────┼────────────┤
│ TOTAL (typical) │ --     │ ~364 mA │ ~1541 mW   │
│ TOTAL (max)     │ --     │ ~2000mA │ ~14.8 W    │
└─────────────────┴────────┴─────────┴────────────┘

Battery Life (1000mAh @ 7.4V):
- Typical operation: ~2 hours
- Max load (both servos stalled): ~30 minutes

WARNING: Servos can draw up to 800mA each when stalled!
Use adequate BEC (3A+ recommended)
```

### Receiver Power System

```
┌──────────────────────────────────────────────────────────────┐
│               RECEIVER POWER DISTRIBUTION                     │
└──────────────────────────────────────────────────────────────┘

                    USB Cable (5V)
                   ┌─────────────┐
                   │   Computer  │
                   │   or Adapter│
                   └──────┬──────┘
                          │ 5V, 500mA
                          │
                    ┌─────▼──────┐
                    │   ESP32    │
                    │  Regulator │
                    │  (AMS1117) │
                    └─────┬──────┘
                          │ 3.3V
                 ┌────────┴────────┐
                 │                 │
           ┌─────▼─────┐     ┌─────▼──────┐
           │   ESP32   │     │  ST7735    │
           │   CORE    │     │  Display   │
           │           │     │            │
           │  - WiFi   │     │ - LCD      │
           │  - CPU    │     │ - Backlight│
           └───────────┘     └────────────┘

Power Budget:
┌─────────────────┬────────┬─────────┬────────────┐
│ Component       │ Voltage│ Current │ Power      │
├─────────────────┼────────┼─────────┼────────────┤
│ ESP32 (active)  │ 3.3V   │ 160 mA  │ 528 mW     │
│ ST7735 LCD      │ 3.3V   │ 40 mA   │ 132 mW     │
│ LED Backlight   │ 3.3V   │ 80 mA   │ 264 mW     │
├─────────────────┼────────┼─────────┼────────────┤
│ TOTAL           │ --     │ 280 mA  │ 924 mW     │
└─────────────────┴────────┴─────────┴────────────┘

USB Power Budget: 500mA @ 5V = 2.5W available
Our Usage: 280mA @ 5V = 1.4W (56% of budget) ✓ Safe

Can be powered by:
- Computer USB port (500mA)
- USB wall adapter (1A recommended)
- Power bank
```

---

## Signal Levels

### Logic Level Compatibility

```
┌──────────────────────────────────────────────────────────────┐
│                    SIGNAL VOLTAGE LEVELS                      │
└──────────────────────────────────────────────────────────────┘

ESP32 GPIO:
- Output HIGH: 3.3V (typical: 3.2V - 3.3V)
- Output LOW:  0V   (typical: 0V - 0.1V)
- Input threshold HIGH: > 2.0V
- Input threshold LOW:  < 0.8V
- 5V tolerant: NO! (will damage ESP32)

MPU6050 I2C:
- Logic HIGH: > 0.7 × VCC = > 2.31V (at 3.3V supply)
- Logic LOW:  < 0.3 × VCC = < 0.99V (at 3.3V supply)
- Compatible with 3.3V ESP32: YES ✓

Servo PWM Input:
- Logic HIGH: > 2.0V (most servos)
- Logic LOW:  < 0.8V
- Compatible with 3.3V ESP32: YES ✓
- Note: Servos are 5V powered but accept 3.3V logic

ST7735 SPI:
- Logic levels: 3.3V (if using 3.3V board)
- Compatible with ESP32: YES ✓

Voltage Level Summary:
┌──────────────┬──────────┬──────────┬──────────────┐
│ Interface    │ ESP32    │ Sensor   │ Compatible?  │
├──────────────┼──────────┼──────────┼──────────────┤
│ MPU6050 I2C  │ 3.3V     │ 3.3V     │ YES ✓        │
│ Servo PWM    │ 3.3V     │ 5V logic │ YES ✓        │
│ ST7735 SPI   │ 3.3V     │ 3.3V     │ YES ✓        │
└──────────────┴──────────┴──────────┴──────────────┘
```

### PWM Signal Timing

```
Servo PWM Signal (50 Hz):

      20ms period (50 Hz)
├─────────────────────────────┤
                                
    ┌──┐                       ┌──┐
    │  │                       │  │
────┘  └───────────────────────┘  └────
    
    └─┘
   Pulse width determines angle:
   
   500µs  = 0° (minimum)
   1000µs = 45°
   1500µs = 90° (center)
   2000µs = 135°
   2500µs = 180° (maximum)
   
Our constrained range (30° to 150°):
   833µs  ≈ 30° (our minimum)
   1500µs = 90° (center)
   2167µs ≈ 150° (our maximum)

The ESP32Servo library handles this automatically!
```

---

## Wiring Best Practices

### 1. Wire Gauge Selection

```
Current Rating vs Wire Gauge (AWG):
┌──────┬──────────┬─────────────────────┐
│ AWG  │ Diameter │ Max Current         │
├──────┼──────────┼─────────────────────┤
│  22  │ 0.64 mm  │ 0.7 A (signal wire) │
│  20  │ 0.81 mm  │ 1.5 A               │
│  18  │ 1.02 mm  │ 2.5 A (servo power) │
│  16  │ 1.29 mm  │ 6 A                 │
└──────┴──────────┴─────────────────────┘

Recommendations:
- Signal wires (I2C, SPI, PWM): 22-24 AWG
- Servo power wires: 18-20 AWG
- Battery to BEC: 16-18 AWG
```

### 2. Wire Length Limits

```
Signal Type     Max Length    Why?
───────────────────────────────────────────
I2C (100kHz)    1 meter       Capacitance
SPI (40MHz)     10 cm         High speed
PWM Servo       1 meter       Low frequency
Power (5V)      30 cm         Voltage drop

Voltage Drop Calculation:
V_drop = I × R_wire
R_wire = ρ × L / A

For 18 AWG, 30cm, 2A:
R ≈ 0.021 Ω/m × 0.3m = 0.0063 Ω
V_drop = 2A × 0.0063Ω = 0.0126V
(negligible!)
```

### 3. Ground Loops Prevention

```
WRONG (Ground Loop):         RIGHT (Star Ground):

ESP32 ──┬─── MPU6050          ESP32 ──┐
        │                             │
        └─┬─ Servo1                   │
          │                           ├─── GND Point ──┬─ MPU6050
          └─ Servo2                   │                ├─ Servo1
                                      │                ├─ Servo2
        (Current flows               Battery ─────────┘
         through ground,
         causes noise)            (All grounds meet
                                   at ONE point)
```

### 4. Power Decoupling

```
Place capacitors near ICs:

        ESP32
    ┌──────────┐
    │          │
    │   VCC ───┼──┬──[10µF]──┐
    │   GND ───┼──┤          ├─── Power Supply
    │          │  └─[100nF]──┘
    └──────────┘
    
- 100nF ceramic: High frequency noise
- 10µF electrolytic: Low frequency, bulk
- Place as close as possible to chip
```

### 5. Shielding and Twisting

```
For I2C over longer distances:

SDA ─────────────────────  (Twisted pair)
GND ─────────────────────
     └─── Twisted together reduces interference

For servo wires:
PWM ─────────────────────  (Twisted with ground)
GND ─────────────────────
```

### 6. Connector Standards

```
Servo Connectors (standard):
┌─────────────────┐
│  Brown  Black   │ ← Ground
│  Red            │ ← +5V
│  Orange Yellow  │ ← Signal
└─────────────────┘
   Futaba/JR style (most common)

Polarity Marking:
- Use colored wire or labels
- Mark + and - on connectors
- Use keyed connectors when possible
```

### 7. Common Mistakes to Avoid

```
❌ DON'T:
- Connect 5V to ESP32 GPIO (use 3.3V)
- Power servos from ESP32 3.3V pin (not enough current)
- Use long wires for SPI (causes errors)
- Forget common ground between devices
- Cross power and signal wires (causes noise)

✓ DO:
- Use separate power supply for servos (BEC)
- Keep wires short and organized
- Connect all grounds together (star topology)
- Add decoupling capacitors
- Test continuity with multimeter
- Check polarity before powering on
```

---

## Troubleshooting Guide

### Power Issues

```
Problem: ESP32 resets when servo moves
Cause: Voltage drop / insufficient current
Solution: 
  - Use separate BEC for servos
  - Add bulk capacitor (100-1000µF) near servos
  - Check battery capacity

Problem: MPU6050 not detected
Cause: Poor I2C connection or wrong voltage
Solution:
  - Check 3.3V supply (measure with multimeter)
  - Verify I2C wiring (SCL, SDA, GND)
  - Check I2C pull-up resistors (should be 4.7kΩ)
  - Try I2C scanner code

Problem: Display garbled
Cause: SPI connection issue
Solution:
  - Check all 8 wires (VCC, GND, SCK, SDA, CS, RST, DC, LED)
  - Verify 3.3V supply
  - Ensure short wires for SPI
```

### Signal Issues

```
Problem: Servo jitters
Cause: Noise on PWM line or insufficient power
Solution:
  - Add capacitor across servo power (100µF)
  - Shorten PWM wire
  - Reduce PID gains (if flutter during hold)
  - Check ground connection

Problem: I2C communication errors
Cause: Bus speed too high or EMI
Solution:
  - Reduce I2C speed to 100kHz
  - Add ferrite beads to I2C lines
  - Keep I2C wires away from motors/servos
  - Check pull-up resistor values
```

---

## Safety Warnings

### ⚠️ IMPORTANT SAFETY NOTES

1. **Servo Stall Protection**
   - Servos draw HIGH current when stalled (800mA+)
   - Can overheat and damage servo or BEC
   - Always test range of motion mechanically first
   - Use current limiting if possible

2. **Battery Safety (LiPo)**
   - Never discharge below 3.0V per cell
   - Always use LiPo-safe charging bag
   - Never puncture or short circuit
   - Store at 3.8V per cell (storage voltage)

3. **ESD Protection**
   - ESP32 and MPU6050 are static sensitive
   - Ground yourself before handling
   - Use anti-static bags for storage

4. **Reverse Polarity**
   - Always check polarity with multimeter first
   - Mark positive/negative clearly
   - Use keyed connectors when possible
   - Reverse polarity WILL damage components

5. **Heat Management**
   - Voltage regulators can get hot under load
   - Ensure adequate ventilation
   - Consider heatsinks for BEC
   - Monitor temperature during extended use

---

## Complete Parts List

### Transmitter

| Qty | Part | Specs | Purpose |
|-----|------|-------|---------|
| 1 | ESP32 DevKit | 30-pin | Main controller |
| 1 | MPU6050 | Breakout board | IMU sensor |
| 2 | Servo | SG90 or similar | Elevons |
| 1 | BEC | 5V, 3A | Servo power |
| 1 | LiPo Battery | 7.4V 2S, 1000mAh+ | Main power |
| - | Wire | 22 AWG | Signal wires |
| - | Wire | 18 AWG | Power wires |
| 1 | Capacitor | 100µF 16V | Servo decoupling |
| 1 | Capacitor | 10µF | ESP32 decoupling |

### Receiver

| Qty | Part | Specs | Purpose |
|-----|------|-------|---------|
| 1 | ESP32 DevKit | 30-pin | Main controller |
| 1 | ST7735 LCD | 1.8" 128x160 | Display |
| 1 | USB Cable | USB-A to Micro-USB | Power |
| - | Wire | 22 AWG | Connections |
| 1 | Capacitor | 10µF | ESP32 decoupling |

---

**END OF HARDWARE DOCUMENTATION**

