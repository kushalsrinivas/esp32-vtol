# 🚀 Quick Start Guide - ESP-NOW Telemetry System

## Get Up and Running in 5 Minutes!

### 📦 What You Need

**Hardware:**
- 2x ESP32 boards
- 1x MPU6050 sensor
- 2x Servo motors
- 1x ILI9341 TFT display
- External 5V power for servos
- Jumper wires

**Software:**
- Arduino IDE with ESP32 support
- Libraries: ESP32Servo, Adafruit_GFX, Adafruit_ILI9341

---

## 🔌 Quick Wiring Reference

### Transmitter ESP32
```
MPU6050:          Servos:
  VCC → 3.3V      GPIO 25 → Servo 1 Signal
  GND → GND       GPIO 26 → Servo 2 Signal
  SDA → GPIO 21   5V EXT → Servo VCC (both)
  SCL → GPIO 22   GND → Servo GND (both)
```

### Receiver ESP32
```
ILI9341 TFT:
  VCC → 3.3V      CS → GPIO 5
  GND → GND       DC → GPIO 21
  MOSI → GPIO 23  RST → GPIO 4
  SCK → GPIO 18   LED → 3.3V
```

---

## ⚡ Upload Process (Step-by-Step)

### Step 1️⃣: Upload Receiver FIRST
1. Connect **Receiver ESP32** via USB
2. Open `receiver_esp32.ino`
3. Select: **Tools → Board → ESP32 Dev Module**
4. Select: **Tools → Port → (Your COM port)**
5. Click **Upload**
6. Open **Serial Monitor (115200 baud)**
7. **📝 COPY THE MAC ADDRESS** shown (e.g., `A4:CF:12:34:56:78`)

### Step 2️⃣: Configure Transmitter
1. Open `transmitter_esp32.ino`
2. Find line ~54:
   ```cpp
   uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
   ```
3. Replace with your receiver's MAC address:
   ```cpp
   // Example for MAC: A4:CF:12:34:56:78
   uint8_t receiverAddress[] = {0xA4, 0xCF, 0x12, 0x34, 0x56, 0x78};
   ```

### Step 3️⃣: Upload Transmitter
1. Connect **Transmitter ESP32** via USB
2. Click **Upload**
3. Wait for "Done uploading" message

### Step 4️⃣: Power Everything Up
1. Connect MPU6050 and servos to transmitter
2. Connect TFT display to receiver
3. Power both ESP32s
4. **Watch the magic happen! ✨**

---

## ✅ Success Indicators

**Receiver Display Should Show:**
- ✓ "TELEMETRY" title at top
- ✓ Two scrolling graphs (cyan & magenta)
- ✓ Real-time numerical values at bottom
- ✓ Green "CONNECTED" status

**Transmitter Serial Monitor:**
- ✓ Lines starting with "TX→" showing data
- ✓ No error messages

**Physical Test:**
- ✓ Tilt MPU6050 → graphs update
- ✓ Servos move smoothly
- ✓ Display shows ~20 Hz update rate

---

## 🆘 Common Issues

| Problem | Quick Fix |
|---------|-----------|
| "WAITING..." on display | Wrong MAC address in transmitter code |
| Blank TFT screen | Check TFT wiring (especially CS, DC, RST) |
| Jittery servos | Use external 5V power, NOT ESP32 pin! |
| "ESP-NOW Init Failed" | Update ESP32 board package in Arduino |
| Compilation error "esp_now_recv_cb_t" | Update ESP32 core to v3.0.0+ (See full guide) |
| No serial output | Wrong baud rate (should be 115200) |

---

## 📖 Full Documentation

For detailed setup, troubleshooting, and advanced features:
- **ESP_NOW_SETUP_GUIDE.md** - Complete setup instructions
- **PID_TUNING_QUICK_REFERENCE.md** - Servo tuning guide
- **README.md** - Original project documentation

---

## 🎯 Next Steps

1. **Test the system** - Tilt sensor and watch display
2. **Tune PID if needed** - Adjust Kp, Ki, Kd values
3. **Optimize placement** - Keep ESP32s within good range
4. **Monitor performance** - Check update rate on display

---

## 💡 Pro Tips

- ⚡ First time? Keep ESP32s close (~2m) for testing
- 🔋 Use good quality power supply for servos
- 📱 Open serial monitors on BOTH ESP32s for debugging
- 🎨 Display colors are customizable in code
- 📊 Update rate should be 18-22 Hz for best performance

---

**Ready to fly? 🚁 Follow the steps above and you'll be up in minutes!**

For questions, refer to the troubleshooting section in ESP_NOW_SETUP_GUIDE.md

