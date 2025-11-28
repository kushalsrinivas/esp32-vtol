/* ESP32 RECEIVER - ESP-NOW + ST7735 TFT Display

   This receiver gets telemetry data via ESP-NOW and displays it
   on a 1.8" ST7735 TFT LCD screen with real-time data.

   IMPORTANT - ESP32 ARDUINO CORE VERSION:
   ========================================
   This code is compatible with ESP32 Arduino Core v3.x and newer.
   If you're using an older version (v2.x), you'll get a compilation error.

   To fix: Update your ESP32 board package in Arduino IDE:
     Tools → Board → Boards Manager → Search "ESP32" → Update to latest version

   OR if you must use v2.x, change line ~109 to:
     void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)

   HARDWARE SETUP:
   ===============
   ST7735 TFT Display (1.8" 128x160):
     VCC -> 3.3V (or 5V depending on your module)
     GND -> GND
     SDA (MOSI) -> GPIO 23
     SCK        -> GPIO 18
     CS         -> GPIO 5
     DC (A0)    -> GPIO 21
     RST        -> GPIO 4
     LED (BL)   -> 3.3V

   5-Way Joystick (Optional):
     VRX -> GPIO 34 (Analog X)
     VRY -> GPIO 35 (Analog Y)
     SW  -> GPIO 13 (Button)
     VCC -> 3.3V
     GND -> GND

   FIRST TIME SETUP:
   =================
   1. Upload this code to your receiver ESP32
   2. Open Serial Monitor (115200 baud)
   3. Note down the MAC address displayed
   4. Copy that MAC address to the transmitter code
   5. Upload transmitter code with the correct MAC address

   FEATURES:
   =========
   - Real-time telemetry display optimized for 128x160 screen
   - Multiple view screens (Orientation, Gyro, Accel, Servo, Data Logger)
   - Connection status indicator
   - Data update rate display
   - Color-coded values
   - Menu navigation system
*/

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ==================== TFT PIN CONFIGURATION ====================
#define TFT_CS 5
#define TFT_DC 21
#define TFT_RST 4
#define TFT_MOSI 23
#define TFT_SCLK 18

// ==================== 5-WAY JOYSTICK CONFIGURATION ====================
#define JOYSTICK_VRX 34 // Analog X-axis
#define JOYSTICK_VRY 35 // Analog Y-axis
#define JOYSTICK_SW 13  // Center button (active LOW)

// ==================== TELEMETRY DATA STRUCTURE ====================
typedef struct
{
    float tiltX;
    float tiltY;
    float servo1_des;
    float servo1_actual;
    float servo2_des;
    float servo2_actual;
    float gyroX;
    float gyroY;
    float accelX;
    float accelY;
    float accelZ;
    uint32_t timestamp;
} TelemetryData;

TelemetryData receivedData;

// ==================== TFT DISPLAY ====================
// ST7735 Display - 128x160 pixels (smaller than ILI9341)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// ==================== DISPLAY VARIABLES ====================
// ST7735 is typically 128x160 in portrait, or 160x128 in landscape
#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 128

// Connection monitoring
unsigned long lastReceiveTime = 0;
unsigned long lastUpdateTime = 0;
bool isConnected = false;
int packetCount = 0;
float updateRate = 0;

// Data ready flag (set in ISR, cleared in main loop)
volatile bool dataReady = false;

// ==================== MENU SYSTEM ====================
enum MenuScreen
{
    MENU_MAIN,
    SCREEN_ORIENTATION,
    SCREEN_GYRO,
    SCREEN_ACCEL,
    SCREEN_SERVO_DETAIL,
    SCREEN_DATA_LOGGER
};

MenuScreen currentScreen = MENU_MAIN;
int menuSelection = 0;
const int menuItemCount = 5;

// Menu item names
const char *menuItems[] = {
    "Orientation",
    "Gyroscope",
    "Accelerometer",
    "Servo Detail",
    "Data Logger"};

// Joystick variables
unsigned long lastJoystickRead = 0;
unsigned long lastButtonPress = 0;
const unsigned long joystickDebounce = 150;
const unsigned long buttonDebounce = 300;
int joystickCenterX = 2048;
int joystickCenterY = 2048;

// Min/Max tracking
struct MinMaxTracker
{
    float tiltX_min, tiltX_max;
    float tiltY_min, tiltY_max;
    float gyroX_min, gyroX_max;
    float gyroY_min, gyroY_max;
    float accelX_min, accelX_max;
    float accelY_min, accelY_max;
    float accelZ_min, accelZ_max;
    unsigned long startTime;
    bool tracking;
} minMaxData;

// Screen update optimization
unsigned long lastScreenUpdate = 0;
const unsigned long screenUpdateInterval = 100; // ~10 FPS (slower for smaller display)

// Display buffering to reduce flicker
bool needsFullRedraw = true;
TelemetryData lastDisplayedData;

// Colors - ST7735 color definitions
#define COLOR_BG ST77XX_BLACK
#define COLOR_TEXT ST77XX_WHITE
#define COLOR_TILTX ST77XX_CYAN
#define COLOR_TILTY ST77XX_MAGENTA
#define COLOR_SERVO1 ST77XX_GREEN
#define COLOR_SERVO2 ST77XX_YELLOW
#define COLOR_GYRO ST77XX_ORANGE
#define COLOR_GRID 0x2104 // Dark gray
#define COLOR_WARN ST77XX_RED
#define COLOR_GOOD ST77XX_GREEN

// ==================== JOYSTICK DIRECTION ENUM ====================
enum JoystickDirection
{
    JS_NONE,
    JS_UP,
    JS_DOWN,
    JS_LEFT,
    JS_RIGHT,
    JS_PRESSED
};

// ==================== JOYSTICK FUNCTIONS ====================
void initJoystick()
{
    pinMode(JOYSTICK_SW, INPUT_PULLUP);

    // Calibrate center position
    joystickCenterX = analogRead(JOYSTICK_VRX);
    joystickCenterY = analogRead(JOYSTICK_VRY);
    delay(100);
}

JoystickDirection readJoystick()
{
    unsigned long now = millis();

    // Check button press
    if (digitalRead(JOYSTICK_SW) == LOW)
    {
        if (now - lastButtonPress > buttonDebounce)
        {
            lastButtonPress = now;
            return JS_PRESSED;
        }
        return JS_NONE;
    }

    // Check joystick movement
    if (now - lastJoystickRead < joystickDebounce)
    {
        return JS_NONE;
    }

    int x = analogRead(JOYSTICK_VRX);
    int y = analogRead(JOYSTICK_VRY);

    const int threshold = 1000; // Deadzone threshold

    if (y < joystickCenterY - threshold)
    {
        lastJoystickRead = now;
        return JS_UP;
    }
    else if (y > joystickCenterY + threshold)
    {
        lastJoystickRead = now;
        return JS_DOWN;
    }
    else if (x < joystickCenterX - threshold)
    {
        lastJoystickRead = now;
        return JS_LEFT;
    }
    else if (x > joystickCenterX + threshold)
    {
        lastJoystickRead = now;
        return JS_RIGHT;
    }

    return JS_NONE;
}

// ==================== MIN/MAX TRACKER ====================
void resetMinMax()
{
    minMaxData.tiltX_min = minMaxData.tiltY_min = 999;
    minMaxData.gyroX_min = minMaxData.gyroY_min = 999;
    minMaxData.accelX_min = minMaxData.accelY_min = minMaxData.accelZ_min = 999;

    minMaxData.tiltX_max = minMaxData.tiltY_max = -999;
    minMaxData.gyroX_max = minMaxData.gyroY_max = -999;
    minMaxData.accelX_max = minMaxData.accelY_max = minMaxData.accelZ_max = -999;

    minMaxData.startTime = millis();
    minMaxData.tracking = true;
}

void updateMinMax()
{
    if (!minMaxData.tracking)
        return;

    minMaxData.tiltX_min = min(minMaxData.tiltX_min, receivedData.tiltX);
    minMaxData.tiltX_max = max(minMaxData.tiltX_max, receivedData.tiltX);
    minMaxData.tiltY_min = min(minMaxData.tiltY_min, receivedData.tiltY);
    minMaxData.tiltY_max = max(minMaxData.tiltY_max, receivedData.tiltY);

    minMaxData.gyroX_min = min(minMaxData.gyroX_min, receivedData.gyroX);
    minMaxData.gyroX_max = max(minMaxData.gyroX_max, receivedData.gyroX);
    minMaxData.gyroY_min = min(minMaxData.gyroY_min, receivedData.gyroY);
    minMaxData.gyroY_max = max(minMaxData.gyroY_max, receivedData.gyroY);

    minMaxData.accelX_min = min(minMaxData.accelX_min, receivedData.accelX);
    minMaxData.accelX_max = max(minMaxData.accelX_max, receivedData.accelX);
    minMaxData.accelY_min = min(minMaxData.accelY_min, receivedData.accelY);
    minMaxData.accelY_max = max(minMaxData.accelY_max, receivedData.accelY);
    minMaxData.accelZ_min = min(minMaxData.accelZ_min, receivedData.accelZ);
    minMaxData.accelZ_max = max(minMaxData.accelZ_max, receivedData.accelZ);
}

// ==================== ESP-NOW CALLBACK ====================
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
    if (len == sizeof(TelemetryData))
    {
        memcpy(&receivedData, incomingData, sizeof(receivedData));

        unsigned long now = millis();
        unsigned long timeSinceLastUpdate = now - lastUpdateTime;

        if (timeSinceLastUpdate > 0)
        {
            updateRate = 1000.0 / timeSinceLastUpdate;
        }

        lastReceiveTime = now;
        lastUpdateTime = now;
        isConnected = true;
        packetCount++;

        dataReady = true;
        updateMinMax();
    }
}

// ==================== SETUP ====================
void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  ESP32 RECEIVER - ST7735 Display      ║");
    Serial.println("║  with ESP-NOW Telemetry                ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    // ========== INITIALIZE TFT DISPLAY ==========
    Serial.println("→ Initializing ST7735 display...");

    // Initialize ST7735 - use initR(INITR_BLACKTAB) for most black tab displays
    // Other options: INITR_GREENTAB, INITR_REDTAB, INITR_144GREENTAB
    tft.initR(INITR_BLACKTAB);

    tft.setRotation(3); // Landscape orientation (160x128)
    tft.fillScreen(COLOR_BG);
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(1);

    // Display startup message
    tft.setCursor(30, 50);
    tft.setTextSize(1);
    tft.println("ESP32 RECEIVER");
    tft.setCursor(30, 65);
    tft.println("Initializing...");

    Serial.println("✓ ST7735 display initialized\n");

    // ========== INITIALIZE ESP-NOW ==========
    Serial.println("→ Initializing ESP-NOW...");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    String macAddress = WiFi.macAddress();
    Serial.print("  Receiver MAC Address: ");
    Serial.println(macAddress);
    Serial.println("\n  ╔══════════════════════════════════════════════╗");
    Serial.println("  ║  IMPORTANT: Copy this MAC address to the    ║");
    Serial.println("  ║  transmitter code (receiverAddress array)   ║");
    Serial.println("  ╚══════════════════════════════════════════════╝\n");

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("✗ ERROR: ESP-NOW initialization failed!");
        tft.fillScreen(COLOR_BG);
        tft.setCursor(20, 50);
        tft.setTextColor(COLOR_WARN);
        tft.println("ESP-NOW Failed!");
        while (1)
            delay(1000);
    }

    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("✓ ESP-NOW initialized successfully\n");

    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  SYSTEM READY - Waiting for data...   ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    // Initialize joystick
    Serial.println("→ Initializing joystick...");
    initJoystick();
    Serial.println("✓ Joystick initialized\n");

    // Initialize min/max tracker
    resetMinMax();

    // Draw initial menu
    drawMainMenu();

    lastReceiveTime = millis();
}

// ==================== MAIN LOOP ====================
void loop()
{
    // Handle joystick input
    JoystickDirection jsDir = readJoystick();

    if (jsDir != JS_NONE)
    {
        handleJoystickInput(jsDir);
    }

    // Process new data if available
    if (dataReady)
    {
        dataReady = false;
    }

    // Check connection status
    if (millis() - lastReceiveTime > 2000)
    {
        if (isConnected)
        {
            isConnected = false;
            if (currentScreen != MENU_MAIN)
            {
                displayConnectionLost();
            }
        }
    }

    // Update screen at optimized refresh rate
    unsigned long now = millis();
    if (now - lastScreenUpdate >= screenUpdateInterval && isConnected)
    {
        updateCurrentScreen();
        lastScreenUpdate = now;
    }

    delay(10);
}

// ==================== JOYSTICK INPUT HANDLER ====================
void handleJoystickInput(JoystickDirection dir)
{
    if (currentScreen == MENU_MAIN)
    {
        if (dir == JS_UP)
        {
            menuSelection = (menuSelection - 1 + menuItemCount) % menuItemCount;
            drawMainMenu();
        }
        else if (dir == JS_DOWN)
        {
            menuSelection = (menuSelection + 1) % menuItemCount;
            drawMainMenu();
        }
        else if (dir == JS_PRESSED)
        {
            currentScreen = (MenuScreen)(menuSelection + 1);
            initCurrentScreen();
        }
    }
    else
    {
        if (dir == JS_LEFT || dir == JS_PRESSED)
        {
            currentScreen = MENU_MAIN;
            drawMainMenu();
        }
        else if (dir == JS_UP || dir == JS_DOWN)
        {
            int screenIndex = (int)currentScreen - 1;
            if (dir == JS_UP)
            {
                screenIndex = (screenIndex - 1 + menuItemCount) % menuItemCount;
            }
            else
            {
                screenIndex = (screenIndex + 1) % menuItemCount;
            }
            currentScreen = (MenuScreen)(screenIndex + 1);
            initCurrentScreen();
        }
    }
}

// ==================== SCREEN UPDATE ROUTER ====================
void updateCurrentScreen()
{
    switch (currentScreen)
    {
    case MENU_MAIN:
        break;
    case SCREEN_ORIENTATION:
        updateOrientationScreen();
        break;
    case SCREEN_GYRO:
        updateGyroScreen();
        break;
    case SCREEN_ACCEL:
        updateAccelScreen();
        break;
    case SCREEN_SERVO_DETAIL:
        updateServoDetailScreen();
        break;
    case SCREEN_DATA_LOGGER:
        updateDataLoggerScreen();
        break;
    }
}

void initCurrentScreen()
{
    tft.fillScreen(COLOR_BG);
    needsFullRedraw = true;
    memset(&lastDisplayedData, 0, sizeof(lastDisplayedData));

    switch (currentScreen)
    {
    case SCREEN_ORIENTATION:
        drawOrientationScreen();
        break;
    case SCREEN_GYRO:
        drawGyroScreen();
        break;
    case SCREEN_ACCEL:
        drawAccelScreen();
        break;
    case SCREEN_SERVO_DETAIL:
        drawServoDetailScreen();
        break;
    case SCREEN_DATA_LOGGER:
        drawDataLoggerScreen();
        break;
    default:
        break;
    }
}

// ==================== DISPLAY FUNCTIONS ====================

// ---------- MAIN MENU ----------
void drawMainMenu()
{
    tft.fillScreen(COLOR_BG);

    // Title with decorative border
    tft.fillRect(0, 0, SCREEN_WIDTH, 15, 0x1082);
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(48, 4);
    tft.println("TELEMETRY");

    // Connection status
    tft.fillCircle(5, 7, 2, isConnected ? COLOR_GOOD : COLOR_WARN);

    // Menu items - compact layout for small screen
    int startY = 20;
    int itemHeight = 20;

    for (int i = 0; i < menuItemCount; i++)
    {
        int y = startY + (i * itemHeight);

        if (i == menuSelection)
        {
            tft.fillRoundRect(5, y - 1, SCREEN_WIDTH - 10, 18, 3, 0x1082);
            tft.drawRoundRect(5, y - 1, SCREEN_WIDTH - 10, 18, 3, COLOR_GOOD);
        }

        tft.setTextSize(1);
        tft.setTextColor(i == menuSelection ? COLOR_GOOD : COLOR_TEXT);
        tft.setCursor(10, y + 4);
        tft.println(menuItems[i]);
    }

    // Instructions
    tft.setTextSize(1);
    tft.setTextColor(0x7BEF);
    tft.setCursor(15, 118);
    tft.println("UP/DN: Nav  SEL");
}

// ---------- HELPER FUNCTIONS ----------
void drawScreenHeader(const char *title)
{
    tft.fillRect(0, 0, SCREEN_WIDTH, 12, 0x1082);
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    int titleWidth = strlen(title) * 6;
    tft.setCursor((SCREEN_WIDTH - titleWidth) / 2, 2);
    tft.println(title);

    // Back indicator
    tft.setCursor(2, 2);
    tft.print("<");

    // Connection indicator
    tft.fillCircle(SCREEN_WIDTH - 5, 6, 2, isConnected ? COLOR_GOOD : COLOR_WARN);
}

// ---------- ORIENTATION SCREEN ----------
void drawOrientationScreen()
{
    drawScreenHeader("ORIENTATION");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(5, 18);
    tft.print("Roll:");
    tft.setCursor(5, 52);
    tft.print("Pitch:");
}

void updateOrientationScreen()
{
    bool tiltXChanged = abs(receivedData.tiltX - lastDisplayedData.tiltX) > 0.2;
    bool tiltYChanged = abs(receivedData.tiltY - lastDisplayedData.tiltY) > 0.2;

    if (tiltXChanged || needsFullRedraw)
    {
        // Roll value
        tft.fillRect(5, 28, 150, 20, COLOR_BG);
        tft.setTextSize(2);
        tft.setTextColor(COLOR_TILTX);
        tft.setCursor(5, 28);
        tft.printf("%+5.1f", receivedData.tiltX);
        tft.print((char)247);

        // Roll bar
        tft.fillRect(5, 48, 150, 4, COLOR_BG);
        tft.drawFastVLine(80, 48, 4, COLOR_GRID);
        int rollBarX = 80 + (int)(receivedData.tiltX * 1.5);
        rollBarX = constrain(rollBarX, 5, 155);
        int barWidth = abs(rollBarX - 80);
        if (barWidth > 0)
        {
            tft.fillRect(min(80, rollBarX), 48, barWidth, 4, COLOR_TILTX);
        }
    }

    if (tiltYChanged || needsFullRedraw)
    {
        // Pitch value
        tft.fillRect(5, 62, 150, 20, COLOR_BG);
        tft.setTextSize(2);
        tft.setTextColor(COLOR_TILTY);
        tft.setCursor(5, 62);
        tft.printf("%+5.1f", receivedData.tiltY);
        tft.print((char)247);

        // Pitch bar
        tft.fillRect(5, 82, 150, 4, COLOR_BG);
        tft.drawFastVLine(80, 82, 4, COLOR_GRID);
        int pitchBarX = 80 + (int)(receivedData.tiltY * 1.5);
        pitchBarX = constrain(pitchBarX, 5, 155);
        int barWidth = abs(pitchBarX - 80);
        if (barWidth > 0)
        {
            tft.fillRect(min(80, pitchBarX), 82, barWidth, 4, COLOR_TILTY);
        }
    }

    // Status info
    if (needsFullRedraw)
    {
        tft.fillRect(5, 90, 150, 35, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(5, 92);
        tft.print("Servo 1:");
        tft.setCursor(5, 102);
        tft.print("Servo 2:");
        tft.setCursor(5, 112);
        tft.print("Rate:");
    }

    // Update servo values and rate (less frequently)
    static unsigned long lastInfoUpdate = 0;
    if (millis() - lastInfoUpdate > 200 || needsFullRedraw)
    {
        tft.fillRect(60, 92, 95, 30, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(COLOR_SERVO1);
        tft.setCursor(60, 92);
        tft.printf("%d", (int)receivedData.servo1_actual);
        tft.print((char)247);

        tft.setTextColor(COLOR_SERVO2);
        tft.setCursor(60, 102);
        tft.printf("%d", (int)receivedData.servo2_actual);
        tft.print((char)247);

        tft.setTextColor(COLOR_GOOD);
        tft.setCursor(60, 112);
        tft.printf("%.1f Hz", updateRate);

        lastInfoUpdate = millis();
    }

    needsFullRedraw = false;
    lastDisplayedData.tiltX = receivedData.tiltX;
    lastDisplayedData.tiltY = receivedData.tiltY;
}

// ---------- GYROSCOPE SCREEN ----------
void drawGyroScreen()
{
    drawScreenHeader("GYROSCOPE");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(5, 18);
    tft.print("Gyro X:");
    tft.setCursor(5, 52);
    tft.print("Gyro Y:");
}

void updateGyroScreen()
{
    bool gyroXChanged = abs(receivedData.gyroX - lastDisplayedData.gyroX) > 0.5;
    bool gyroYChanged = abs(receivedData.gyroY - lastDisplayedData.gyroY) > 0.5;

    if (gyroXChanged || needsFullRedraw)
    {
        tft.fillRect(5, 28, 150, 20, COLOR_BG);
        tft.setTextSize(2);
        tft.setTextColor(COLOR_GYRO);
        tft.setCursor(5, 28);
        tft.printf("%+5.1f", receivedData.gyroX);
        tft.setTextSize(1);
        tft.print("d/s");
    }

    if (gyroYChanged || needsFullRedraw)
    {
        tft.fillRect(5, 62, 150, 20, COLOR_BG);
        tft.setTextSize(2);
        tft.setTextColor(COLOR_GYRO);
        tft.setCursor(5, 62);
        tft.printf("%+5.1f", receivedData.gyroY);
        tft.setTextSize(1);
        tft.print("d/s");
    }

    // Min/Max values
    if (needsFullRedraw)
    {
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(5, 90);
        tft.print("Min/Max:");
    }

    static unsigned long lastMinMaxUpdate = 0;
    if (millis() - lastMinMaxUpdate > 500 || needsFullRedraw)
    {
        tft.fillRect(5, 100, 150, 20, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(5, 100);
        tft.printf("X: %.1f/%.1f", minMaxData.gyroX_min, minMaxData.gyroX_max);
        tft.setCursor(5, 110);
        tft.printf("Y: %.1f/%.1f", minMaxData.gyroY_min, minMaxData.gyroY_max);

        lastMinMaxUpdate = millis();
    }

    needsFullRedraw = false;
    lastDisplayedData.gyroX = receivedData.gyroX;
    lastDisplayedData.gyroY = receivedData.gyroY;
}

// ---------- ACCELEROMETER SCREEN ----------
void drawAccelScreen()
{
    drawScreenHeader("ACCELEROMETER");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(5, 18);
    tft.print("Accel X:");
    tft.setCursor(5, 42);
    tft.print("Accel Y:");
    tft.setCursor(5, 66);
    tft.print("Accel Z:");
}

void updateAccelScreen()
{
    bool accelChanged = abs(receivedData.accelX - lastDisplayedData.accelX) > 0.02 ||
                        abs(receivedData.accelY - lastDisplayedData.accelY) > 0.02 ||
                        abs(receivedData.accelZ - lastDisplayedData.accelZ) > 0.02;

    if (accelChanged || needsFullRedraw)
    {
        tft.fillRect(5, 26, 150, 35, COLOR_BG);

        tft.setTextSize(1);
        tft.setTextColor(0xF800); // Red
        tft.setCursor(5, 26);
        tft.printf("%+5.2f g", receivedData.accelX);

        tft.setTextColor(0x07E0); // Green
        tft.setCursor(5, 50);
        tft.printf("%+5.2f g", receivedData.accelY);

        tft.setTextColor(0x001F); // Blue
        tft.setCursor(5, 74);
        tft.printf("%+5.2f g", receivedData.accelZ);

        // Simple vector visualization
        tft.fillRect(5, 88, 80, 35, COLOR_BG);
        tft.drawRect(5, 88, 80, 35, COLOR_GRID);

        int centerX = 45;
        int centerY = 105;
        int scale = 25;

        // Draw acceleration vector
        int endX = centerX + (int)(receivedData.accelX * scale);
        int endY = centerY - (int)(receivedData.accelY * scale);
        endX = constrain(endX, 8, 82);
        endY = constrain(endY, 91, 120);

        tft.drawLine(centerX, centerY, endX, endY, COLOR_WARN);
        tft.fillCircle(endX, endY, 2, COLOR_WARN);

        // Magnitude
        float magnitude = sqrt(receivedData.accelX * receivedData.accelX +
                               receivedData.accelY * receivedData.accelY +
                               receivedData.accelZ * receivedData.accelZ);
        tft.fillRect(90, 100, 65, 15, COLOR_BG);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(90, 100);
        tft.printf("Mag:%.2fg", magnitude);
    }

    needsFullRedraw = false;
    lastDisplayedData.accelX = receivedData.accelX;
    lastDisplayedData.accelY = receivedData.accelY;
    lastDisplayedData.accelZ = receivedData.accelZ;
}

// ---------- SERVO DETAIL SCREEN ----------
void drawServoDetailScreen()
{
    drawScreenHeader("SERVO DETAIL");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);

    tft.setCursor(5, 18);
    tft.print("SERVO 1:");
    tft.drawRect(5, 30, 150, 30, COLOR_SERVO1);

    tft.setCursor(5, 68);
    tft.print("SERVO 2:");
    tft.drawRect(5, 80, 150, 30, COLOR_SERVO2);
}

void updateServoDetailScreen()
{
    bool servo1Changed = abs(receivedData.servo1_actual - lastDisplayedData.servo1_actual) > 0.5;
    bool servo2Changed = abs(receivedData.servo2_actual - lastDisplayedData.servo2_actual) > 0.5;

    if (servo1Changed || needsFullRedraw)
    {
        tft.fillRect(8, 33, 144, 24, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(COLOR_SERVO1);
        tft.setCursor(10, 35);
        tft.printf("T:%d A:%d", (int)receivedData.servo1_des, (int)receivedData.servo1_actual);

        int error1 = abs((int)(receivedData.servo1_des - receivedData.servo1_actual));
        tft.setCursor(10, 46);
        tft.setTextColor(error1 > 5 ? COLOR_WARN : COLOR_GOOD);
        tft.printf("Err: %d", error1);
        tft.print((char)247);

        // Visual bar
        tft.drawRect(10, 54, 135, 4, COLOR_GRID);
        int bar1Pos = map((int)receivedData.servo1_actual, 0, 180, 10, 145);
        tft.fillRect(bar1Pos - 1, 53, 3, 6, COLOR_SERVO1);
    }

    if (servo2Changed || needsFullRedraw)
    {
        tft.fillRect(8, 83, 144, 24, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(COLOR_SERVO2);
        tft.setCursor(10, 85);
        tft.printf("T:%d A:%d", (int)receivedData.servo2_des, (int)receivedData.servo2_actual);

        int error2 = abs((int)(receivedData.servo2_des - receivedData.servo2_actual));
        tft.setCursor(10, 96);
        tft.setTextColor(error2 > 5 ? COLOR_WARN : COLOR_GOOD);
        tft.printf("Err: %d", error2);
        tft.print((char)247);

        // Visual bar
        tft.drawRect(10, 104, 135, 4, COLOR_GRID);
        int bar2Pos = map((int)receivedData.servo2_actual, 0, 180, 10, 145);
        tft.fillRect(bar2Pos - 1, 103, 3, 6, COLOR_SERVO2);
    }

    // Performance metrics
    static unsigned long lastPerfUpdate = 0;
    if (millis() - lastPerfUpdate > 500 || needsFullRedraw)
    {
        tft.fillRect(5, 112, 150, 14, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(5, 112);
        tft.printf("Rate:%.1fHz Pkts:%d", updateRate, packetCount);

        lastPerfUpdate = millis();
    }

    needsFullRedraw = false;
    lastDisplayedData.servo1_actual = receivedData.servo1_actual;
    lastDisplayedData.servo2_actual = receivedData.servo2_actual;
}

// ---------- DATA LOGGER SCREEN ----------
void drawDataLoggerScreen()
{
    drawScreenHeader("DATA LOGGER");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(5, 18);
    tft.print("MIN / MAX VALUES:");

    tft.setCursor(5, 30);
    tft.print("Tilt X:");
    tft.setCursor(5, 42);
    tft.print("Tilt Y:");
    tft.setCursor(5, 54);
    tft.print("Gyro X:");
    tft.setCursor(5, 66);
    tft.print("Gyro Y:");
    tft.setCursor(5, 78);
    tft.print("Acc X:");
    tft.setCursor(5, 90);
    tft.print("Acc Y:");
    tft.setCursor(5, 102);
    tft.print("Acc Z:");

    tft.setCursor(5, 115);
    tft.setTextColor(COLOR_GOOD);
    tft.print("Tracking...");
}

void updateDataLoggerScreen()
{
    static unsigned long lastLogUpdate = 0;
    if (millis() - lastLogUpdate < 1000 && !needsFullRedraw)
    {
        return;
    }
    lastLogUpdate = millis();

    tft.fillRect(48, 30, 107, 80, COLOR_BG);

    tft.setTextSize(1);

    tft.setTextColor(COLOR_TILTX);
    tft.setCursor(48, 30);
    tft.printf("%.1f/%.1f", minMaxData.tiltX_min, minMaxData.tiltX_max);

    tft.setTextColor(COLOR_TILTY);
    tft.setCursor(48, 42);
    tft.printf("%.1f/%.1f", minMaxData.tiltY_min, minMaxData.tiltY_max);

    tft.setTextColor(COLOR_GYRO);
    tft.setCursor(48, 54);
    tft.printf("%.1f/%.1f", minMaxData.gyroX_min, minMaxData.gyroX_max);
    tft.setCursor(48, 66);
    tft.printf("%.1f/%.1f", minMaxData.gyroY_min, minMaxData.gyroY_max);

    tft.setTextColor(0xFD20);
    tft.setCursor(48, 78);
    tft.printf("%.2f/%.2f", minMaxData.accelX_min, minMaxData.accelX_max);
    tft.setCursor(48, 90);
    tft.printf("%.2f/%.2f", minMaxData.accelY_min, minMaxData.accelY_max);
    tft.setCursor(48, 102);
    tft.printf("%.2f/%.2f", minMaxData.accelZ_min, minMaxData.accelZ_max);

    needsFullRedraw = false;
}

void displayConnectionLost()
{
    Serial.println("⚠ Connection lost - waiting for transmitter...");

    tft.fillRect(20, 50, 120, 30, COLOR_WARN);
    tft.drawRect(20, 50, 120, 30, COLOR_TEXT);
    tft.setTextColor(COLOR_BG);
    tft.setCursor(35, 55);
    tft.setTextSize(1);
    tft.println("NO SIGNAL");
    tft.setCursor(25, 68);
    tft.println("Check transmitter");

    delay(1500);
    initCurrentScreen();
}

// ==================== ADDITIONAL INFO ====================
/*
 * ST7735 TELEMETRY SYSTEM
 * =======================
 *
 * DISPLAY INFO:
 * -------------
 * Screen: 160x128 pixels (landscape)
 * Library: Adafruit_ST7735
 * Common variants: INITR_BLACKTAB, INITR_GREENTAB, INITR_REDTAB
 *
 * JOYSTICK CONTROLS:
 * ------------------
 * Main Menu:
 *   UP/DOWN: Navigate menu items
 *   PRESS: Select menu item
 *
 * Sub-Screens:
 *   UP/DOWN: Cycle through screens
 *   LEFT/PRESS: Return to main menu
 *
 * AVAILABLE SCREENS:
 * ------------------
 * 1. ORIENTATION: Tilt angles with visual bars
 * 2. GYROSCOPE: Real-time gyro readings with min/max
 * 3. ACCELEROMETER: 3D acceleration with vector display
 * 4. SERVO DETAILS: Servo position tracking and errors
 * 5. DATA LOGGER: Min/Max tracking for all sensors
 *
 * FEATURES:
 * ---------
 * - Optimized for smaller ST7735 screen (160x128)
 * - Compact UI with essential information
 * - ~10 FPS refresh rate for smooth updates
 * - Connection status on all screens
 * - Real-time performance metrics
 *
 * NOTES:
 * ------
 * - Adjust INITR_BLACKTAB in setup() if display shows wrong colors
 * - Transmitter must send matching TelemetryData structure
 * - Simplified UI compared to ILI9341 version due to smaller screen
 */
