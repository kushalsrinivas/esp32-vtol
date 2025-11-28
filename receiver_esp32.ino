/* ESP32 RECEIVER - ESP-NOW + ILI9341 TFT Display

   This receiver gets telemetry data via ESP-NOW and displays it
   on a 2.4" ILI9341 TFT LCD screen with real-time graphs.

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
   ILI9341 TFT Display:
     VCC -> 3.3V
     GND -> GND
     SDA (MOSI) -> GPIO 23
     SCK        -> GPIO 18
     CS         -> GPIO 5
     DC (A0)    -> GPIO 21
     RST        -> GPIO 4
     LED        -> 3.3V

   FIRST TIME SETUP:
   =================
   1. Upload this code to your receiver ESP32
   2. Open Serial Monitor (115200 baud)
   3. Note down the MAC address displayed
   4. Copy that MAC address to the transmitter code
   5. Upload transmitter code with the correct MAC address

   FEATURES:
   =========
   - Real-time telemetry display
   - Live scrolling graphs for tilt angles
   - Connection status indicator
   - Data update rate display
   - Color-coded servo positions
*/

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

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
    float accelX; // Added accelerometer data
    float accelY;
    float accelZ;
    uint32_t timestamp;
} TelemetryData;

TelemetryData receivedData;

// ==================== CONTROL DATA STRUCTURE (Receiver → Transmitter) ====================
typedef struct
{
    bool manualMode;    // true = manual control, false = auto stabilization
    float pitchCommand; // -1.0 to +1.0 (joystick Y axis normalized)
    float rollCommand;  // -1.0 to +1.0 (joystick X axis normalized)
    uint32_t timestamp; // For connection monitoring
} ControlData;

ControlData controlData;

// ==================== TFT DISPLAY ====================
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// ==================== ESP-NOW TRANSMITTER PEER ====================
// Transmitter ESP32's MAC Address - REPLACE WITH YOUR TRANSMITTER MAC
uint8_t transmitterAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // UPDATE THIS!
esp_now_peer_info_t peerInfo;

// ==================== DISPLAY VARIABLES ====================
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// Graph settings
#define GRAPH_WIDTH 280
#define GRAPH_HEIGHT 60
#define GRAPH_X 35
#define GRAPH_Y_TILTX 50
#define GRAPH_Y_TILTY 130

int16_t graphData_TiltX[GRAPH_WIDTH];
int16_t graphData_TiltY[GRAPH_WIDTH];
int16_t graphData_GyroX[GRAPH_WIDTH];
int16_t graphData_GyroY[GRAPH_WIDTH];
int16_t graphData_AccelZ[GRAPH_WIDTH];
int graphIndex = 0;

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
    SCREEN_DATA_LOGGER,
    SCREEN_3D_HORIZON,
    SCREEN_MANUAL_MODE
};

MenuScreen currentScreen = MENU_MAIN;
int menuSelection = 0;
const int menuItemCount = 7;

// Menu item names
const char *menuItems[] = {
    "Orientation",
    "Gyroscope Data",
    "Accelerometer",
    "Servo Details",
    "Data Logger",
    "3D Horizon",
    "Manual Mode"};

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
const unsigned long screenUpdateInterval = 50; // ~20 FPS (reduced for smoother rendering)

// Display buffering to reduce flicker
bool needsFullRedraw = true;
TelemetryData lastDisplayedData; // Track what's currently on screen

// Colors
#define COLOR_BG 0x0000     // Black
#define COLOR_TEXT 0xFFFF   // White
#define COLOR_TILTX 0x07FF  // Cyan
#define COLOR_TILTY 0xF81F  // Magenta
#define COLOR_SERVO1 0x07E0 // Green
#define COLOR_SERVO2 0xFFE0 // Yellow
#define COLOR_GYRO 0xFD20   // Orange
#define COLOR_GRID 0x2104   // Dark gray
#define COLOR_WARN 0xF800   // Red
#define COLOR_GOOD 0x07E0   // Green

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
// IMPORTANT: This runs in ISR context - keep it minimal!
// Do NOT call display functions here - just set a flag
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
    if (len == sizeof(TelemetryData))
    {
        memcpy(&receivedData, incomingData, sizeof(receivedData));

        unsigned long now = millis();
        unsigned long timeSinceLastUpdate = now - lastUpdateTime;

        if (timeSinceLastUpdate > 0)
        {
            updateRate = 1000.0 / timeSinceLastUpdate; // Updates per second
        }

        lastReceiveTime = now;
        lastUpdateTime = now;
        isConnected = true;
        packetCount++;

        // Signal main loop that new data is ready
        dataReady = true;

        // Update min/max tracker
        updateMinMax();
    }
}

// ==================== SETUP ====================
void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  ESP32 RECEIVER - TFT Display         ║");
    Serial.println("║  with ESP-NOW Telemetry                ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    // ========== INITIALIZE TFT DISPLAY ==========
    Serial.println("→ Initializing TFT display...");
    tft.begin();
    tft.setRotation(3); // Landscape orientation

    // Optimize SPI speed for ILI9341 - higher speed = smoother rendering
    // ILI9341 can handle up to 40MHz, but 27MHz is more stable
    SPI.setFrequency(27000000); // 27 MHz

    tft.fillScreen(COLOR_BG);
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(1);

    // Display startup message
    tft.setCursor(60, 100);
    tft.setTextSize(2);
    tft.println("ESP32 RECEIVER");
    tft.setCursor(70, 120);
    tft.setTextSize(1);
    tft.println("Initializing...");

    Serial.println("✓ TFT display initialized\n");

    // ========== INITIALIZE ESP-NOW ==========
    Serial.println("→ Initializing ESP-NOW...");

    // Initialize WiFi in Station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); // Ensure clean state
    delay(100);        // Give WiFi time to initialize

    // Display MAC address
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
        tft.setCursor(50, 100);
        tft.setTextColor(COLOR_WARN);
        tft.println("ESP-NOW Init Failed!");
        while (1)
            delay(1000);
    }

    // Register receive callback
    esp_now_register_recv_cb(OnDataRecv);

    // Register transmitter as peer (for sending control data)
    memcpy(peerInfo.peer_addr, transmitterAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("⚠ Warning: Failed to add transmitter peer!");
        Serial.println("  Manual mode will not work until transmitter MAC is set.");
    }
    else
    {
        Serial.println("✓ Transmitter peer added successfully");
        Serial.print("  Can send control data to: ");
        for (int i = 0; i < 6; i++)
        {
            Serial.printf("%02X", transmitterAddress[i]);
            if (i < 5)
                Serial.print(":");
        }
        Serial.println();
    }

    Serial.println("✓ ESP-NOW initialized successfully\n");

    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  SYSTEM READY - Waiting for data...   ║");
    Serial.println("╚════════════════════════════════════════╝\n");

    // Initialize graph data
    for (int i = 0; i < GRAPH_WIDTH; i++)
    {
        graphData_TiltX[i] = GRAPH_HEIGHT / 2;
        graphData_TiltY[i] = GRAPH_HEIGHT / 2;
        graphData_GyroX[i] = GRAPH_HEIGHT / 2;
        graphData_GyroY[i] = GRAPH_HEIGHT / 2;
        graphData_AccelZ[i] = GRAPH_HEIGHT / 2;
    }

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

    // Process new data if available (from ESP-NOW callback)
    if (dataReady)
    {
        dataReady = false; // Clear flag

        // Update graphs with new data
        updateAllGraphs();
    }

    // Check connection status
    if (millis() - lastReceiveTime > 2000)
    {
        if (isConnected)
        {
            isConnected = false;
            // Only show connection lost if not in menu
            if (currentScreen != MENU_MAIN)
            {
                displayConnectionLost();
            }
        }
    }

    // Update screen at optimized refresh rate (reduced for smoother rendering)
    unsigned long now = millis();
    if (now - lastScreenUpdate >= screenUpdateInterval && isConnected && dataReady == false)
    {
        updateCurrentScreen();
        lastScreenUpdate = now;
    }

    // Small delay to prevent CPU hogging
    delay(10); // Slightly increased to reduce SPI bus contention
}

// ==================== JOYSTICK INPUT HANDLER ====================
void handleJoystickInput(JoystickDirection dir)
{
    if (currentScreen == MENU_MAIN)
    {
        // Main menu navigation
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
            // Enter selected screen
            currentScreen = (MenuScreen)(menuSelection + 1);
            initCurrentScreen();
        }
    }
    else
    {
        // In a sub-screen
        if (dir == JS_LEFT || dir == JS_PRESSED)
        {
            // Exit manual mode if leaving that screen
            if (currentScreen == SCREEN_MANUAL_MODE)
            {
                exitManualMode();
            }
            // Return to main menu
            currentScreen = MENU_MAIN;
            drawMainMenu();
        }
        else if (dir == JS_UP || dir == JS_DOWN)
        {
            // Exit manual mode if leaving that screen
            if (currentScreen == SCREEN_MANUAL_MODE)
            {
                exitManualMode();
            }
            // Navigate between screens
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
        // Menu doesn't need updating
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
    case SCREEN_3D_HORIZON:
        update3DHorizonScreen();
        break;
    case SCREEN_MANUAL_MODE:
        updateManualModeScreen();
        break;
    }
}

void initCurrentScreen()
{
    tft.fillScreen(COLOR_BG);
    needsFullRedraw = true;                                   // Force full redraw on screen change
    memset(&lastDisplayedData, 0, sizeof(lastDisplayedData)); // Reset comparison data

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
    case SCREEN_3D_HORIZON:
        draw3DHorizonScreen();
        break;
    case SCREEN_MANUAL_MODE:
        drawManualModeScreen();
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
    tft.fillRect(0, 0, SCREEN_WIDTH, 30, 0x1082);
    tft.setTextSize(2);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(90, 8);
    tft.println("TELEMETRY");

    // Connection status
    tft.setTextSize(1);
    tft.setCursor(5, 8);
    if (isConnected)
    {
        tft.fillCircle(10, 15, 3, COLOR_GOOD);
    }
    else
    {
        tft.fillCircle(10, 15, 3, COLOR_WARN);
    }

    // Menu items
    int startY = 45;
    int itemHeight = 30;

    for (int i = 0; i < menuItemCount; i++)
    {
        int y = startY + (i * itemHeight);

        // Highlight selected item
        if (i == menuSelection)
        {
            tft.fillRoundRect(10, y - 2, SCREEN_WIDTH - 20, 26, 5, 0x1082);
            tft.drawRoundRect(10, y - 2, SCREEN_WIDTH - 20, 26, 5, COLOR_GOOD);
        }

        tft.setTextSize(2);
        tft.setTextColor(i == menuSelection ? COLOR_GOOD : COLOR_TEXT);
        tft.setCursor(25, y + 4);
        tft.println(menuItems[i]);
    }

    // Instructions
    tft.setTextSize(1);
    tft.setTextColor(0x7BEF);
    tft.setCursor(65, 225);
    tft.println("UP/DN: Navigate  SELECT: Enter");
}

// ---------- HELPER FUNCTIONS ----------
void drawScreenHeader(const char *title)
{
    tft.fillRect(0, 0, SCREEN_WIDTH, 20, 0x1082);
    tft.setTextSize(2);
    tft.setTextColor(COLOR_TEXT);
    int titleWidth = strlen(title) * 12;
    tft.setCursor((SCREEN_WIDTH - titleWidth) / 2, 2);
    tft.println(title);

    // Back indicator
    tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.print("<");

    // Connection indicator
    tft.fillCircle(310, 10, 3, isConnected ? COLOR_GOOD : COLOR_WARN);
}

void drawGraphBorder(int x, int y, int w, int h)
{
    tft.drawRect(x, y, w, h, COLOR_GRID);
    tft.drawFastHLine(x, y + h / 2, w, COLOR_GRID);
}

void updateAllGraphs()
{
    // Map tilt angles (-45 to +45) to graph coordinates
    int tiltX_mapped = map(receivedData.tiltX * 10, -450, 450, 0, GRAPH_HEIGHT - 1);
    int tiltY_mapped = map(receivedData.tiltY * 10, -450, 450, 0, GRAPH_HEIGHT - 1);

    // Map gyro rates (-250 to +250) to graph coordinates
    int gyroX_mapped = map(receivedData.gyroX * 10, -2500, 2500, 0, GRAPH_HEIGHT - 1);
    int gyroY_mapped = map(receivedData.gyroY * 10, -2500, 2500, 0, GRAPH_HEIGHT - 1);

    // Map accel Z (-2g to +2g) to graph coordinates
    int accelZ_mapped = map(receivedData.accelZ * 100, -200, 200, 0, GRAPH_HEIGHT - 1);

    tiltX_mapped = constrain(tiltX_mapped, 0, GRAPH_HEIGHT - 1);
    tiltY_mapped = constrain(tiltY_mapped, 0, GRAPH_HEIGHT - 1);
    gyroX_mapped = constrain(gyroX_mapped, 0, GRAPH_HEIGHT - 1);
    gyroY_mapped = constrain(gyroY_mapped, 0, GRAPH_HEIGHT - 1);
    accelZ_mapped = constrain(accelZ_mapped, 0, GRAPH_HEIGHT - 1);

    graphData_TiltX[graphIndex] = tiltX_mapped;
    graphData_TiltY[graphIndex] = tiltY_mapped;
    graphData_GyroX[graphIndex] = gyroX_mapped;
    graphData_GyroY[graphIndex] = gyroY_mapped;
    graphData_AccelZ[graphIndex] = accelZ_mapped;

    graphIndex = (graphIndex + 1) % GRAPH_WIDTH;
}

void drawCompactGraph(int16_t *data, int x, int y, int w, int h, uint16_t color)
{
    // Draw only the line graph (no borders, optimized)
    for (int i = 0; i < w - 1; i++)
    {
        int displayIndex1 = (graphIndex + i) % w;
        int displayIndex2 = (graphIndex + i + 1) % w;

        int y1 = y + h - 1 - map(data[displayIndex1], 0, GRAPH_HEIGHT - 1, 0, h - 1);
        int y2 = y + h - 1 - map(data[displayIndex2], 0, GRAPH_HEIGHT - 1, 0, h - 1);

        tft.drawLine(x + i, y1, x + i + 1, y2, color);
    }
}

void drawFullGraph(int16_t *data, int graphY, uint16_t color)
{
    // Clear graph area
    tft.fillRect(GRAPH_X + 1, graphY + 1, GRAPH_WIDTH - 2, GRAPH_HEIGHT - 2, COLOR_BG);
    tft.drawFastHLine(GRAPH_X, graphY + GRAPH_HEIGHT / 2, GRAPH_WIDTH, COLOR_GRID);

    // Draw graph data
    for (int i = 0; i < GRAPH_WIDTH - 1; i++)
    {
        int displayIndex1 = (graphIndex + i) % GRAPH_WIDTH;
        int displayIndex2 = (graphIndex + i + 1) % GRAPH_WIDTH;

        int y1 = graphY + GRAPH_HEIGHT - 1 - data[displayIndex1];
        int y2 = graphY + GRAPH_HEIGHT - 1 - data[displayIndex2];

        tft.drawLine(GRAPH_X + i, y1, GRAPH_X + i + 1, y2, color);
    }
}

// Optimized graph update - only draws the newest point
void drawGraphIncremental(int16_t *data, int x, int y, int w, int h, uint16_t color)
{
    // Calculate the position of the new point
    int prevIndex = (graphIndex - 1 + w) % w;
    int currIndex = graphIndex;

    // Clear the column where we'll draw (vertical strip)
    int drawX = x + currIndex;
    tft.drawFastVLine(drawX, y, h, COLOR_BG);

    // Redraw center line
    tft.drawPixel(drawX, y + h / 2, COLOR_GRID);

    // Draw the new line segment
    if (prevIndex != currIndex)
    {
        int prevX = x + prevIndex;
        int y1 = y + h - 1 - map(data[prevIndex], 0, GRAPH_HEIGHT - 1, 0, h - 1);
        int y2 = y + h - 1 - map(data[currIndex], 0, GRAPH_HEIGHT - 1, 0, h - 1);

        tft.drawLine(prevX, y1, drawX, y2, color);
    }
}

// ---------- ORIENTATION SCREEN ----------
void drawOrientationScreen()
{
    drawScreenHeader("ORIENTATION");

    // Draw tilt indicators
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(10, 30);
    tft.print("Roll (X):");
    tft.setCursor(10, 90);
    tft.print("Pitch (Y):");

    // Draw graph borders
    drawGraphBorder(10, 150, 300, 80);

    tft.setCursor(130, 135);
    tft.print("Tilt History");
}

void updateOrientationScreen()
{
    // Only update if values changed significantly (reduces flicker)
    bool tiltXChanged = abs(receivedData.tiltX - lastDisplayedData.tiltX) > 0.1;
    bool tiltYChanged = abs(receivedData.tiltY - lastDisplayedData.tiltY) > 0.1;

    if (tiltXChanged || needsFullRedraw)
    {
        // Clear only the text area for Roll
        tft.fillRect(100, 28, 120, 25, COLOR_BG);

        // Draw Roll (X) value
        tft.setTextSize(3);
        tft.setTextColor(COLOR_TILTX);
        tft.setCursor(100, 28);
        tft.printf("%+6.1f", receivedData.tiltX);
        tft.print((char)247);

        // Clear and redraw Roll bar
        tft.fillRect(100, 60, 200, 15, COLOR_BG);
        tft.drawFastVLine(160, 55, 25, COLOR_GRID);
        int rollBarX = 160 + (int)(receivedData.tiltX * 2.5);
        rollBarX = constrain(rollBarX, 100, 300);
        tft.fillRect(min(160, rollBarX), 60, abs(rollBarX - 160), 15, COLOR_TILTX);
    }

    if (tiltYChanged || needsFullRedraw)
    {
        // Clear only the text area for Pitch
        tft.fillRect(100, 88, 120, 25, COLOR_BG);

        // Draw Pitch (Y) value
        tft.setTextSize(3);
        tft.setTextColor(COLOR_TILTY);
        tft.setCursor(100, 88);
        tft.printf("%+6.1f", receivedData.tiltY);
        tft.print((char)247);

        // Clear and redraw Pitch bar
        tft.fillRect(100, 120, 200, 15, COLOR_BG);
        tft.drawFastVLine(160, 115, 25, COLOR_GRID);
        int pitchBarX = 160 + (int)(receivedData.tiltY * 2.5);
        pitchBarX = constrain(pitchBarX, 100, 300);
        tft.fillRect(min(160, pitchBarX), 120, abs(pitchBarX - 160), 15, COLOR_TILTY);
    }

    // Draw graphs incrementally (only new data point)
    if (needsFullRedraw)
    {
        tft.fillRect(11, 151, 298, 78, COLOR_BG);
        drawCompactGraph(graphData_TiltX, 11, 151, 298, 38, COLOR_TILTX);
        drawCompactGraph(graphData_TiltY, 11, 190, 298, 38, COLOR_TILTY);
        tft.drawFastHLine(11, 170, 298, COLOR_GRID);
        tft.drawFastHLine(11, 209, 298, COLOR_GRID);
        needsFullRedraw = false;
    }
    else
    {
        // Incremental update for graphs
        drawGraphIncremental(graphData_TiltX, 11, 151, 298, 38, COLOR_TILTX);
        drawGraphIncremental(graphData_TiltY, 11, 190, 298, 38, COLOR_TILTY);
    }

    // Update last displayed values
    lastDisplayedData.tiltX = receivedData.tiltX;
    lastDisplayedData.tiltY = receivedData.tiltY;
}

// ---------- GYROSCOPE SCREEN ----------
void drawGyroScreen()
{
    drawScreenHeader("GYROSCOPE");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(10, 30);
    tft.print("Gyro X:");
    tft.setCursor(10, 70);
    tft.print("Gyro Y:");

    // Draw dual graphs
    drawGraphBorder(10, 110, 145, 55);
    drawGraphBorder(165, 110, 145, 55);

    tft.setCursor(50, 100);
    tft.print("Gyro X");
    tft.setCursor(205, 100);
    tft.print("Gyro Y");

    // Min/Max labels
    tft.setCursor(10, 175);
    tft.print("Min/Max:");
}

void updateGyroScreen()
{
    bool gyroXChanged = abs(receivedData.gyroX - lastDisplayedData.gyroX) > 0.5;
    bool gyroYChanged = abs(receivedData.gyroY - lastDisplayedData.gyroY) > 0.5;

    if (gyroXChanged || needsFullRedraw)
    {
        // Clear and update Gyro X value
        tft.fillRect(80, 28, 230, 35, COLOR_BG);
        tft.setTextSize(3);
        tft.setTextColor(COLOR_GYRO);
        tft.setCursor(80, 28);
        tft.printf("%+6.1f", receivedData.gyroX);
        tft.setTextSize(2);
        tft.print(" deg/s");
    }

    if (gyroYChanged || needsFullRedraw)
    {
        // Clear and update Gyro Y value
        tft.fillRect(80, 68, 230, 35, COLOR_BG);
        tft.setTextSize(3);
        tft.setTextColor(COLOR_GYRO);
        tft.setCursor(80, 68);
        tft.printf("%+6.1f", receivedData.gyroY);
        tft.setTextSize(2);
        tft.print(" deg/s");
    }

    // Update graphs - only new data points
    if (needsFullRedraw)
    {
        tft.fillRect(11, 111, 143, 53, COLOR_BG);
        tft.fillRect(166, 111, 143, 53, COLOR_BG);
        drawCompactGraph(graphData_GyroX, 11, 111, 143, 53, COLOR_GYRO);
        drawCompactGraph(graphData_GyroY, 166, 111, 143, 53, COLOR_GYRO);
    }

    // Min/Max values - update less frequently
    static unsigned long lastMinMaxUpdate = 0;
    if (millis() - lastMinMaxUpdate > 500 || needsFullRedraw)
    {
        tft.fillRect(10, 185, 240, 30, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(10, 185);
        tft.printf("X: %.1f / %.1f", minMaxData.gyroX_min, minMaxData.gyroX_max);
        tft.setCursor(10, 200);
        tft.printf("Y: %.1f / %.1f", minMaxData.gyroY_min, minMaxData.gyroY_max);

        // Update rate
        tft.fillRect(10, 220, 150, 15, COLOR_BG);
        tft.setTextColor(COLOR_GOOD);
        tft.setCursor(10, 220);
        tft.printf("Rate: %.1f Hz", updateRate);

        lastMinMaxUpdate = millis();
    }

    if (needsFullRedraw)
    {
        needsFullRedraw = false;
    }

    lastDisplayedData.gyroX = receivedData.gyroX;
    lastDisplayedData.gyroY = receivedData.gyroY;
}

// ---------- ACCELEROMETER SCREEN ----------
void drawAccelScreen()
{
    drawScreenHeader("ACCELEROMETER");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(10, 30);
    tft.print("Accel X:");
    tft.setCursor(10, 55);
    tft.print("Accel Y:");
    tft.setCursor(10, 80);
    tft.print("Accel Z:");

    // 3D visualization area
    tft.drawRect(10, 110, 130, 120, COLOR_GRID);
    tft.setCursor(45, 100);
    tft.print("3D Vector");

    // Z-axis graph
    drawGraphBorder(150, 110, 160, 120);
    tft.setCursor(190, 100);
    tft.print("Z History");
}

void updateAccelScreen()
{
    bool accelChanged = abs(receivedData.accelX - lastDisplayedData.accelX) > 0.02 ||
                        abs(receivedData.accelY - lastDisplayedData.accelY) > 0.02 ||
                        abs(receivedData.accelZ - lastDisplayedData.accelZ) > 0.02;

    if (accelChanged || needsFullRedraw)
    {
        // Clear and update acceleration values
        tft.fillRect(80, 28, 160, 60, COLOR_BG);

        tft.setTextSize(2);
        tft.setTextColor(0xF800); // Red for X
        tft.setCursor(80, 28);
        tft.printf("%+5.2f g", receivedData.accelX);

        tft.setTextColor(0x07E0); // Green for Y
        tft.setCursor(80, 53);
        tft.printf("%+5.2f g", receivedData.accelY);

        tft.setTextColor(0x001F); // Blue for Z
        tft.setCursor(80, 78);
        tft.printf("%+5.2f g", receivedData.accelZ);

        // 3D vector visualization
        tft.fillRect(11, 111, 128, 118, COLOR_BG);

        int centerX = 75;
        int centerY = 170;
        int scale = 40;

        // Draw axes
        tft.drawFastHLine(centerX - 50, centerY, 100, COLOR_GRID);
        tft.drawFastVLine(centerX, centerY - 50, 100, COLOR_GRID);

        // Draw acceleration vector
        int endX = centerX + (int)(receivedData.accelX * scale);
        int endY = centerY - (int)(receivedData.accelY * scale);
        endX = constrain(endX, 20, 130);
        endY = constrain(endY, 120, 220);

        tft.drawLine(centerX, centerY, endX, endY, COLOR_WARN);
        tft.fillCircle(endX, endY, 3, COLOR_WARN);

        // Z magnitude indicator
        int zRadius = abs((int)(receivedData.accelZ * 15));
        zRadius = constrain(zRadius, 2, 25);
        tft.drawCircle(centerX, centerY, zRadius, 0x001F);
    }

    // Update Z history graph
    if (needsFullRedraw)
    {
        tft.fillRect(151, 111, 158, 118, COLOR_BG);
        drawCompactGraph(graphData_AccelZ, 151, 111, 158, 118, 0x001F);
        tft.drawFastHLine(151, 170, 158, COLOR_GRID);
        needsFullRedraw = false;
    }

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

    // Servo 1 section
    tft.setCursor(10, 30);
    tft.print("SERVO 1:");
    tft.drawRect(10, 45, 300, 50, COLOR_SERVO1);

    // Servo 2 section
    tft.setCursor(10, 105);
    tft.print("SERVO 2:");
    tft.drawRect(10, 120, 300, 50, COLOR_SERVO2);

    // Performance metrics
    tft.setCursor(10, 180);
    tft.print("PERFORMANCE:");
    tft.drawRect(10, 195, 300, 40, COLOR_GRID);
}

void updateServoDetailScreen()
{
    bool servo1Changed = abs(receivedData.servo1_actual - lastDisplayedData.servo1_actual) > 0.5;
    bool servo2Changed = abs(receivedData.servo2_actual - lastDisplayedData.servo2_actual) > 0.5;

    if (servo1Changed || needsFullRedraw)
    {
        // Servo 1 details - only update changed values
        tft.fillRect(70, 48, 100, 35, COLOR_BG);
        tft.setTextSize(2);
        tft.setTextColor(COLOR_SERVO1);
        tft.setCursor(70, 48);
        tft.printf("%3d", (int)receivedData.servo1_des);
        tft.print((char)247);

        tft.setCursor(70, 66);
        tft.printf("%3d", (int)receivedData.servo1_actual);
        tft.print((char)247);

        // Error indicator
        int error1 = abs((int)(receivedData.servo1_des - receivedData.servo1_actual));
        tft.fillRect(200, 58, 110, 15, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(error1 > 5 ? COLOR_WARN : COLOR_GOOD);
        tft.setCursor(200, 58);
        tft.printf("Error: %d", error1);
        tft.print((char)247);

        // Visual bar
        tft.fillRect(150, 50, 140, 8, COLOR_BG);
        tft.drawRect(150, 50, 140, 8, COLOR_GRID);
        int bar1Pos = map((int)receivedData.servo1_actual, 0, 180, 150, 290);
        tft.fillRect(bar1Pos - 2, 48, 4, 12, COLOR_SERVO1);
    }

    if (servo2Changed || needsFullRedraw)
    {
        // Servo 2 details - only update changed values
        tft.fillRect(70, 123, 100, 35, COLOR_BG);
        tft.setTextSize(2);
        tft.setTextColor(COLOR_SERVO2);
        tft.setCursor(70, 123);
        tft.printf("%3d", (int)receivedData.servo2_des);
        tft.print((char)247);

        tft.setCursor(70, 141);
        tft.printf("%3d", (int)receivedData.servo2_actual);
        tft.print((char)247);

        // Error indicator
        int error2 = abs((int)(receivedData.servo2_des - receivedData.servo2_actual));
        tft.fillRect(200, 133, 110, 15, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(error2 > 5 ? COLOR_WARN : COLOR_GOOD);
        tft.setCursor(200, 133);
        tft.printf("Error: %d", error2);
        tft.print((char)247);

        // Visual bar
        tft.fillRect(150, 125, 140, 8, COLOR_BG);
        tft.drawRect(150, 125, 140, 8, COLOR_GRID);
        int bar2Pos = map((int)receivedData.servo2_actual, 0, 180, 150, 290);
        tft.fillRect(bar2Pos - 2, 123, 4, 12, COLOR_SERVO2);
    }

    // Performance metrics - update less frequently
    static unsigned long lastPerfUpdate = 0;
    if (millis() - lastPerfUpdate > 500 || needsFullRedraw)
    {
        int error1 = abs((int)(receivedData.servo1_des - receivedData.servo1_actual));
        int error2 = abs((int)(receivedData.servo2_des - receivedData.servo2_actual));

        tft.fillRect(15, 200, 290, 30, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(15, 200);
        tft.printf("Update Rate: %.1f Hz", updateRate);
        tft.setCursor(15, 213);
        tft.printf("Packets: %d", packetCount);

        tft.setCursor(180, 200);
        tft.print("Total Error:");
        tft.setTextColor(COLOR_GOOD);
        tft.setCursor(180, 213);
        tft.printf("%d degrees", error1 + error2);

        lastPerfUpdate = millis();
    }

    if (needsFullRedraw)
    {
        // Redraw static labels
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(15, 50);
        tft.print("Target:");
        tft.setCursor(15, 68);
        tft.print("Actual:");
        tft.setCursor(15, 125);
        tft.print("Target:");
        tft.setCursor(15, 143);
        tft.print("Actual:");

        needsFullRedraw = false;
    }

    lastDisplayedData.servo1_actual = receivedData.servo1_actual;
    lastDisplayedData.servo2_actual = receivedData.servo2_actual;
}

// ---------- DATA LOGGER SCREEN ----------
void drawDataLoggerScreen()
{
    drawScreenHeader("DATA LOGGER");

    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(10, 30);
    tft.print("MIN / MAX VALUES:");

    tft.setCursor(10, 50);
    tft.print("Tilt X:");
    tft.setCursor(10, 65);
    tft.print("Tilt Y:");
    tft.setCursor(10, 80);
    tft.print("Gyro X:");
    tft.setCursor(10, 95);
    tft.print("Gyro Y:");
    tft.setCursor(10, 110);
    tft.print("Acc X:");
    tft.setCursor(10, 125);
    tft.print("Acc Y:");
    tft.setCursor(10, 140);
    tft.print("Acc Z:");

    tft.setCursor(10, 165);
    tft.print("SESSION:");

    tft.setCursor(10, 200);
    tft.setTextColor(COLOR_GOOD);
    tft.print("Tracking active...");
}

void updateDataLoggerScreen()
{
    // Update less frequently - this is a logging screen, doesn't need high refresh
    static unsigned long lastLogUpdate = 0;
    if (millis() - lastLogUpdate < 1000 && !needsFullRedraw)
    {
        return; // Update only once per second
    }
    lastLogUpdate = millis();

    tft.fillRect(70, 50, 240, 105, COLOR_BG);

    tft.setTextSize(1);

    // Tilt data
    tft.setTextColor(COLOR_TILTX);
    tft.setCursor(70, 50);
    tft.printf("%.1f / %.1f deg", minMaxData.tiltX_min, minMaxData.tiltX_max);

    tft.setTextColor(COLOR_TILTY);
    tft.setCursor(70, 65);
    tft.printf("%.1f / %.1f deg", minMaxData.tiltY_min, minMaxData.tiltY_max);

    // Gyro data
    tft.setTextColor(COLOR_GYRO);
    tft.setCursor(70, 80);
    tft.printf("%.1f / %.1f d/s", minMaxData.gyroX_min, minMaxData.gyroX_max);
    tft.setCursor(70, 95);
    tft.printf("%.1f / %.1f d/s", minMaxData.gyroY_min, minMaxData.gyroY_max);

    // Accel data
    tft.setTextColor(0xFD20);
    tft.setCursor(70, 110);
    tft.printf("%.2f / %.2f g", minMaxData.accelX_min, minMaxData.accelX_max);
    tft.setCursor(70, 125);
    tft.printf("%.2f / %.2f g", minMaxData.accelY_min, minMaxData.accelY_max);
    tft.setCursor(70, 140);
    tft.printf("%.2f / %.2f g", minMaxData.accelZ_min, minMaxData.accelZ_max);

    // Session info
    tft.fillRect(70, 165, 240, 30, COLOR_BG);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(70, 165);
    unsigned long sessionTime = (millis() - minMaxData.startTime) / 1000;
    tft.printf("Time: %02lu:%02lu", sessionTime / 60, sessionTime % 60);
    tft.setCursor(70, 180);
    tft.printf("Packets: %d", packetCount);

    // Range indicators
    tft.fillRect(10, 215, 300, 20, COLOR_BG);
    float tiltRange = max(abs(minMaxData.tiltX_max - minMaxData.tiltX_min),
                          abs(minMaxData.tiltY_max - minMaxData.tiltY_min));
    tft.setCursor(10, 215);
    tft.setTextColor(COLOR_TEXT);
    tft.print("Max Tilt Range: ");
    tft.setTextColor(tiltRange > 30 ? COLOR_WARN : COLOR_GOOD);
    tft.printf("%.1f deg", tiltRange);

    if (needsFullRedraw)
    {
        needsFullRedraw = false;
    }
}

// ---------- 3D HORIZON SCREEN ----------
void draw3DHorizonScreen()
{
    drawScreenHeader("3D HORIZON");

    tft.drawRect(10, 30, 300, 180, COLOR_GRID);
    tft.setCursor(130, 215);
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.print("Artificial Horizon");
}

void update3DHorizonScreen()
{
    // Only redraw if orientation changed significantly
    bool orientationChanged = abs(receivedData.tiltX - lastDisplayedData.tiltX) > 0.5 ||
                              abs(receivedData.tiltY - lastDisplayedData.tiltY) > 0.5;

    if (!orientationChanged && !needsFullRedraw)
    {
        return; // Skip redraw if no significant change
    }

    // Artificial horizon display - must redraw entire area due to complex graphics
    tft.fillRect(11, 31, 298, 178, COLOR_BG);

    int centerX = 160;
    int centerY = 120;
    int horizonRadius = 80;

    // Calculate horizon line based on tilt
    float rollRad = receivedData.tiltX * PI / 180.0;
    float pitchOffset = receivedData.tiltY * 2.0;

    // Draw sky (upper half)
    tft.fillRect(11, 31, 298, (int)(89 - pitchOffset), 0x001F); // Blue

    // Draw ground (lower half)
    tft.fillRect(11, (int)(120 - pitchOffset), 298, (int)(89 + pitchOffset), 0x4208); // Brown

    // Draw horizon line
    int x1 = centerX - (int)(horizonRadius * cos(rollRad));
    int y1 = centerY - (int)(horizonRadius * sin(rollRad)) - (int)pitchOffset;
    int x2 = centerX + (int)(horizonRadius * cos(rollRad));
    int y2 = centerY + (int)(horizonRadius * sin(rollRad)) - (int)pitchOffset;

    tft.drawLine(x1, y1, x2, y2, COLOR_TEXT);
    tft.drawLine(x1, y1 + 1, x2, y2 + 1, COLOR_TEXT);

    // Draw aircraft symbol (fixed center)
    tft.drawFastHLine(centerX - 30, centerY, 60, COLOR_WARN);
    tft.drawFastVLine(centerX, centerY - 5, 10, COLOR_WARN);
    tft.fillCircle(centerX, centerY, 3, COLOR_WARN);

    // Draw pitch ladder
    for (int i = -2; i <= 2; i++)
    {
        if (i == 0)
            continue;
        int ladderY = centerY + (int)(i * 20 - pitchOffset);
        if (ladderY > 35 && ladderY < 205)
        {
            int ladderWidth = (i % 2 == 0) ? 40 : 20;
            tft.drawFastHLine(centerX - ladderWidth, ladderY, ladderWidth * 2, COLOR_GRID);
        }
    }

    // Draw roll indicator (arc at top) - skip full circle for performance
    // Only draw reference marks
    for (int angle = -60; angle <= 60; angle += 30)
    {
        float rad = angle * PI / 180.0;
        int x = centerX + (int)((horizonRadius + 5) * sin(rad));
        int y = centerY - (int)((horizonRadius + 5) * cos(rad));
        tft.fillCircle(x, y, 2, COLOR_GRID);
    }

    // Current roll indicator
    int triX = centerX + (int)((horizonRadius + 10) * sin(rollRad));
    int triY = centerY - (int)((horizonRadius + 10) * cos(rollRad));
    tft.fillCircle(triX, triY, 3, COLOR_WARN);

    // Display numeric values
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.fillRect(11, 195, 150, 12, COLOR_BG);
    tft.setCursor(15, 195);
    tft.printf("Roll: %.1f", receivedData.tiltX);
    tft.print((char)247);

    tft.fillRect(160, 195, 150, 12, COLOR_BG);
    tft.setCursor(165, 195);
    tft.printf("Pitch: %.1f", receivedData.tiltY);
    tft.print((char)247);

    lastDisplayedData.tiltX = receivedData.tiltX;
    lastDisplayedData.tiltY = receivedData.tiltY;

    if (needsFullRedraw)
    {
        needsFullRedraw = false;
    }
}

void displayConnectionLost()
{
    Serial.println("⚠ Connection lost - waiting for transmitter...");

    // Show message on screen
    tft.fillRect(60, 100, 200, 40, COLOR_WARN);
    tft.drawRect(60, 100, 200, 40, COLOR_TEXT);
    tft.setTextColor(COLOR_BG);
    tft.setCursor(95, 110);
    tft.setTextSize(2);
    tft.println("NO SIGNAL");
    tft.setTextSize(1);
    tft.setCursor(85, 128);
    tft.println("Check transmitter");

    delay(1500);
    initCurrentScreen();
}

// ---------- MANUAL MODE SCREEN ----------
void drawManualModeScreen()
{
    drawScreenHeader("MANUAL MODE");

    tft.setTextSize(2);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(50, 40);
    tft.println("JOYSTICK CONTROL");

    // Draw joystick indicator background
    tft.drawRect(85, 70, 150, 120, COLOR_GRID);
    tft.drawFastHLine(85, 130, 150, COLOR_GRID); // Center horizontal
    tft.drawFastVLine(160, 70, 120, COLOR_GRID); // Center vertical

    // Labels
    tft.setTextSize(1);
    tft.setTextColor(COLOR_TEXT);
    tft.setCursor(10, 70);
    tft.print("Pitch:");
    tft.setCursor(250, 70);
    tft.print("Roll:");

    tft.setCursor(10, 200);
    tft.print("Elevons:");

    // Instructions
    tft.setTextSize(1);
    tft.setTextColor(0x7BEF);
    tft.setCursor(30, 225);
    tft.println("Joystick controls servos directly");
}

void updateManualModeScreen()
{
    // Read joystick continuously
    int rawX = analogRead(JOYSTICK_VRX);
    int rawY = analogRead(JOYSTICK_VRY);

    // Normalize to -1.0 to +1.0 with deadzone
    const int deadzone = 200;
    float pitchCmd = 0.0;
    float rollCmd = 0.0;

    int deltaY = rawY - joystickCenterY;
    int deltaX = rawX - joystickCenterX;

    if (abs(deltaY) > deadzone)
    {
        pitchCmd = (float)deltaY / 2048.0;
        pitchCmd = constrain(pitchCmd, -1.0, 1.0);
    }

    if (abs(deltaX) > deadzone)
    {
        rollCmd = (float)deltaX / 2048.0;
        rollCmd = constrain(rollCmd, -1.0, 1.0);
    }

    // Send control data to transmitter
    controlData.manualMode = true;
    controlData.pitchCommand = pitchCmd;
    controlData.rollCommand = rollCmd;
    controlData.timestamp = millis();

    esp_err_t result = esp_now_send(transmitterAddress, (uint8_t *)&controlData, sizeof(controlData));

    // Update visual indicators
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 50) // 20 FPS
    {
        // Clear joystick indicator area
        tft.fillRect(86, 71, 148, 118, COLOR_BG);
        tft.drawFastHLine(85, 130, 150, COLOR_GRID); // Center horizontal
        tft.drawFastVLine(160, 70, 120, COLOR_GRID); // Center vertical

        // Draw joystick position
        int displayX = 160 + (int)(rollCmd * 70);
        int displayY = 130 - (int)(pitchCmd * 55);
        displayX = constrain(displayX, 90, 230);
        displayY = constrain(displayY, 75, 185);

        tft.fillCircle(displayX, displayY, 8, COLOR_WARN);
        tft.drawCircle(displayX, displayY, 9, COLOR_TEXT);

        // Display numeric values
        tft.fillRect(10, 85, 70, 100, COLOR_BG);
        tft.fillRect(250, 85, 70, 100, COLOR_BG);

        tft.setTextSize(2);
        tft.setTextColor(COLOR_TILTY);
        tft.setCursor(10, 90);
        tft.printf("%+.2f", pitchCmd);

        tft.setTextColor(COLOR_TILTX);
        tft.setCursor(250, 90);
        tft.printf("%+.2f", rollCmd);

        // Display servo angles (from telemetry)
        tft.fillRect(80, 200, 230, 20, COLOR_BG);
        tft.setTextSize(1);
        tft.setTextColor(COLOR_SERVO1);
        tft.setCursor(80, 200);
        tft.printf("R: %d", (int)receivedData.servo1_actual);
        tft.print((char)247);

        tft.setTextColor(COLOR_SERVO2);
        tft.setCursor(160, 200);
        tft.printf("L: %d", (int)receivedData.servo2_actual);
        tft.print((char)247);

        // Connection status
        tft.fillRect(80, 215, 150, 10, COLOR_BG);
        tft.setTextColor(result == ESP_OK ? COLOR_GOOD : COLOR_WARN);
        tft.setCursor(80, 215);
        tft.print(result == ESP_OK ? "TX: OK" : "TX: FAIL");

        lastDisplayUpdate = millis();
    }
}

// Exit manual mode - send command to resume auto stabilization
void exitManualMode()
{
    controlData.manualMode = false;
    controlData.pitchCommand = 0.0;
    controlData.rollCommand = 0.0;
    controlData.timestamp = millis();

    esp_now_send(transmitterAddress, (uint8_t *)&controlData, sizeof(controlData));
    Serial.println("→ Exiting manual mode - transmitter resuming auto stabilization");
}

// ==================== ADDITIONAL INFO ====================
/*
 * MENU-DRIVEN TELEMETRY SYSTEM
 * =============================
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
 * 1. ORIENTATION: Visual tilt indicators with history graphs
 * 2. GYROSCOPE DATA: Real-time gyro readings with dual graphs
 * 3. ACCELEROMETER: 3D acceleration vector visualization
 * 4. SERVO DETAILS: Detailed servo position tracking and errors
 * 5. DATA LOGGER: Min/Max value tracking for all sensors
 * 6. 3D HORIZON: Artificial horizon with pitch/roll display
 *
 * FEATURES:
 * ---------
 * - High refresh rate (~33 FPS) for smooth updates
 * - Optimized screen redraw (only updates data areas)
 * - Min/Max tracking across all sensor data
 * - Multiple visualization styles (graphs, bars, 3D)
 * - Connection status on all screens
 * - Real-time performance metrics
 *
 * COLORS:
 * -------
 * Cyan: Tilt X / Roll
 * Magenta: Tilt Y / Pitch
 * Green: Servo 1 & Connection OK
 * Yellow: Servo 2
 * Orange: Gyroscope
 * Red: X-axis / Warnings
 * Blue: Z-axis / Sky
 *
 * NOTES:
 * ------
 * - Transmitter must send accelX, accelY, accelZ in TelemetryData
 * - Screen updates optimized for minimal flicker
 * - All graphs show historical data (280 samples)
 * - Data logger resets on receiver restart
 */
