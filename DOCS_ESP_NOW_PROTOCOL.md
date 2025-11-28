# ESP-NOW Communication Protocol - Deep Dive

## Table of Contents
1. [Protocol Overview](#protocol-overview)
2. [ESP-NOW Fundamentals](#esp-now-fundamentals)
3. [Physical Layer](#physical-layer)
4. [Data Link Layer](#data-link-layer)
5. [Packet Structure](#packet-structure)
6. [Pairing and Discovery](#pairing-and-discovery)
7. [Performance Analysis](#performance-analysis)
8. [Comparison with Alternatives](#comparison-with-alternatives)

---

## Protocol Overview

### What is ESP-NOW?

ESP-NOW is a **connectionless WiFi communication protocol** developed by Espressif Systems for ESP8266 and ESP32 chips.

**Key characteristics:**
- Peer-to-peer (no router/AP needed)
- Low latency (~3ms)
- Low power
- Small packets (up to 250 bytes)
- Based on vendor-specific WiFi action frames
- Works alongside WiFi (with limitations)

```
Traditional WiFi:        ESP-NOW:
┌──────────┐            ┌──────────┐
│ Device A │            │ Device A │
└────┬─────┘            └────┬─────┘
     │                       │
     │  Association          │  Direct
     ▼  Authentication       ▼  Transmission
┌─────────┐            ┌──────────┐
│ Router  │            │ Device B │
│   AP    │            └──────────┘
└────┬────┘
     │
     ▼
┌──────────┐
│ Device B │
└──────────┘

3-way process           2-way process
Higher latency          Lower latency
More overhead           Less overhead
```

---

## ESP-NOW Fundamentals

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    ESP-NOW PROTOCOL STACK                        │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                      APPLICATION LAYER                           │
│  Your Code: Pack data, call esp_now_send()                      │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                       ESP-NOW API LAYER                          │
│  - esp_now_init()                                                │
│  - esp_now_send()                                                │
│  - esp_now_register_send_cb()                                    │
│  - esp_now_register_recv_cb()                                    │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ESP-NOW PROTOCOL LAYER                        │
│  - Packet formatting                                             │
│  - Encryption (AES-128, optional)                                │
│  - Peer management                                               │
│  - Retransmission (if enabled)                                   │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                      WiFi MAC LAYER                              │
│  - Vendor-specific action frames (type 0xD0)                     │
│  - CRC checking                                                  │
│  - MAC addressing                                                │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    WiFi PHYSICAL LAYER                           │
│  - 2.4 GHz radio                                                 │
│  - OFDM/DSSS modulation                                          │
│  - CSMA/CA channel access                                        │
└─────────────────────────────────────────────────────────────────┘
```

### Roles

ESP-NOW is **role-agnostic**, but we assign logical roles:

```
TRANSMITTER (Aircraft):
- Sends telemetry data
- Initiates communication
- Master role (in our application)

RECEIVER (Ground Station):
- Listens for data
- Displays received information
- Slave role (in our application)

Note: Either device can send to the other
      (ESP-NOW is bidirectional)
```

### MAC Address Based Addressing

Every ESP32 has a unique MAC address:
```
Format: XX:XX:XX:XX:XX:XX (6 bytes)
Example: FC:E8:C0:E0:D2:F4

Structure:
├─ Bytes 0-2: Manufacturer OUI (Espressif: FC:E8:C0)
└─ Bytes 3-5: Device unique ID (E0:D2:F4)

To read your MAC:
```cpp
WiFi.mode(WIFI_STA);
Serial.println(WiFi.macAddress());
```

Output: "FC:E8:C0:E0:D2:F4"
```

---

## Physical Layer

### Radio Characteristics

```
Frequency Band: 2.4 GHz ISM band
Channels: 1-14 (depends on region)
  - US: 1-11
  - Europe: 1-13
  - Japan: 1-14
  
Channel spacing: 5 MHz
Channel width: 20 MHz or 40 MHz

Transmit Power: Adjustable 0-20 dBm
Default: ~17 dBm (50 mW)

Range:
- Indoor (walls): 30-100m
- Outdoor (line of sight): 200-400m
- Depends on obstacles, interference

Data Rates (WiFi b/g/n):
- 802.11b: 1-11 Mbps (DSSS)
- 802.11g: 6-54 Mbps (OFDM)
- 802.11n: up to 150 Mbps (MIMO, not for ESP-NOW)

ESP-NOW typically uses: 802.11b or 802.11g
```

### Modulation

```
OFDM (Orthogonal Frequency Division Multiplexing):

Frequency
 ▲
 │  ║ ║ ║ ║ ║ ║ ║ ║    Each bar = subcarrier
 │  ║ ║ ║ ║ ║ ║ ║ ║    52 subcarriers for 802.11g
 │  ║ ║ ║ ║ ║ ║ ║ ║    20 MHz total bandwidth
 └──────────────────► Time

Advantages:
- Resistant to multipath fading
- High spectral efficiency
- Robust against interference
```

### CSMA/CA (Collision Avoidance)

```
Channel Access Method:

1. Listen before transmit (carrier sense)
2. If busy → wait random backoff time
3. If clear → transmit
4. Wait for ACK
5. If no ACK → retransmit

Timeline:
Sender:    │  Listen  │XXXXX TX XXXXX│  Wait ACK │
           └──────────┴──────────────┴───────────┘
                                     
Receiver:                 │ RX │ Process │ ACK │
           └──────────────┴────┴─────────┴─────┘

DIFS: Distributed Inter-Frame Space (wait time)
SIFS: Short IFS (ACK wait time)
```

---

## Data Link Layer

### Frame Format

ESP-NOW uses **IEEE 802.11 Action Frames**:

```
┌─────────────────────────────────────────────────────────────────┐
│                  802.11 ACTION FRAME STRUCTURE                   │
└─────────────────────────────────────────────────────────────────┘

╔═══════════╦════════════╦══════════╦═══════════╦═════════╦═══════╗
║  Frame    ║  Duration  ║  Dest.   ║  Source   ║  BSSID  ║  Seq  ║
║  Control  ║  / ID      ║  Address ║  Address  ║         ║  Ctrl ║
║  (2 B)    ║  (2 B)     ║  (6 B)   ║  (6 B)    ║  (6 B)  ║ (2 B) ║
╚═══════════╩════════════╩══════════╩═══════════╩═════════╩═══════╝
╔═══════════════════════════════════════════════════════════════╗
║                    Frame Body (0-250 bytes)                    ║
║  ┌──────────────┬──────────────────────────────┐              ║
║  │ Category (1B)│    ESP-NOW Payload           │              ║
║  │  (Vendor)    │    (Your data struct)        │              ║
║  └──────────────┴──────────────────────────────┘              ║
╚═══════════════════════════════════════════════════════════════╝
╔═══════════╗
║    FCS    ║  Frame Check Sequence (CRC-32)
║   (4 B)   ║
╚═══════════╝

Total overhead: 24 bytes (header) + 5 bytes (action frame) + 4 bytes (FCS) = 33 bytes
Maximum payload: 250 bytes (ESP-NOW limit)
Maximum total frame: 283 bytes
```

### Our Packet Structure

```cpp
typedef struct {
    float pitch;               // 4 bytes
    float roll;                // 4 bytes
    float right_elevon_des;    // 4 bytes
    float right_elevon_actual; // 4 bytes
    float left_elevon_des;     // 4 bytes
    float left_elevon_actual;  // 4 bytes
    float gyro_pitch;          // 4 bytes
    float gyro_roll;           // 4 bytes
    float accelX;              // 4 bytes
    float accelY;              // 4 bytes
    float accelZ;              // 4 bytes
    uint32_t timestamp;        // 4 bytes
} TelemetryData;  // Total: 48 bytes
```

**Memory layout:**
```
Offset  Field                Type     Value (example)
─────────────────────────────────────────────────────
0x00    pitch                float    5.2°
0x04    roll                 float    -2.1°
0x08    right_elevon_des     float    0.0°
0x0C    right_elevon_actual  float    85.0°
0x10    left_elevon_des      float    0.0°
0x14    left_elevon_actual   float    105.0°
0x18    gyro_pitch           float    0.8 °/s
0x1C    gyro_roll            float    -0.3 °/s
0x20    accelX               float    0.15 g
0x24    accelY               float    0.03 g
0x28    accelZ               float    1.02 g
0x2C    timestamp            uint32   12458 ms
─────────────────────────────────────────────────────
Total: 48 bytes (0x30)
```

### Byte Order (Endianness)

```
ESP32 is LITTLE ENDIAN:

float value = 5.2
IEEE 754 binary: 0x40A66666

Memory layout:
Address   Value
───────────────
0x100     0x66  ← LSB
0x101     0x66
0x102     0xA6
0x103     0x40  ← MSB

Both TX and RX are ESP32 (little endian)
→ No conversion needed! ✓
```

---

## Pairing and Discovery

### Pairing Process

Unlike Bluetooth, ESP-NOW requires **manual pairing** via MAC address:

```
┌─────────────────────────────────────────────────────────────────┐
│                    ESP-NOW PAIRING SEQUENCE                      │
└─────────────────────────────────────────────────────────────────┘

STEP 1: FIND RECEIVER MAC ADDRESS
──────────────────────────────────
Receiver (run first):
```cpp
WiFi.mode(WIFI_STA);
Serial.println(WiFi.macAddress());
```
Output: FC:E8:C0:E0:D2:F4  ← Copy this!


STEP 2: ADD PEER TO TRANSMITTER
────────────────────────────────
Transmitter (update code):
```cpp
uint8_t receiverAddress[] = {0xFC, 0xE8, 0xC0, 0xE0, 0xD2, 0xF4};
                             └── Paste MAC here (hex format)

esp_now_peer_info_t peerInfo;
memcpy(peerInfo.peer_addr, receiverAddress, 6);
peerInfo.channel = 0;  // Auto (same as WiFi channel)
peerInfo.encrypt = false;  // No encryption

esp_now_add_peer(&peerInfo);  // Register the peer
```


STEP 3: COMMUNICATION ACTIVE
────────────────────────────
Transmitter can now send:
```cpp
esp_now_send(receiverAddress, (uint8_t*)&telemetry, sizeof(telemetry));
```

Receiver automatically receives (no pairing needed on RX side)
```

### Channel Selection

```
WiFi Channel: Frequency channel for communication

Options:
1. Same channel as WiFi AP (if connected)
2. Channel 0 = Auto (uses current WiFi channel, or channel 1)
3. Specific channel (1-14)

Our setup:
  peerInfo.channel = 0;  // Auto
  
This means:
- No WiFi AP connected → Uses channel 1
- Both devices must be on same channel
- ESP-NOW and WiFi can coexist on same channel
```

### Encryption

```
ESP-NOW supports AES-128 encryption:

Encrypted Mode:
```cpp
peerInfo.encrypt = true;
// Must also set: esp_now_set_pmk() with 16-byte key
```

Our setup: UNENCRYPTED
```cpp
peerInfo.encrypt = false;
```

Why no encryption?
- Lower latency
- No sensitive data (just aircraft angles)
- Simpler setup
- Slightly better range

Use encryption if:
- Sending sensitive data
- Need authentication
- Worried about eavesdropping
```

---

## Implementation Details

### Initialization Sequence

```cpp
// TRANSMITTER SETUP:
// ══════════════════

// 1. Set WiFi mode
WiFi.mode(WIFI_STA);
WiFi.disconnect();
delay(100);

// 2. Initialize ESP-NOW
esp_now_init();

// 3. Register callback (optional, for send status)
esp_now_register_send_cb(OnDataSent);

// 4. Add receiver as peer
memcpy(peerInfo.peer_addr, receiverAddress, 6);
peerInfo.channel = 0;
peerInfo.encrypt = false;
esp_now_add_peer(&peerInfo);

// Done! Can now send.


// RECEIVER SETUP:
// ═══════════════

// 1. Set WiFi mode
WiFi.mode(WIFI_STA);
WiFi.disconnect();
delay(100);

// 2. Initialize ESP-NOW
esp_now_init();

// 3. Register receive callback
esp_now_register_recv_cb(OnDataRecv);

// Done! Can now receive.
```

### Sending Data

```cpp
// Transmitter (every 20ms):

// Pack data into struct
telemetry.pitch = pitch;
telemetry.roll = roll;
// ... (12 fields total)
telemetry.timestamp = millis();

// Send packet
esp_err_t result = esp_now_send(
    receiverAddress,           // Destination MAC
    (uint8_t*)&telemetry,      // Data pointer (cast to byte array)
    sizeof(telemetry)          // Size (48 bytes)
);

// Check result
if (result == ESP_OK) {
    Serial.println("Sent OK");
} else {
    Serial.println("Send failed!");
}
```

**Return values:**
- `ESP_OK` (0): Queued successfully
- `ESP_ERR_ESPNOW_NOT_INIT`: ESP-NOW not initialized
- `ESP_ERR_ESPNOW_ARG`: Invalid argument
- `ESP_ERR_ESPNOW_INTERNAL`: Internal error
- `ESP_ERR_ESPNOW_NO_MEM`: Out of memory
- `ESP_ERR_ESPNOW_NOT_FOUND`: Peer not found

**Note:** `ESP_OK` means "queued", not "delivered"!
Use send callback to confirm delivery.

### Receiving Data

```cpp
// Receiver callback:

void OnDataRecv(const esp_now_recv_info_t *recv_info,
                const uint8_t *incomingData,
                int len)
{
    // This runs in INTERRUPT CONTEXT (Core 1)
    // Keep it SHORT and FAST!
    
    // Copy received data to global struct
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    
    // Set flag for main loop
    dataReceived = true;
    lastDataTime = millis();
    packetsReceived++;
    
    // DO NOT:
    // - Serial.print() (too slow, may crash)
    // - Long calculations
    // - Blocking operations
    // - Access shared data without protection
}
```

**Callback parameters:**
- `recv_info`: Contains sender MAC, RSSI, channel
- `incomingData`: Pointer to received bytes
- `len`: Number of bytes received

**recv_info structure:**
```cpp
typedef struct {
    uint8_t src_addr[6];  // Sender MAC address
    int8_t rssi;          // Signal strength (dBm)
    uint8_t channel;      // WiFi channel
} esp_now_recv_info_t;
```

### Send Callback

```cpp
void OnDataSent(const wifi_tx_info_t *tx_info, 
                esp_now_send_status_t status)
{
    // Called after transmission attempt
    
    if (status == ESP_NOW_SEND_SUCCESS) {
        // Packet delivered ✓
        successCount++;
    } else {
        // Packet failed ✗
        failCount++;
    }
}
```

**wifi_tx_info_t:**
```cpp
typedef struct {
    bool ack;             // ACK received?
    uint8_t retries;      // Number of retries
} wifi_tx_info_t;
```

---

## Performance Analysis

### Throughput

```
Packet size: 48 bytes
Send rate: 50 Hz (every 20ms)

Data rate:
  48 bytes/packet × 50 packets/sec = 2,400 bytes/sec
  = 19.2 kbps (kilobits per second)

ESP-NOW capacity: ~250 kbps (typical)
Usage: 7.7% (very light!)

Maximum packet rate:
  250 kbps ÷ (48 bytes × 8 bits/byte) ≈ 650 packets/sec
  Our 50 Hz is well below limit ✓
```

### Latency

```
┌─────────────────────────────────────────────────────────────────┐
│                      LATENCY BREAKDOWN                           │
└─────────────────────────────────────────────────────────────────┘

Transmitter:
  1. esp_now_send() call        0.05 ms  (queue packet)
  2. Wait for channel           0-5 ms   (CSMA/CA backoff)
  3. Transmit packet            0.5 ms   (at 1 Mbps: 283 bytes)
  4. Propagation delay          < 0.001 ms (speed of light)
  ────────────────────────────────────────
  Total TX side:                0.5-5.5 ms

Receiver:
  5. Receive packet             0.5 ms
  6. CRC check                  0.05 ms
  7. Callback triggered         0.05 ms
  ────────────────────────────────────────
  Total RX side:                0.6 ms

TOTAL LATENCY:                  1-6 ms (typical: 3 ms)

Comparison:
  ESP-NOW:        1-6 ms      ✓ Very fast
  WiFi TCP:       10-50 ms    (connection overhead)
  Bluetooth:      5-20 ms     (pairing, profiles)
  Traditional RC: 10-20 ms    (2.4GHz RC systems)
```

### Packet Loss

```
Causes of packet loss:
1. Out of range (signal too weak)
2. Interference (other WiFi, microwave, etc)
3. Obstacles (walls, metal)
4. Buffer overflow (sending too fast)

Measured packet loss (typical):
┌───────────────────┬──────────────┐
│ Condition         │ Loss Rate    │
├───────────────────┼──────────────┤
│ Clear line-of-sight│ < 0.1%      │
│ Indoor (same room)│ 0.5-1%       │
│ Through walls     │ 2-5%         │
│ Long range        │ 5-20%        │
│ Heavy interference│ 10-50%       │
└───────────────────┴──────────────┘

Our telemetry system:
- Loss rate: ~1% (typical indoor)
- 50 Hz update rate
- Effective rate: ~49-50 Hz (still very smooth)
- No retransmission (not needed for telemetry)
```

### RSSI (Signal Strength)

```
RSSI: Received Signal Strength Indicator (dBm)

Scale:
  -30 dBm: Excellent (very close)
  -50 dBm: Good
  -70 dBm: Weak (usable)
  -80 dBm: Very weak (marginal)
  -90 dBm: Unusable (packet loss)

Access via callback:
```cpp
void OnDataRecv(const esp_now_recv_info_t *recv_info, ...) {
    int8_t rssi = recv_info->rssi;
    Serial.printf("Signal: %d dBm\n", rssi);
}
```

Typical values:
  Same room: -40 to -60 dBm
  Through wall: -60 to -75 dBm
  Outdoor 100m: -70 to -85 dBm
```

---

## Reliability Features

### Error Detection

```
1. CRC-32 (Frame Check Sequence)
   - Automatic (part of 802.11 MAC)
   - Detects bit errors
   - Corrupted packets discarded
   - No delivery to application

2. Length checking
   - ESP-NOW checks packet size
   - Rejects oversized packets
   - Rejects undersized packets

3. MAC address filtering
   - Only receives from paired peers
   - Rejects unknown senders
```

### No Automatic Retransmission

```
ESP-NOW (by default): No ARQ (Automatic Repeat Request)

Packet lost → Lost forever
Application doesn't know (unless using send callback)

Why no retransmission?
- Telemetry doesn't need it (new data coming)
- Lower latency
- Simpler protocol

When to enable retransmission?
- Command/control data (must arrive)
- Configuration changes
- Critical alerts

How to enable (not used in our code):
- Not directly supported
- Must implement in application layer
```

### Connection Monitoring

```
We implement application-level monitoring:

Transmitter: Sends timestamp in every packet
```cpp
telemetry.timestamp = millis();
```

Receiver: Checks time since last packet
```cpp
unsigned long timeSinceLastData = millis() - lastDataTime;
bool connected = (timeSinceLastData < 1000);  // 1 sec timeout
```

Visual feedback:
  Connected: Green dot, "CONN", shows Hz
  Disconnected: Red dot, "LOST", shows "--Hz"
```

---

## Comparison with Alternatives

### ESP-NOW vs WiFi UDP

```
┌──────────────────┬─────────────┬──────────────┐
│ Feature          │ ESP-NOW     │ WiFi UDP     │
├──────────────────┼─────────────┼──────────────┤
│ Latency          │ 1-6 ms      │ 10-50 ms     │
│ Setup            │ MAC address │ IP/Router    │
│ Range            │ Good        │ Same         │
│ Power            │ Low         │ Higher       │
│ Overhead         │ Low         │ Higher       │
│ Payload          │ 250 bytes   │ 1472 bytes   │
│ Reliability      │ Moderate    │ Moderate     │
│ Infrastructure   │ None        │ Router needed│
└──────────────────┴─────────────┴──────────────┘

Use ESP-NOW when:
✓ Low latency needed
✓ Small packets
✓ Peer-to-peer
✓ No router available
✓ Battery powered

Use WiFi UDP when:
✓ Large data transfers
✓ Multiple devices
✓ Internet connection needed
✓ Existing WiFi network
```

### ESP-NOW vs Bluetooth

```
┌──────────────────┬─────────────┬──────────────┐
│ Feature          │ ESP-NOW     │ Bluetooth    │
├──────────────────┼─────────────┼──────────────┤
│ Frequency        │ 2.4 GHz     │ 2.4 GHz      │
│ Range            │ 200-400m    │ 10-100m      │
│ Latency          │ 1-6 ms      │ 5-20 ms      │
│ Pairing          │ Manual MAC  │ Automatic    │
│ Power            │ Low         │ Very low     │
│ Throughput       │ 250 kbps    │ 1-3 Mbps     │
│ Compatibility    │ ESP only    │ Universal    │
└──────────────────┴─────────────┴──────────────┘

Use ESP-NOW when:
✓ ESP32-to-ESP32
✓ Longer range needed
✓ Lower latency critical
✓ Simple setup

Use Bluetooth when:
✓ Mobile phone app
✓ Compatibility important
✓ Audio streaming
✓ Standard profiles needed
```

### ESP-NOW vs 2.4GHz RC

```
┌──────────────────┬─────────────┬──────────────┐
│ Feature          │ ESP-NOW     │ RC (2.4GHz)  │
├──────────────────┼─────────────┼──────────────┤
│ Latency          │ 1-6 ms      │ 5-20 ms      │
│ Channels         │ 8-16 ch     │ Unlimited    │
│ Range            │ 200-400m    │ 500-1000m    │
│ Failsafe         │ Manual      │ Built-in     │
│ Cost             │ $2-5        │ $20-200      │
│ Complexity       │ High        │ Low (plug&play)│
│ Flexibility      │ Very high   │ Limited      │
└──────────────────┴─────────────┴──────────────┘

ESP-NOW is NOT a replacement for RC transmitter!
(We use it for telemetry only, not control)
```

---

## Troubleshooting

### Common Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| **No data received** | Wrong MAC address | Verify RX MAC, update TX code |
| | Not paired | Check esp_now_add_peer() |
| | Different WiFi channel | Set same channel on both |
| **Intermittent reception** | Weak signal | Reduce distance, remove obstacles |
| | Interference | Change WiFi channel |
| | Antenna blocked | Reorient devices |
| **ESP_ERR_ESPNOW_NOT_INIT** | Init failed | Check esp_now_init() return value |
| | Called before init | Move esp_now_send() after init |
| **High packet loss** | Out of range | Move closer |
| | Metal obstruction | Remove/relocate |
| | 2.4GHz interference | Use WiFi analyzer, change channel |
| **Send always fails** | Peer not added | Check esp_now_add_peer() |
| | Invalid MAC | Verify MAC format |
| | Buffer full | Reduce send rate |

### Diagnostic Tools

```cpp
// 1. Check initialization:
esp_err_t result = esp_now_init();
if (result != ESP_OK) {
    Serial.printf("Init failed: %d\n", result);
}

// 2. Verify MAC address:
Serial.printf("My MAC: %s\n", WiFi.macAddress().c_str());

// 3. Monitor send status:
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    Serial.printf("Send %s (retries: %d)\n", 
                  status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL",
                  tx_info->retries);
}

// 4. Check RSSI:
void OnDataRecv(const esp_now_recv_info_t *recv_info, ...) {
    Serial.printf("RSSI: %d dBm\n", recv_info->rssi);
}

// 5. Count packets:
static unsigned long total = 0, lost = 0;
// In receive callback: total++;
// Check if timestamp jumped: if (gap > 20ms) lost++;
float lossRate = 100.0 * lost / total;
Serial.printf("Loss rate: %.1f%%\n", lossRate);
```

---

## Advanced Topics

### Multiple Peers

```cpp
// Transmitter can send to multiple receivers:

uint8_t receiver1[] = {0xFC, 0xE8, 0xC0, 0xE0, 0xD2, 0xF4};
uint8_t receiver2[] = {0x24, 0x6F, 0x28, 0x7A, 0xBC, 0xDE};

// Add both as peers:
esp_now_peer_info_t peer1, peer2;
memcpy(peer1.peer_addr, receiver1, 6);
memcpy(peer2.peer_addr, receiver2, 6);
esp_now_add_peer(&peer1);
esp_now_add_peer(&peer2);

// Send to specific receiver:
esp_now_send(receiver1, data, len);
esp_now_send(receiver2, data, len);

// Or broadcast to all peers:
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_send(broadcastAddr, data, len);  // All peers receive
```

### Bidirectional Communication

```cpp
// Aircraft can send telemetry AND receive commands:

// Setup (on aircraft):
esp_now_register_send_cb(OnDataSent);   // For telemetry
esp_now_register_recv_cb(OnDataRecv);   // For commands

// Ground station sends command:
CommandData cmd;
cmd.mode = ARM;
cmd.setpoint_pitch = 5.0;
esp_now_send(aircraftMAC, (uint8_t*)&cmd, sizeof(cmd));

// Aircraft receives:
void OnDataRecv(...) {
    CommandData cmd;
    memcpy(&cmd, incomingData, sizeof(cmd));
    // Execute command
}
```

### Power Management

```cpp
// Reduce power consumption:

// 1. Lower TX power:
esp_wifi_set_max_tx_power(40);  // Range: 8-84 (0.5dBm steps)
                                 // Default: 80 (20dBm)
                                 // 40 = 10dBm (lower power, shorter range)

// 2. Modem sleep (if not always transmitting):
esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // Light sleep between packets

// 3. Reduce send rate:
// Send at 10 Hz instead of 50 Hz → 5x power savings
```

---

## Security Considerations

### Encryption

```cpp
// Enable AES-128 encryption:

// 1. Set primary master key (PMK):
uint8_t pmk[16] = {0x00, 0x11, 0x22, ...};  // 128-bit key
esp_now_set_pmk(pmk);

// 2. Set local master key (LMK) for peer:
uint8_t lmk[16] = {0xAA, 0xBB, 0xCC, ...};
peerInfo.lmk = lmk;
peerInfo.encrypt = true;

// 3. Add encrypted peer:
esp_now_add_peer(&peerInfo);

// Now all packets to/from this peer are encrypted!
```

### Vulnerabilities

```
WITHOUT ENCRYPTION:
❌ Packets can be sniffed (Wireshark, etc)
❌ Anyone can receive telemetry
❌ No authentication (spoofing possible)
❌ Replay attacks possible

WITH ENCRYPTION:
✓ Packets encrypted (AES-128)
✓ Only paired devices can decrypt
⚠ Still no authentication (pre-shared key)
⚠ Key must be securely distributed
```

### Best Practices

```
1. Use encryption for sensitive data
2. Change default keys
3. Limit range (lower TX power)
4. Add application-layer authentication
5. Implement timeout/failsafe
6. Validate received data (range checks)
7. Add sequence numbers (detect replay)
```

---

## Summary

ESP-NOW is a **low-latency, connectionless WiFi protocol** ideal for:
- ✅ Real-time telemetry
- ✅ Peer-to-peer communication
- ✅ Small packet sizes (< 250 bytes)
- ✅ Low power applications
- ✅ No infrastructure needed

**Our implementation:**
- Transmitter: Sends 48-byte packets at 50 Hz
- Receiver: Displays data in real-time
- Latency: ~3ms (excellent!)
- Range: 30-100m indoors
- Packet loss: < 1% (typical)
- No encryption (not needed for telemetry)

**Key advantages over alternatives:**
- Lower latency than WiFi UDP
- Longer range than Bluetooth
- Simpler setup than RC systems
- Very cost-effective ($2-5 per node)

---

**END OF ESP-NOW PROTOCOL DOCUMENTATION**

