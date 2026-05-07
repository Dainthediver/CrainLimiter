# CrainLimiter Quick Reference

**Last Updated:** May 4, 2026

## Hardware Configuration

### ESP32 + DW1000 BU-01 Pinout
```
PIN_RST  = 27  // Reset pin for DW1000
PIN_IRQ  = 34  // Interrupt pin for DW1000
PIN_SS   = 4   // SPI Chip Select pin for DW1000
```

### Device Constants
```
MAX_LINKED_DEVICES      = 5
DISTANCE_MEASUREMENTS  = 16
LINK_TIMEOUT_MS        = 5000
LISTEN_PERIOD_MS       = 10000
```

## Message Protocol

| Message Type | Value | Description |
|--------------|-------|-------------|
| MSG_HANDSHAKE_REQUEST | 0x01 | "Are you available?" |
| MSG_HANDSHAKE_ACCEPT | 0x02 | "Yes, I'm available" |
| MSG_HANDSHAKE_REJECT | 0x03 | "No, I'm busy" |
| MSG_PING_REQUEST | 0x04 | Distance measurement ping |
| MSG_PING_RESPONSE | 0x05 | Distance measurement response |
| MSG_DISTANCE_COMPLETE | 0x06 | All measurements complete |
| MSG_UNLINK | 0x07 | Ending the link |

## State Machine

```
STATE_IDLE              → Device is idle, listening for requests
STATE_LINKING           → Device is attempting to establish link
STATE_LINKED            → Device is in an active measurement session
STATE_MEASURING         → Device is performing distance measurements
STATE_WAITING_RESPONSE  → Device is waiting for response
STATE_UNLINKING         → Device is ending a link session
```

## Configuration Variables

Each puck needs:
- **Channel:** Wireless channel for communication
- **Priority:** Device priority (1-5)
- **Direction1:** Custom variable (TBD)
- **Direction2:** Custom variable (TBD)

## Distance Measurement Protocol

1. **Handshake:** Device A → "Are you available?" → Device B → "Yes"
2. **Link Mode:** Both devices enter linked mode, ignore other requests
3. **Measurement:** Lower-numbered device initiates, performs 16 round-trip measurements
4. **Average:** Both devices average their 16 measurements
5. **Store:** Result stored in `distanceToDeviceX` variable
6. **Unlink:** Devices unlink and repeat with other devices
7. **Timeout:** Timer expires, then listen for new devices

## Library Required

**arduino-dw1000**
- Source: https://github.com/thotro/arduino-dw1000
- PlatformIO: Add to `platformio.ini` lib_deps

## PlatformIO Setup

```ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    https://github.com/thotro/arduino-dw1000.git
monitor_speed = 115200
```

## Serial Monitor Settings

- **Baud Rate:** 115200
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1

## WiFi AP Configuration (Future)

```
SSID: CrainLimiter-Puck-XX
IP: 192.168.4.1
Port: 80 (HTTP)
```

## Hardware UI Components

- **1 Button:** Long press = config mode, short press = cycle displays
- **2 LEDs:** Green (connected), Red (error/low battery)

## Test Checklist

- [ ] Single device initialization
- [ ] Two-device handshake and ranging
- [ ] Multi-device (3-5) sequential ranging
- [ ] Accuracy verification (known distances)
- [ ] Timeout and error handling
- [ ] Power consumption measurement
- [ ] Environmental robustness testing

## Key Functions

```cpp
void initializeDW1000();              // Setup DW1000 chip
void handleReceivedMessage();          // Process incoming messages
void sendHandshakeRequest(uint8_t);   // Initiate link
void performDistanceMeasurement();     // Execute TWR
float calculateDistance();            // Compute distance from TWR
void averageAndStoreDistance();        // Average 16 measurements
void scanForDevices();                // Discover available devices
```

## Data Structures

```cpp
struct LinkedDevice {
    uint8_t deviceNumber;           // Device number (1-255)
    uint8_t priority;               // Priority (1-5)
    bool isActive;                  // Is this device slot active?
    bool hasBeenMeasured;           // Have we measured distance?
    float distanceToDeviceX;        // Average distance in meters
    uint32_t lastSeenTime;          // Last time we heard from device
    uint16_t measurementCount;      // Number of measurements taken
    float measurements[16];         // Individual measurements
};
```

## TWR Timing Variables

```cpp
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
```

## File Locations

```
/home/dainlinux/CrainLimiter/
├── Claude.cpp              # Main implementation (1037 lines)
├── prompt.txt              # Development instructions
├── Grok Link.txt           # Grok conversation link
├── Grok History.url        # Empty placeholder
├── PROJECT_SUMMARY.md      # Project overview
├── CONVERSATION_LOG.md     # Detailed conversation
└── QUICK_REFERENCE.md      # This file
```

## Next Steps

1. Set up PlatformIO project
2. Install arduino-dw1000 library
3. Verify pin configuration
4. Flash Claude.cpp to hardware
5. Run initial tests
6. Debug any issues
7. Begin UI design

## Important Notes

- DW1000 requires antenna delay calibration for accuracy
- Use 16-bit short addresses for speed
- LONGDATA_RANGE_LOWPOWER mode recommended
- Sub-10cm accuracy claimed (needs verification)
- 100-200m range depending on mode
- Industrial-grade protection required

---

*Quick Reference - CrainLimiter Project*
