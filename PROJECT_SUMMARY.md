# CrainLimiter Project Summary

**Date:** May 4, 2026
**Project Name:** CrainLimiter
**User:** Mighty Overlord Dain

## Project Overview

CrainLimiter is a multi-device wireless distance measurement system using ESP32 microcontrollers paired with DW1000 BU-01 UWB (Ultra-Wideband) chips. Each unit (called a "puck") can measure distances to up to 5 other devices on the same channel using priority-based communication.

## Hardware Components

- **Microcontroller:** ESP32
- **UWB Chip:** DW1000 BU-01
- **UI Components:** 
  - Buttons (for local control)
  - OLED display (for local status)
  - WiFi/Bluetooth (for phone-based GUI)
- **Protection:** Industrial-grade enclosure for indoor/outdoor use

## Current Files

### 1. Claude.cpp (1037 lines)
Complete ESP32 DW1000 Multi-Device Distance Measurement System implementation.

**Key Features:**
- Pin configuration: RST=27, IRQ=34, SS=4
- Support for up to 5 linked devices
- 16 distance measurements per session
- Message protocol with handshake, ping, distance complete, and unlink messages
- State machine: idle, linking, linked, measuring, waiting response, unlinking
- Two-Way Ranging (TWR) implementation
- Device discovery, distance measurement, and averaging functions

**Library Required:**
- arduino-dw1000 from https://github.com/thotro/arduino-dw1000

### 2. Grok Link.txt
Link to Grok conversation about "Priority-Based Wireless Node Polling"

### 3. prompt.txt
Instructions for C++ developer to write ESP32 + DW1000 functions:
- Function 1: Linked devices (up to 5 on same channel)
- Function 2: Agreed distance measurement with handshake protocol

### 4. Grok History.url
Empty file (placeholder)

## Grok Conversation Summary

The Grok discussion covered priority-based wireless node polling systems:

**Research Precedents:**
- Wireless Sensor Networks (WSNs)
- MAC protocols for priority-based scheduling
- Wireless Token Ring Protocol (WTRP)

**Practical Implementations:**
- Master-slave polling with nRF24L01+, LoRa, ESP-NOW
- Mesh networks with priority logic (painlessMesh, ESP-WIFI-MESH)

**Key Insight:** ESP32 + DW1000 is a mature, well-supported setup for multi-node distance measurement networks. The DW1000 provides sub-10cm accuracy via Time-of-Flight and supports bidirectional data exchange.

## User Interface Design

### Decision: Hybrid Approach

**Primary UI: Phone-Based GUI (WiFi)**
- ESP32 creates WiFi AP: "CrainLimiter-Puck-01"
- Phone connects, browser opens to 192.168.4.1
- Configure: channel, priority, direction1, direction2
- Monitor: real-time distances, linked devices, status
- Rich touch interface, data visualization, logging

**Secondary UI: Minimal Hardware**
- 1 button: Long press = config mode, short press = cycle displays
- 2 LEDs: Green (connected), Red (error/low battery)
- Basic status indication without phone

**Benefits:**
- Rich configuration and monitoring via phone
- Basic status indication without phone
- Lower hardware cost than full OLED
- Field-ready operation
- Industrial-grade robustness

## Next Steps

### Phase 1: Hardware Validation (Current Priority)
**Goal:** Verify core distance measurement functionality

**Tasks:**
1. Flash Claude.cpp to ESP32 + DW1000
2. Test basic distance measurement (2 devices)
3. Verify multi-device linking (3-5 devices)
4. Check accuracy and reliability
5. Identify any code issues

**Hardware Needed:**
- 2x ESP32 + DW1000 BU-01 modules
- USB cables for programming
- Serial monitor for debugging

**Software Setup:**
- PlatformIO project setup
- arduino-dw1000 library installed
- Serial terminal (115200 baud)

**Test Plan:**
1. Single device initialization
2. Two-device handshake and ranging
3. Multi-device (3-5) sequential ranging
4. Accuracy verification (known distances)
5. Timeout and error handling

### Phase 2: Parallel Development (During Phase 1)
- Design hardware UI (button + OLED layout)
- Sketch web GUI interface
- Define data API (what web server will expose)

### Phase 3: UI Implementation
- Build hardware UI (OLED + buttons)
- Create WiFi web server
- Develop phone GUI

## Key Technical Details

### Distance Measurement Protocol
1. Handshake: Device A asks "Are you available?" → Device B responds "Yes"
2. Link mode: Both devices enter linked mode, ignore other requests
3. Measurement: Lower-numbered device initiates, performs 16 round-trip measurements
4. Average: Both devices average their 16 measurements
5. Store: Result stored in `distanceToDeviceX` variable
6. Unlink: Devices unlink and repeat with other devices
7. Timeout: Timer expires, then listen for new devices

### Configuration Variables
- **Channel:** Wireless channel for communication
- **Priority:** Device priority (1-5)
- **Direction1:** Custom variable (TBD)
- **Direction2:** Custom variable (TBD)

## Notes for Next Session

- User has hardware on hand for testing
- Priority is hardware validation before GUI development
- Industrial-grade design with protection for buttons/screen
- Indoor/outdoor use case
- Need to verify DW1000 accuracy claims (sub-10cm)
- Multi-device collision handling needs testing

## Status

**Current Phase:** Hardware Validation
**Next Action:** Test Claude.cpp on real ESP32 + DW1000 hardware
**Blocking:** None - ready to proceed with hardware testing

---

*End of Summary - May 4, 2026*
