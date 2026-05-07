# CrainLimiter

Multi-device wireless distance measurement system using ESP32 + DW1000 UWB chips.

## Overview

CrainLimiter is an industrial-grade system for measuring distances between multiple devices using Ultra-Wideband (UWB) technology. Each "puck" (ESP32 + DW1000) can measure distances to up to 5 other devices on the same channel with sub-10cm accuracy.

## Features

- **Multi-device ranging:** Up to 5 linked devices per puck
- **Priority-based communication:** Devices communicate in priority order (1-5)
- **Two-Way Ranging (TWR):** Accurate distance measurement via time-of-flight
- **Handshake protocol:** Reliable device linking and unlinking
- **Automatic discovery:** Scan for and connect to available devices
- **Hybrid UI:** Phone-based GUI + minimal hardware controls
- **Industrial grade:** Robust design for indoor/outdoor use

## Hardware

- **Microcontroller:** ESP32
- **UWB Chip:** DW1000 BU-01
- **Range:** 100-200m line-of-sight
- **Accuracy:** Sub-10cm (claimed, needs verification)

## Pin Configuration

```
PIN_RST  = 27  // Reset pin for DW1000
PIN_IRQ  = 34  // Interrupt pin for DW1000
PIN_SS   = 4   // SPI Chip Select pin for DW1000
```

## Configuration

Each puck requires:
- **Channel:** Wireless channel for communication
- **Priority:** Device priority (1-5)
- **Direction1:** Custom variable (TBD)
- **Direction2:** Custom variable (TBD)

## Distance Measurement Protocol

1. **Handshake:** Device A asks "Are you available?" → Device B responds "Yes"
2. **Link Mode:** Both devices enter linked mode, ignore other requests
3. **Measurement:** Lower-numbered device initiates, performs 16 round-trip measurements
4. **Average:** Both devices average their 16 measurements
5. **Store:** Result stored in `distanceToDeviceX` variable
6. **Unlink:** Devices unlink and repeat with other devices
7. **Timeout:** Timer expires, then listen for new devices

## User Interface

### Primary: Phone-Based GUI (WiFi)
- ESP32 creates WiFi AP: "CrainLimiter-Puck-XX"
- Configure: channel, priority, direction1, direction2
- Monitor: real-time distances, linked devices, status

### Secondary: Minimal Hardware
- 1 button: Long press = config mode, short press = cycle displays
- 2 LEDs: Green (connected), Red (error/low battery)

## Getting Started

### Prerequisites

- ESP32 development board
- DW1000 BU-01 UWB module
- PlatformIO
- arduino-dw1000 library

### Installation

1. Install PlatformIO extension for VS Code
2. Clone this repository
3. Install dependencies:
   ```bash
   pio lib install https://github.com/thotro/arduino-dw1000.git
   ```
4. Open in PlatformIO
5. Configure device number and priority in `Claude.cpp`
6. Upload to ESP32

### Testing

1. Flash code to 2+ ESP32 + DW1000 devices
2. Open serial monitor at 115200 baud
3. Verify device initialization
4. Test handshake and ranging between devices
5. Verify multi-device sequential ranging

## Documentation

- `Claude.cpp` - Main implementation (1037 lines)
- `QUICK_REFERENCE.md` - Pin configurations, message protocol, state machine
- `PROJECT_SUMMARY.md` - Project overview and next steps
- `CONVERSATION_LOG.md` - Detailed development history

## Library Requirements

- [arduino-dw1000](https://github.com/thotro/arduino-dw1000) - DW1000 UWB library

## Status

- ✅ Code implementation complete
- ✅ UI design finalized (hybrid approach)
- 🔄 Hardware validation testing (in progress)
- ⏳ WiFi web server development
- ⏳ Phone GUI development

## License

MIT License - See LICENSE file for details

## Contributing

Contributions welcome! Please feel free to submit a Pull Request.

## Author

Mighty Overlord Dain

## Acknowledgments

- Based on [arduino-dw1000](https://github.com/thotro/arduino-dw1000) library
- Inspired by priority-based wireless node polling research
- Grok conversation on wireless multi-device coordination
