/*
 * ESP32 DW1000 BU-01 Multi-Device Distance Measurement System
 * 
 * This code allows an ESP32 to control a DW1000 BU-01 chip for measuring
 * distances to up to 5 other devices on the same channel.
 * 
 * LIBRARY REQUIREMENT:
 * You must install the arduino-dw1000 library from:
 * https://github.com/thotro/arduino-dw1000
 * 
 * Author: Custom Implementation
 * Platform: PlatformIO with ESP32
 * Based on: thotro/arduino-dw1000 library
 */

#include <SPI.h>
#include <DW1000.h>       // From https://github.com/thotro/arduino-dw1000
#include <DW1000Time.h>   // From https://github.com/thotro/arduino-dw1000

// ============================================================================
// PIN CONFIGURATION FOR ESP32
// ============================================================================
#define PIN_RST 27    // Reset pin for DW1000
#define PIN_IRQ 34    // Interrupt pin for DW1000
#define PIN_SS  4     // SPI Chip Select pin for DW1000

// ============================================================================
// DEVICE CONFIGURATION CONSTANTS
// ============================================================================
#define MAX_LINKED_DEVICES 5      // Maximum number of devices we can link to
#define DISTANCE_MEASUREMENTS 16   // Number of round-trip measurements per session
#define LINK_TIMEOUT_MS 5000      // Timeout for linked mode in milliseconds
#define LISTEN_PERIOD_MS 10000    // How long to listen for new devices

// ============================================================================
// DW1000 UWB CONFIGURATION
// ============================================================================
// UWB Channel Selection (1, 2, 3, 4, 5, or 7)
// Channel 5: 6.5 GHz center frequency, good for long range
#define UWB_CHANNEL 5

// Preamble Length (symbols)
// Options: 64, 128, 256, 1024, 2048, 4096
// Longer preamble = better range but slower acquisition
#define PREAMBLE_LENGTH 1024

// Pulse Repetition Frequency (PRF)
// Options: 16 MHz or 64 MHz
// 64 MHz provides better ranging accuracy
#define PRF 64

// Data Rate (kbps)
// Options: 110, 850, 6800
// Lower data rate = longer range
#define DATA_RATE 110

// Transmit Power
// Options: See DW1000 datasheet for power levels
// Higher power = longer range but more power consumption
#define TX_POWER 0x254085A0  // Example: ~-14 dBm, adjust for your hardware

// Antenna Delay (in DW1000 time units, ~15.65 ps per unit)
// CRITICAL FOR ACCURATE DISTANCE MEASUREMENT!
// This value MUST be calibrated for your specific hardware setup
// Typical range: 16384-32768 (depends on antenna, PCB trace length, etc.)
// To calibrate: Measure known distance, adjust this value until reading is accurate
#define ANTENNA_DELAY 16384  // STARTING VALUE - MUST BE CALIBRATED!

// ============================================================================
// MESSAGE TYPES - Protocol for device communication
// ============================================================================
#define MSG_HANDSHAKE_REQUEST  0x01  // "Are you available?"
#define MSG_HANDSHAKE_ACCEPT   0x02  // "Yes, I'm available"
#define MSG_HANDSHAKE_REJECT   0x03  // "No, I'm busy"
#define MSG_PING_REQUEST       0x04  // Distance measurement ping
#define MSG_PING_RESPONSE      0x05  // Distance measurement response
#define MSG_DISTANCE_COMPLETE  0x06  // All measurements complete
#define MSG_UNLINK             0x07  // Ending the link

// ============================================================================
// DEVICE STATE MACHINE STATES
// ============================================================================
enum DeviceState {
    STATE_IDLE,              // Device is idle, listening for requests
    STATE_LINKING,           // Device is attempting to establish link
    STATE_LINKED,            // Device is in an active measurement session
    STATE_MEASURING,         // Device is performing distance measurements
    STATE_WAITING_RESPONSE,  // Device is waiting for response
    STATE_UNLINKING          // Device is ending a link session
};

// ============================================================================
// LINKED DEVICE STRUCTURE
// Each DW1000 can track up to 5 other devices
// ============================================================================
struct LinkedDevice {
    uint8_t deviceNumber;           // Device number (1-255)
    uint8_t priority;               // Priority (1-5)
    bool isActive;                  // Is this device slot active?
    bool hasBeenMeasured;           // Have we measured distance to this device?
    float distanceToDeviceX;        // Average distance in meters
    uint32_t lastSeenTime;          // Last time we heard from this device
    uint16_t measurementCount;      // Number of measurements taken
    float measurements[DISTANCE_MEASUREMENTS];  // Array of individual measurements
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
uint8_t myDeviceNumber = 1;         // This device's unique number (set this per device)
uint8_t myPriority = 1;             // This device's priority (1-5)
DeviceState currentState = STATE_IDLE;
LinkedDevice linkedDevices[MAX_LINKED_DEVICES];
uint8_t linkedDeviceCount = 0;
uint8_t currentLinkedDevice = 0;    // Index of device currently linked to
uint16_t currentMeasurementIndex = 0;
uint32_t linkStartTime = 0;
uint32_t listenStartTime = 0;
bool isInitiator = false;           // Are we initiating the measurement?

// Variables for Two-Way Ranging (TWR)
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;

// Message buffer for sending/receiving
byte messageBuffer[128];
volatile bool messageReceived = false;
volatile bool messageSent = false;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initializeDW1000();
void handleReceivedMessage();
void handleSentMessage();
void sendHandshakeRequest(uint8_t targetDevice);
void sendHandshakeAccept(uint8_t targetDevice);
void sendHandshakeReject(uint8_t targetDevice);
void sendPingRequest();
void sendPingResponse();
void sendDistanceComplete();
void sendUnlink();
void processHandshakeRequest(uint8_t senderDevice);
void processHandshakeAccept(uint8_t senderDevice);
void processHandshakeReject(uint8_t senderDevice);
void processPingRequest();
void processPingResponse();
void processDistanceComplete();
void processUnlink();
void startMeasurementSession(uint8_t deviceIndex);
void performDistanceMeasurement();
float calculateDistance();
void averageAndStoreDistance();
void unlinkCurrentDevice();
void scanForDevices();
int findDeviceByNumber(uint8_t deviceNumber);
int findEmptyDeviceSlot();
void updateDeviceActivity();
void printDistances();

// ============================================================================
// SETUP FUNCTION
// Called once when ESP32 starts
// ============================================================================
void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);
    Serial.println("ESP32 DW1000 Distance Measurement System");
    Serial.print("My Device Number: ");
    Serial.println(myDeviceNumber);
    Serial.print("My Priority: ");
    Serial.println(myPriority);
    
    // Initialize all linked device slots as inactive
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        linkedDevices[i].isActive = false;
        linkedDevices[i].hasBeenMeasured = false;
        linkedDevices[i].measurementCount = 0;
        linkedDevices[i].distanceToDeviceX = 0.0;
    }
    
    // Initialize DW1000 chip
    initializeDW1000();
    
    // Start in idle state, ready to listen
    currentState = STATE_IDLE;
    listenStartTime = millis();
    
    Serial.println("System initialized. Listening for devices...");
}

// ============================================================================
// MAIN LOOP FUNCTION
// Called repeatedly
// ============================================================================
void loop() {
    // Handle received messages
    if (messageReceived) {
        messageReceived = false;
        handleReceivedMessage();
    }
    
    // Handle sent message confirmation
    if (messageSent) {
        messageSent = false;
        handleSentMessage();
    }
    
    // State machine for device behavior
    switch (currentState) {
        case STATE_IDLE:
            // In idle state, we scan for other devices to link with
            // Check if listen period has expired
            if (millis() - listenStartTime > LISTEN_PERIOD_MS) {
                listenStartTime = millis();  // Reset timer
                scanForDevices();  // Look for devices we haven't measured yet
            }
            break;
            
        case STATE_LINKING:
            // Waiting for handshake response
            // Timeout check
            if (millis() - linkStartTime > 1000) {  // 1 second timeout
                Serial.println("Handshake timeout, returning to idle");
                currentState = STATE_IDLE;
            }
            break;
            
        case STATE_LINKED:
            // We're linked, start measurements
            currentState = STATE_MEASURING;
            currentMeasurementIndex = 0;
            
            // Determine who initiates based on device number
            // Lower device number goes first
            if (myDeviceNumber < linkedDevices[currentLinkedDevice].deviceNumber) {
                isInitiator = true;
                delay(10);  // Small delay before starting
                performDistanceMeasurement();
            } else {
                isInitiator = false;
                // Wait for other device to initiate
            }
            break;
            
        case STATE_MEASURING:
            // Check if all measurements are complete
            if (currentMeasurementIndex >= DISTANCE_MEASUREMENTS) {
                // Calculate average and store
                averageAndStoreDistance();
                
                // Send completion message
                sendDistanceComplete();
                
                // Mark this device as measured
                linkedDevices[currentLinkedDevice].hasBeenMeasured = true;
                
                // Unlink and return to idle
                currentState = STATE_UNLINKING;
            }
            
            // Check for timeout
            if (millis() - linkStartTime > LINK_TIMEOUT_MS) {
                Serial.println("Measurement timeout");
                unlinkCurrentDevice();
            }
            break;
            
        case STATE_WAITING_RESPONSE:
            // Waiting for ping response, timeout handled in message processing
            if (millis() - linkStartTime > LINK_TIMEOUT_MS) {
                Serial.println("Response timeout");
                unlinkCurrentDevice();
            }
            break;
            
        case STATE_UNLINKING:
            // Send unlink message
            sendUnlink();
            unlinkCurrentDevice();
            break;
    }
    
    // Update device activity status
    updateDeviceActivity();
    
    // Small delay to prevent CPU hogging
    delay(1);
}

// ============================================================================
// INITIALIZE DW1000 CHIP
// Sets up SPI communication and configures the DW1000 for ranging
// Based on: https://github.com/thotro/arduino-dw1000/blob/master/src/DW1000.h
// ============================================================================
void initializeDW1000() {
    Serial.println("Initializing DW1000...");
    
    // Initialize DW1000 with interrupt and reset pins
    // Function signature from DW1000.h: begin(int irq, int rst = 0xff)
    DW1000.begin(PIN_IRQ, PIN_RST);
    
    // Select the SPI chip select pin for this DW1000
    // Function from DW1000.h: select(int ss)
    DW1000.select(PIN_SS);
    
    // Start a new configuration session
    // Function from DW1000.h: newConfiguration()
    DW1000.newConfiguration();
    
    // Set UWB Channel (1, 2, 3, 4, 5, or 7)
    // Channel 5: 6.5 GHz, good for long range applications
    DW1000.setChannel(UWB_CHANNEL);
    
    // Set Preamble Length
    // Longer preamble = better range but slower acquisition
    // Options: TX_PREAMBLE_LEN_64, TX_PREAMBLE_LEN_128, TX_PREAMBLE_LEN_256,
    //           TX_PREAMBLE_LEN_512, TX_PREAMBLE_LEN_1024, TX_PREAMBLE_LEN_2048, TX_PREAMBLE_LEN_4096
    switch(PREAMBLE_LENGTH) {
        case 64:
            DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_64);
            break;
        case 128:
            DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_128);
            break;
        case 256:
            DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_256);
            break;
        case 512:
            DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_512);
            break;
        case 1024:
            DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_1024);
            break;
        case 2048:
            DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_2048);
            break;
        case 4096:
            DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_4096);
            break;
        default:
            DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_1024);
    }
    
    // Set Pulse Repetition Frequency (PRF)
    // 64 MHz provides better ranging accuracy than 16 MHz
    // Options: DW1000.TX_POW_18DB (16 MHz) or TX_POW_12DB (64 MHz)
    if (PRF == 64) {
        DW1000.setPRF(DW1000.TX_POW_12DB);  // 64 MHz PRF
    } else {
        DW1000.setPRF(DW1000.TX_POW_18DB);  // 16 MHz PRF
    }
    
    // Set Data Rate
    // Lower data rate = longer range but slower communication
    // Options: DW1000.TX_POW_18DB (110 kbps), TX_POW_12DB (850 kbps), TX_POW_6DB (6.8 Mbps)
    switch(DATA_RATE) {
        case 110:
            DW1000.setDataRate(DW1000.TX_POW_18DB);
            break;
        case 850:
            DW1000.setDataRate(DW1000.TX_POW_12DB);
            break;
        case 6800:
            DW1000.setDataRate(DW1000.TX_POW_6DB);
            break;
        default:
            DW1000.setDataRate(DW1000.TX_POW_18DB);
    }
    
    // Set Transmit Power
    // Adjust based on your hardware and regulatory requirements
    // Format: 0xXXXXXXXX (see DW1000 datasheet for power register values)
    DW1000.setTxPower(TX_POWER);
    
    // Set this device's address (using device number)
    // Function from DW1000.h: setDeviceAddress(int16_t val)
    DW1000.setDeviceAddress(myDeviceNumber);
    
    // Set network ID (all devices on same network should use same ID)
    // Function from DW1000.h: setNetworkId(int16_t val)
    DW1000.setNetworkId(10);
    
    // Set Antenna Delay - CRITICAL FOR ACCURATE DISTANCE MEASUREMENT!
    // This compensates for signal delay through antenna and PCB traces
    // MUST be calibrated for your specific hardware setup
    // Function from DW1000.h: setAntennaDelay(uint16_t value)
    DW1000.setAntennaDelay(ANTENNA_DELAY);
    
    // Enable mode for long range, low power operation
    // Modes defined in DW1000.h:
    // MODE_LONGDATA_RANGE_LOWPOWER - Good balance for distance measurement
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    
    // Commit the configuration to the device
    // Function from DW1000.h: commitConfiguration()
    DW1000.commitConfiguration();
    
    // Attach interrupt handlers for sent and received messages
    // These callbacks are called when messages are sent or received
    // Functions from DW1000.h:
    // attachSentHandler(void (*handleSent)(void))
    // attachReceivedHandler(void (*handleReceived)(void))
    DW1000.attachSentHandler([]() {
        messageSent = true;
    });
    
    DW1000.attachReceivedHandler([]() {
        messageReceived = true;
    });
    
    // Start receiver in permanent mode (always listening)
    // Functions from DW1000.h:
    // newReceive(), receivePermanently(bool val), startReceive()
    DW1000.newReceive();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
    
    Serial.println("DW1000 initialized successfully");
    Serial.print("Configuration: Channel ");
    Serial.print(UWB_CHANNEL);
    Serial.print(", Preamble ");
    Serial.print(PREAMBLE_LENGTH);
    Serial.print(", PRF ");
    Serial.print(PRF);
    Serial.print(" MHz, Data Rate ");
    Serial.print(DATA_RATE);
    Serial.println(" kbps");
    Serial.print("Antenna Delay: ");
    Serial.print(ANTENNA_DELAY);
    Serial.println(" (CALIBRATE THIS VALUE!)");
}

// ============================================================================
// HANDLE RECEIVED MESSAGE
// Called when a message is received from another DW1000
// ============================================================================
void handleReceivedMessage() {
    // Get the received message
    // Function from DW1000.h: getData(byte data[], int n)
    int messageLength = DW1000.getDataLength();
    DW1000.getData(messageBuffer, messageLength);
    
    // Message format: [MessageType][SenderDevice][TargetDevice][Data...]
    uint8_t messageType = messageBuffer[0];
    uint8_t senderDevice = messageBuffer[1];
    uint8_t targetDevice = messageBuffer[2];
    
    // Check if message is for us
    if (targetDevice != myDeviceNumber && targetDevice != 0xFF) {
        // Not for us, ignore (0xFF = broadcast)
        return;
    }
    
    Serial.print("Received message type ");
    Serial.print(messageType);
    Serial.print(" from device ");
    Serial.println(senderDevice);
    
    // Process based on message type
    switch (messageType) {
        case MSG_HANDSHAKE_REQUEST:
            processHandshakeRequest(senderDevice);
            break;
        case MSG_HANDSHAKE_ACCEPT:
            processHandshakeAccept(senderDevice);
            break;
        case MSG_HANDSHAKE_REJECT:
            processHandshakeReject(senderDevice);
            break;
        case MSG_PING_REQUEST:
            processPingRequest();
            break;
        case MSG_PING_RESPONSE:
            processPingResponse();
            break;
        case MSG_DISTANCE_COMPLETE:
            processDistanceComplete();
            break;
        case MSG_UNLINK:
            processUnlink();
            break;
        default:
            Serial.println("Unknown message type");
    }
}

// ============================================================================
// HANDLE SENT MESSAGE
// Called when a message has been successfully transmitted
// ============================================================================
void handleSentMessage() {
    // Message was sent successfully
    // Can be used for timing or state management
    Serial.println("Message sent successfully");
}

// ============================================================================
// SEND HANDSHAKE REQUEST
// Initiates a link request to another device
// ============================================================================
void sendHandshakeRequest(uint8_t targetDevice) {
    Serial.print("Sending handshake request to device ");
    Serial.println(targetDevice);
    
    // Build message: [Type][Sender][Target][Priority]
    messageBuffer[0] = MSG_HANDSHAKE_REQUEST;
    messageBuffer[1] = myDeviceNumber;
    messageBuffer[2] = targetDevice;
    messageBuffer[3] = myPriority;
    
    // Transmit the message
    // Functions from DW1000.h: newTransmit(), setData(), startTransmit()
    DW1000.newTransmit();
    DW1000.setData(messageBuffer, 4);
    DW1000.startTransmit();
    
    // Update state
    currentState = STATE_LINKING;
    linkStartTime = millis();
}

// ============================================================================
// SEND HANDSHAKE ACCEPT
// Accepts a link request from another device
// ============================================================================
void sendHandshakeAccept(uint8_t targetDevice) {
    Serial.print("Sending handshake accept to device ");
    Serial.println(targetDevice);
    
    // Build message: [Type][Sender][Target]
    messageBuffer[0] = MSG_HANDSHAKE_ACCEPT;
    messageBuffer[1] = myDeviceNumber;
    messageBuffer[2] = targetDevice;
    
    // Transmit the message
    DW1000.newTransmit();
    DW1000.setData(messageBuffer, 3);
    DW1000.startTransmit();
    
    // Update state - we're now linked
    currentState = STATE_LINKED;
    linkStartTime = millis();
}

// ============================================================================
// SEND HANDSHAKE REJECT
// Rejects a link request (e.g., already linked to another device)
// ============================================================================
void sendHandshakeReject(uint8_t targetDevice) {
    Serial.print("Sending handshake reject to device ");
    Serial.println(targetDevice);
    
    // Build message: [Type][Sender][Target]
    messageBuffer[0] = MSG_HANDSHAKE_REJECT;
    messageBuffer[1] = myDeviceNumber;
    messageBuffer[2] = targetDevice;
    
    // Transmit the message
    DW1000.newTransmit();
    DW1000.setData(messageBuffer, 3);
    DW1000.startTransmit();
}

// ============================================================================
// SEND PING REQUEST
// Sends a ranging ping to the linked device
// Uses Two-Way Ranging (TWR) protocol
// ============================================================================
void sendPingRequest() {
    Serial.println("Sending ping request");
    
    // Build message: [Type][Sender][Target]
    messageBuffer[0] = MSG_PING_REQUEST;
    messageBuffer[1] = myDeviceNumber;
    messageBuffer[2] = linkedDevices[currentLinkedDevice].deviceNumber;
    
    // Send with delayed transmission for precise timing
    // Functions from DW1000.h and DW1000Time.h
    DW1000.newTransmit();
    DW1000.setData(messageBuffer, 3);
    
    // Set a small delay for transmission
    // This helps with timing precision
    DW1000Time deltaTime = DW1000Time(10, DW1000Time::MILLISECONDS);
    timePollSent = DW1000.setDelay(deltaTime);
    
    DW1000.startTransmit();
    
    // Update state
    currentState = STATE_WAITING_RESPONSE;
}

// ============================================================================
// SEND PING RESPONSE
// Responds to a ranging ping from another device
// ============================================================================
void sendPingResponse() {
    Serial.println("Sending ping response");
    
    // Build message: [Type][Sender][Target]
    messageBuffer[0] = MSG_PING_RESPONSE;
    messageBuffer[1] = myDeviceNumber;
    messageBuffer[2] = linkedDevices[currentLinkedDevice].deviceNumber;
    
    // Send response
    DW1000.newTransmit();
    DW1000.setData(messageBuffer, 3);
    
    // Set a reply delay
    DW1000Time deltaTime = DW1000Time(3, DW1000Time::MILLISECONDS);
    timePollAckSent = DW1000.setDelay(deltaTime);
    
    DW1000.startTransmit();
}

// ============================================================================
// SEND DISTANCE COMPLETE
// Notifies the linked device that all measurements are complete
// ============================================================================
void sendDistanceComplete() {
    Serial.println("Sending distance complete");
    
    // Build message: [Type][Sender][Target][AvgDistance(4 bytes)]
    messageBuffer[0] = MSG_DISTANCE_COMPLETE;
    messageBuffer[1] = myDeviceNumber;
    messageBuffer[2] = linkedDevices[currentLinkedDevice].deviceNumber;
    
    // Convert float distance to bytes
    float avgDist = linkedDevices[currentLinkedDevice].distanceToDeviceX;
    memcpy(&messageBuffer[3], &avgDist, sizeof(float));
    
    // Transmit
    DW1000.newTransmit();
    DW1000.setData(messageBuffer, 7);
    DW1000.startTransmit();
}

// ============================================================================
// SEND UNLINK
// Ends the link with the current device
// ============================================================================
void sendUnlink() {
    Serial.println("Sending unlink");
    
    // Build message: [Type][Sender][Target]
    messageBuffer[0] = MSG_UNLINK;
    messageBuffer[1] = myDeviceNumber;
    messageBuffer[2] = linkedDevices[currentLinkedDevice].deviceNumber;
    
    // Transmit
    DW1000.newTransmit();
    DW1000.setData(messageBuffer, 3);
    DW1000.startTransmit();
}

// ============================================================================
// PROCESS HANDSHAKE REQUEST
// Another device wants to link with us
// ============================================================================
void processHandshakeRequest(uint8_t senderDevice) {
    Serial.print("Processing handshake request from device ");
    Serial.println(senderDevice);
    
    // Check if we're already linked to another device
    if (currentState == STATE_LINKED || currentState == STATE_MEASURING) {
        // We're busy, reject the request
        sendHandshakeReject(senderDevice);
        return;
    }
    
    // Check if we've already measured this device
    int deviceIndex = findDeviceByNumber(senderDevice);
    if (deviceIndex >= 0 && linkedDevices[deviceIndex].hasBeenMeasured) {
        // Already measured, reject
        sendHandshakeReject(senderDevice);
        return;
    }
    
    // Find or create a slot for this device
    if (deviceIndex < 0) {
        deviceIndex = findEmptyDeviceSlot();
        if (deviceIndex < 0) {
            // No empty slots, reject
            sendHandshakeReject(senderDevice);
            return;
        }
        
        // Initialize the new device
        linkedDevices[deviceIndex].deviceNumber = senderDevice;
        linkedDevices[deviceIndex].priority = messageBuffer[3];  // Priority from message
        linkedDevices[deviceIndex].isActive = true;
        linkedDevices[deviceIndex].hasBeenMeasured = false;
        linkedDevices[deviceIndex].measurementCount = 0;
        linkedDeviceCount++;
    }
    
    // Accept the link
    currentLinkedDevice = deviceIndex;
    sendHandshakeAccept(senderDevice);
}

// ============================================================================
// PROCESS HANDSHAKE ACCEPT
// Our link request was accepted
// ============================================================================
void processHandshakeAccept(uint8_t senderDevice) {
    Serial.print("Handshake accepted by device ");
    Serial.println(senderDevice);
    
    // Verify this is the device we're trying to link to
    if (linkedDevices[currentLinkedDevice].deviceNumber == senderDevice) {
        // Link established
        currentState = STATE_LINKED;
        linkStartTime = millis();
        Serial.println("Link established");
    }
}

// ============================================================================
// PROCESS HANDSHAKE REJECT
// Our link request was rejected
// ============================================================================
void processHandshakeReject(uint8_t senderDevice) {
    Serial.print("Handshake rejected by device ");
    Serial.println(senderDevice);
    
    // Return to idle state
    currentState = STATE_IDLE;
}

// ============================================================================
// PROCESS PING REQUEST
// Received a ranging ping, need to respond
// ============================================================================
void processPingRequest() {
    Serial.println("Processing ping request");
    
    // Record the time we received the poll
    // Function from DW1000.h: getReceiveTimestamp()
    DW1000.getReceiveTimestamp(timePollReceived);
    
    // Send response after a short delay
    delay(2);  // Small processing delay
    sendPingResponse();
}

// ============================================================================
// PROCESS PING RESPONSE
// Received a response to our ping, calculate distance
// ============================================================================
void processPingResponse() {
    Serial.println("Processing ping response");
    
    // Record the time we received the response
    DW1000.getReceiveTimestamp(timePollAckReceived);
    
    // Calculate the distance based on time-of-flight
    float distance = calculateDistance();
    
    // Store the measurement
    linkedDevices[currentLinkedDevice].measurements[currentMeasurementIndex] = distance;
    currentMeasurementIndex++;
    
    Serial.print("Measurement ");
    Serial.print(currentMeasurementIndex);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" meters");
    
    // If we haven't completed all measurements, continue
    if (currentMeasurementIndex < DISTANCE_MEASUREMENTS) {
        // If we're the initiator, send next ping
        if (isInitiator) {
            // Small delay between measurements
            delay(50);
            performDistanceMeasurement();
        }
        // If we're not the initiator, wait for next ping
    } else {
        // All measurements complete
        currentState = STATE_MEASURING;  // Will trigger averaging in main loop
    }
}

// ============================================================================
// PROCESS DISTANCE COMPLETE
// Other device finished their measurements
// ============================================================================
void processDistanceComplete() {
    Serial.println("Processing distance complete from peer");
    
    // Extract the average distance from the message
    float peerDistance;
    memcpy(&peerDistance, &messageBuffer[3], sizeof(float));
    
    Serial.print("Peer measured distance: ");
    Serial.print(peerDistance);
    Serial.println(" meters");
    
    // We could average our measurement with theirs for better accuracy
    // For now, we'll use our own measurement
}

// ============================================================================
// PROCESS UNLINK
// Other device is ending the link
// ============================================================================
void processUnlink() {
    Serial.println("Processing unlink");
    
    // End the link and return to idle
    unlinkCurrentDevice();
}

// ============================================================================
// START MEASUREMENT SESSION
// Begin a distance measurement session with a specific device
// ============================================================================
void startMeasurementSession(uint8_t deviceIndex) {
    Serial.print("Starting measurement session with device ");
    Serial.println(linkedDevices[deviceIndex].deviceNumber);
    
    currentLinkedDevice = deviceIndex;
    
    // Send handshake request
    sendHandshakeRequest(linkedDevices[deviceIndex].deviceNumber);
}

// ============================================================================
// PERFORM DISTANCE MEASUREMENT
// Initiates a single distance measurement ping
// ============================================================================
void performDistanceMeasurement() {
    Serial.print("Performing measurement ");
    Serial.print(currentMeasurementIndex + 1);
    Serial.print(" of ");
    Serial.println(DISTANCE_MEASUREMENTS);
    
    // Send ping request
    sendPingRequest();
}

// ============================================================================
// CALCULATE DISTANCE
// Calculates distance based on Two-Way Ranging timestamps
// Based on DW1000 Time-of-Flight ranging
// Reference: DW1000 User Manual, Two-Way Ranging section
// ============================================================================
float calculateDistance() {
    // Get all timestamps
    // This implements the Two-Way Ranging (TWR) algorithm
    
    // Time differences (in DW1000 time units)
    DW1000Time round1 = timePollAckReceived - timePollSent;
    DW1000Time reply1 = timePollAckSent - timePollReceived;
    
    // Calculate time-of-flight
    // Formula: ToF = (round1 - reply1) / 2
    DW1000Time tof;
    tof.setTimestamp((round1.getTimestamp() - reply1.getTimestamp()) / 2);
    
    // Convert to distance in meters
    // Function from DW1000Time.h: getAsMeters()
    // Speed of light * time = distance
    float distance = tof.getAsMeters();
    
    return distance;
}

// ============================================================================
// AVERAGE AND STORE DISTANCE
// Calculates average of all measurements and stores in device structure
// ============================================================================
void averageAndStoreDistance() {
    Serial.println("Averaging measurements...");
    
    float sum = 0.0;
    uint16_t validMeasurements = 0;
    
    // Sum all measurements, filtering out obvious errors
    for (int i = 0; i < currentMeasurementIndex; i++) {
        float measurement = linkedDevices[currentLinkedDevice].measurements[i];
        
        // Filter out negative or unreasonably large distances
        if (measurement > 0.0 && measurement < 300.0) {  // Max 300 meters
            sum += measurement;
            validMeasurements++;
        } else {
            Serial.print("Filtered out invalid measurement: ");
            Serial.println(measurement);
        }
    }
    
    // Calculate average
    if (validMeasurements > 0) {
        float average = sum / validMeasurements;
        linkedDevices[currentLinkedDevice].distanceToDeviceX = average;
        linkedDevices[currentLinkedDevice].measurementCount = validMeasurements;
        
        Serial.print("Average distance to device ");
        Serial.print(linkedDevices[currentLinkedDevice].deviceNumber);
        Serial.print(": ");
        Serial.print(average);
        Serial.print(" meters (");
        Serial.print(validMeasurements);
        Serial.println(" valid measurements)");
    } else {
        Serial.println("ERROR: No valid measurements!");
        linkedDevices[currentLinkedDevice].distanceToDeviceX = -1.0;
    }
}

// ============================================================================
// UNLINK CURRENT DEVICE
// Ends the current link session and returns to idle
// ============================================================================
void unlinkCurrentDevice() {
    Serial.println("Unlinking from current device");
    
    // Reset state
    currentState = STATE_IDLE;
    currentMeasurementIndex = 0;
    
    // Print the distance if it was measured
    if (linkedDevices[currentLinkedDevice].distanceToDeviceX > 0) {
        printDistances();
    }
    
    // Return to listening mode
    DW1000.newReceive();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

// ============================================================================
// SCAN FOR DEVICES
// Looks for devices we haven't measured yet and initiates linking
// ============================================================================
void scanForDevices() {
    Serial.println("Scanning for unmeasured devices...");
    
    // Go through our list of known devices
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (linkedDevices[i].isActive && !linkedDevices[i].hasBeenMeasured) {
            // Found a device we need to measure
            startMeasurementSession(i);
            return;
        }
    }
    
    // If we get here, we've measured all known devices
    // In a real implementation, you might broadcast a discovery message here
    Serial.println("All known devices have been measured");
    
    // Print summary
    printDistances();
    
    // Reset the "measured" flags to start over
    // Comment this out if you only want to measure once
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        linkedDevices[i].hasBeenMeasured = false;
    }
}

// ============================================================================
// FIND DEVICE BY NUMBER
// Searches for a device in our linked devices list by device number
// Returns: index if found, -1 if not found
// ============================================================================
int findDeviceByNumber(uint8_t deviceNumber) {
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (linkedDevices[i].isActive && 
            linkedDevices[i].deviceNumber == deviceNumber) {
            return i;
        }
    }
    return -1;  // Not found
}

// ============================================================================
// FIND EMPTY DEVICE SLOT
// Finds an empty slot in the linked devices array
// Returns: index if found, -1 if array is full
// ============================================================================
int findEmptyDeviceSlot() {
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (!linkedDevices[i].isActive) {
            return i;
        }
    }
    return -1;  // No empty slots
}

// ============================================================================
// UPDATE DEVICE ACTIVITY
// Checks for devices that haven't been heard from recently
// Marks them as inactive after timeout
// ============================================================================
void updateDeviceActivity() {
    // This function would track device "last seen" times
    // and mark devices as inactive if they haven't been heard from
    // For now, it's a placeholder for future enhancement
    
    uint32_t currentTime = millis();
    uint32_t activityTimeout = 60000;  // 60 seconds
    
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (linkedDevices[i].isActive) {
            // Check if device has timed out
            if (currentTime - linkedDevices[i].lastSeenTime > activityTimeout) {
                Serial.print("Device ");
                Serial.print(linkedDevices[i].deviceNumber);
                Serial.println(" timed out");
                
                // Could mark as inactive here if desired
                // linkedDevices[i].isActive = false;
            }
        }
    }
}

// ============================================================================
// PRINT DISTANCES
// Prints all measured distances to the serial monitor
// ============================================================================
void printDistances() {
    Serial.println("====================================");
    Serial.println("DISTANCE MEASUREMENTS SUMMARY");
    Serial.println("====================================");
    Serial.print("My Device Number: ");
    Serial.println(myDeviceNumber);
    Serial.println();
    
    bool foundMeasurements = false;
    
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (linkedDevices[i].isActive && linkedDevices[i].hasBeenMeasured) {
            foundMeasurements = true;
            
            Serial.print("Device ");
            Serial.print(linkedDevices[i].deviceNumber);
            Serial.print(" (Priority ");
            Serial.print(linkedDevices[i].priority);
            Serial.print("): ");
            
            if (linkedDevices[i].distanceToDeviceX > 0) {
                Serial.print(linkedDevices[i].distanceToDeviceX, 2);
                Serial.print(" meters");
                Serial.print(" (");
                Serial.print(linkedDevices[i].measurementCount);
                Serial.println(" measurements)");
            } else {
                Serial.println("No valid distance");
            }
        }
    }
    
    if (!foundMeasurements) {
        Serial.println("No measurements available yet");
    }
    
    Serial.println("====================================");
    Serial.println();
}

/*
 * ============================================================================
 * ADDITIONAL NOTES FOR PLATFORMIO SETUP
 * ============================================================================
 * 
 * platformio.ini configuration:
 * 
 * [env:esp32dev]
 * platform = espressif32
 * board = esp32dev
 * framework = arduino
 * lib_deps = 
 *     thotro/arduino-dw1000@^0.9
 * monitor_speed = 115200
 * 
 * ============================================================================
 * USAGE INSTRUCTIONS
 * ============================================================================
 * 
 * 1. Install PlatformIO extension in VS Code
 * 2. Create a new PlatformIO project for ESP32
 * 3. Add the arduino-dw1000 library to lib_deps in platformio.ini
 * 4. Copy this code to src/main.cpp
 * 5. Adjust PIN definitions (PIN_RST, PIN_IRQ, PIN_SS) for your wiring
 * 6. Set unique myDeviceNumber for each ESP32 (1-255)
 * 7. Set myPriority for each device (1-5)
 * 8. Upload to each ESP32
 * 
 * ============================================================================
 * WIRING GUIDE
 * ============================================================================
 * 
 * DW1000 BU-01 -> ESP32
 * VCC    -> 3.3V
 * GND    -> GND
 * MOSI   -> GPIO 23 (default SPI MOSI)
 * MISO   -> GPIO 19 (default SPI MISO)
 * SCK    -> GPIO 18 (default SPI CLK)
 * CS     -> GPIO 4  (or your chosen PIN_SS)
 * IRQ    -> GPIO 34 (or your chosen PIN_IRQ)
 * RST    -> GPIO 27 (or your chosen PIN_RST)
 * 
 * ============================================================================
 * TESTING PROCEDURE
 * ============================================================================
 * 
 * 1. Program Device 1:
 *    - myDeviceNumber = 1
 *    - myPriority = 1
 * 
 * 2. Program Device 2:
 *    - myDeviceNumber = 2
 *    - myPriority = 2
 * 
 * 3. Power both devices
 * 4. Open Serial Monitor for both devices (115200 baud)
 * 5. Device 1 (lower number) will initiate handshake
 * 6. Devices will perform 16 distance measurements
 * 7. Average distance will be calculated and displayed
 * 8. Devices will unlink and scan for other devices
 * 
 * ============================================================================
 * TROUBLESHOOTING
 * ============================================================================
 * 
 * Problem: "DW1000 initialization failed"
 * Solution: Check wiring, especially CS, IRQ, and RST pins
 * 
 * Problem: "No messages received"
 * Solution: Ensure both devices on same network ID (line with setNetworkId)
 *           Ensure both devices use same UWB channel
 * 
 * Problem: "Distance measurements are wildly inaccurate"
 * Solution: CALIBRATE ANTENNA DELAY! (see below)
 *           Ensure good antenna connection, check for interference
 * 
 * Problem: "Handshake timeout"
 * Solution: Check that devices have different device numbers
 *           Ensure devices are within range (usually <100m line of sight)
 * 
 * ============================================================================
 * ANTENNA DELAY CALIBRATION (CRITICAL FOR ACCURACY!)
 * ============================================================================
 * 
 * The ANTENNA_DELAY value compensates for signal delay through your antenna
 * and PCB traces. Without proper calibration, distance measurements will be
 * consistently off by several meters.
 * 
 * CALIBRATION PROCEDURE:
 * 
 * 1. Set up two devices at a KNOWN distance (e.g., exactly 1.0 meter apart)
 * 2. Upload the code with ANTENNA_DELAY = 16384 (starting value)
 * 3. Let devices measure distance and record the reading
 * 4. Calculate error: Error = Measured Distance - Actual Distance
 * 5. Adjust ANTENNA_DELAY:
 *    - If reading is TOO HIGH: Decrease ANTENNA_DELAY
 *    - If reading is TOO LOW: Increase ANTENNA_DELAY
 * 6. Typical adjustment: ~500 units per meter of error
 * 7. Repeat until reading is within 10cm of actual distance
 * 8. Test at multiple distances (1m, 5m, 10m) to verify linearity
 * 
 * EXAMPLE:
 * - Actual distance: 1.0 meter
 * - Measured distance: 1.5 meters (0.5m too high)
 * - Error: +0.5 meters
 * - Adjustment: 0.5 * 500 = 250 units
 * - New ANTENNA_DELAY: 16384 - 250 = 16134
 * 
 * NOTES:
 * - Each hardware setup (antenna, PCB, connectors) needs unique calibration
 * - Calibrate with the same antenna and cable you'll use in production
 * - Temperature can affect calibration slightly
 * - Store calibrated value per device (they may differ slightly)
 * 
 * ============================================================================
 * UWB CHANNEL SELECTION GUIDE
 * ============================================================================
 * 
 * Channel 1: 3.5 GHz - Short range, good for indoor use
 * Channel 2: 3.9 GHz - Short range, alternative to Ch1
 * Channel 3: 4.3 GHz - Medium range
 * Channel 4: 4.7 GHz - Medium range
 * Channel 5: 6.5 GHz - Long range, best for outdoor (RECOMMENDED)
 * Channel 7: 6.5 GHz - Long range, alternative to Ch5
 * 
 * For CrainLimiter: Use Channel 5 for maximum range (100-200m line of sight)
 * 
 * ============================================================================
 * CONFIGURATION TRADE-OFFS
 * ============================================================================
 * 
 * RANGE vs. SPEED:
 * - Longer preamble (4096) = Better range, slower acquisition
 * - Shorter preamble (64) = Faster acquisition, shorter range
 * 
 * ACCURACY vs. POWER:
 * - 64 MHz PRF = Better accuracy, more power consumption
 * - 16 MHz PRF = Lower power, slightly less accurate
 * 
 * RANGE vs. DATA RATE:
 * - 110 kbps = Maximum range, slow communication
 * - 6.8 Mbps = Shorter range, fast communication
 * 
 * RECOMMENDED FOR CRAINLIMITER:
 * - Channel 5, Preamble 1024, PRF 64 MHz, Data Rate 110 kbps
 * - This provides good balance of range, accuracy, and power consumption
 * 
 * ============================================================================
 */