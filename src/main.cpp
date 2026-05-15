/*
 * ESP32 DW1000 BU-01 Multi-Device Distance Measurement System
 * WITH DISCOVERY PROTOCOL
 * 
 * This version implements a proper discovery mechanism where devices
 * periodically broadcast their presence and scan for other devices.
 * 
 * KEY IMPROVEMENT: Devices alternate between broadcasting and listening
 * using time-slotted approach based on device number.
 */

#include <SPI.h>
#include <DW1000.h>
#include <DW1000Time.h>

// ============================================================================
// PIN CONFIGURATION FOR ESP32
// ============================================================================
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS  4

// ============================================================================
// DEVICE CONFIGURATION CONSTANTS
// ============================================================================
#define MAX_LINKED_DEVICES 5
#define DISTANCE_MEASUREMENTS 16
#define LINK_TIMEOUT_MS 5000
#define HANDSHAKE_TIMEOUT_MS 3000
#define LISTEN_PERIOD_MS 10000


// ============================================================================
// DISCOVERY PROTOCOL TIMING
// ============================================================================
// Each device gets a time slot to broadcast based on their device number
// This prevents all devices from transmitting at the same time
#define DISCOVERY_CYCLE_MS 1000      // Total cycle time: 1 second
#define BROADCAST_SLOT_MS 100        // Each device broadcasts for 100ms
#define LISTEN_SLOT_MS 900           // Then listens for 900ms

// Example: Device 1 broadcasts at 0-100ms, Device 2 at 100-200ms, etc.
// Then all devices listen for the remaining time

// ============================================================================
// UWB CONFIGURATION
// ============================================================================
#define UWB_CHANNEL 5
#define PREAMBLE_LENGTH 1024
#define PRF 64
#define DATA_RATE 110
#define ANTENNA_DELAY 16384

// ============================================================================
// MESSAGE TYPES
// ============================================================================
#define MSG_DISCOVERY_BEACON   0x00  // "I'm here!" broadcast
#define MSG_HANDSHAKE_REQUEST  0x01
#define MSG_HANDSHAKE_ACCEPT   0x02
#define MSG_HANDSHAKE_REJECT   0x03
#define MSG_PING_REQUEST       0x04
#define MSG_PING_RESPONSE      0x05
#define MSG_DISTANCE_COMPLETE  0x06
#define MSG_UNLINK             0x07

// ============================================================================
// DEVICE STATE MACHINE STATES
// ============================================================================
enum DeviceState {
    STATE_DISCOVERY,         // Broadcasting and discovering other devices
    STATE_IDLE,              // Idle, listening for requests
    STATE_LINKING,           // Attempting to establish link
    STATE_LINKED,            // In an active measurement session
    STATE_MEASURING,         // Performing distance measurements
    STATE_WAITING_RESPONSE,  // Waiting for response
    STATE_UNLINKING          // Ending a link session
};

// ============================================================================
// LINKED DEVICE STRUCTURE
// ============================================================================
struct LinkedDevice {
    uint8_t deviceNumber;
    uint8_t priority;
    bool isActive;
    bool hasBeenMeasured;
    float distanceToDeviceX;
    uint32_t lastSeenTime;
    uint16_t measurementCount;
    float measurements[DISTANCE_MEASUREMENTS];
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
uint8_t myDeviceNumber = 2;         // CHANGE THIS FOR EACH DEVICE
uint8_t myPriority = 2;             // CHANGE THIS FOR EACH DEVICE
DeviceState currentState = STATE_DISCOVERY;
LinkedDevice linkedDevices[MAX_LINKED_DEVICES];
uint8_t linkedDeviceCount = 0;
uint8_t currentLinkedDevice = 0;
uint16_t currentMeasurementIndex = 0;
uint32_t linkStartTime = 0;
uint32_t discoveryStartTime = 0;
uint32_t lastBeaconTime = 0;
uint32_t lastDiscoveryAttempt = 0;
bool isInitiator = false;
volatile uint8_t lastTxMessageType = 0;

// Two-Way Ranging variables
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;

// Message buffer
byte messageBuffer[128];
byte txBuffer[128];
byte rxBuffer[128];
volatile bool messageReceived = false;
volatile bool messageSent = false;


// Discovery tracking
bool discoveryPhaseComplete = false;
uint32_t discoveryPhaseDuration = 30000;  // Discovery phase: 30 seconds

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initializeDW1000();
void handleDiscoveryPhase();
void sendDiscoveryBeacon();
void processDiscoveryBeacon(uint8_t senderDevice, uint8_t senderPriority);
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
void printDiscoveredDevices();
bool isMyBroadcastSlot();
void enterReceiveMode();
void restartReceiver();
// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n========================================");
    Serial.println("ESP32 DW1000 Distance Measurement");
    Serial.println("WITH DISCOVERY PROTOCOL");
    Serial.println("========================================");
    Serial.print("My Device Number: ");
    Serial.println(myDeviceNumber);
    Serial.print("My Priority: ");
    Serial.println(myPriority);
    Serial.println("========================================\n");
    
    // Initialize linked device slots
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        linkedDevices[i].isActive = false;
        linkedDevices[i].hasBeenMeasured = false;
        linkedDevices[i].measurementCount = 0;
        linkedDevices[i].distanceToDeviceX = 0.0;
    }
    
    // Initialize DW1000
    initializeDW1000();
    
    // Start in discovery mode
    currentState = STATE_DISCOVERY;
    discoveryStartTime = millis();
    lastBeaconTime = 0;
    
    Serial.println("========================================");
    Serial.println("STARTING DISCOVERY PHASE");
    Serial.println("Duration: 30 seconds");
    Serial.println("Broadcasting presence and scanning...");
    Serial.println("========================================\n");
}

// ============================================================================
// MAIN LOOP FUNCTION
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
    
    // State machine
    switch (currentState) {
        case STATE_DISCOVERY:
            handleDiscoveryPhase();
            break;
            
        case STATE_IDLE:
            // Check if it's time to measure devices we've discovered
            if (millis() - lastDiscoveryAttempt > 5000) {
                scanForDevices();
                lastDiscoveryAttempt = millis();
            }
            break;
            
       case STATE_LINKING:

    if (millis() - linkStartTime > HANDSHAKE_TIMEOUT_MS) {
        Serial.println("Handshake timeout, returning to idle");

        restartReceiver();
        currentState = STATE_IDLE;
    }

    break;
            
        case STATE_LINKED:
            // Start measurements
            currentState = STATE_MEASURING;
            currentMeasurementIndex = 0;
            
            // Lower device number initiates
            if (myDeviceNumber < linkedDevices[currentLinkedDevice].deviceNumber) {
                isInitiator = true;
                Serial.println("I am INITIATOR (lower device number)");
                delay(100);
                performDistanceMeasurement();
            } else {
                isInitiator = false;
                Serial.println("I am RESPONDER (higher device number)");
            }
            break;
            
        case STATE_MEASURING:
            // Check if all measurements complete
            if (currentMeasurementIndex >= DISTANCE_MEASUREMENTS) {
                averageAndStoreDistance();
                sendDistanceComplete();
                linkedDevices[currentLinkedDevice].hasBeenMeasured = true;
                currentState = STATE_UNLINKING;
            }
            
            // Timeout check
            if (millis() - linkStartTime > LINK_TIMEOUT_MS) {
                Serial.println("Measurement timeout");
                unlinkCurrentDevice();
            }
            break;
            
        case STATE_WAITING_RESPONSE:
            // Timeout handled in message processing
            if (millis() - linkStartTime > LINK_TIMEOUT_MS) {
                Serial.println("Response timeout");
                unlinkCurrentDevice();
            }
            break;
            
        case STATE_UNLINKING:
            sendUnlink();
            unlinkCurrentDevice();
            break;
    }
    
    // Update device activity
    updateDeviceActivity();
    
    delay(1);
}

// ============================================================================
// HANDLE DISCOVERY PHASE
// Each device broadcasts in its own time slot, then listens
// ============================================================================
void handleDiscoveryPhase() {
    uint32_t currentTime = millis();
    uint32_t elapsedInDiscovery = currentTime - discoveryStartTime;
    
    // Check if discovery phase is complete
    if (elapsedInDiscovery > discoveryPhaseDuration && !discoveryPhaseComplete) {
        discoveryPhaseComplete = true;
        Serial.println("\n========================================");
        Serial.println("DISCOVERY PHASE COMPLETE");
        Serial.println("========================================");
        printDiscoveredDevices();
        Serial.println("========================================");
        Serial.println("STARTING MEASUREMENT PHASE");
        Serial.println("========================================\n");
        
        restartReceiver();
currentState = STATE_IDLE;
        lastDiscoveryAttempt = millis();
        enterReceiveMode();
        return;
    }
    
    // Calculate position in current discovery cycle
    uint32_t cyclePosition = currentTime % DISCOVERY_CYCLE_MS;
    
    // Check if it's our time slot to broadcast
    if (isMyBroadcastSlot()) {
        // Send beacon if we haven't sent one recently
        if (currentTime - lastBeaconTime > (DISCOVERY_CYCLE_MS - 50)) {
            sendDiscoveryBeacon();
            lastBeaconTime = currentTime;
        }
    } else {
        // Not our slot - make sure we're listening
        // The receiver is already in permanent receive mode
        // Just process any incoming messages
    }
}

// ============================================================================
// CHECK IF IT'S OUR BROADCAST TIME SLOT
// ============================================================================
bool isMyBroadcastSlot() {
    uint32_t currentTime = millis();
    uint32_t cyclePosition = currentTime % DISCOVERY_CYCLE_MS;
    
    // Each device gets a 100ms slot based on their device number
    // Device 1: 0-100ms, Device 2: 100-200ms, etc.
    uint32_t mySlotStart = (myDeviceNumber - 1) * BROADCAST_SLOT_MS;
    uint32_t mySlotEnd = mySlotStart + BROADCAST_SLOT_MS;
    
    return (cyclePosition >= mySlotStart && cyclePosition < mySlotEnd);
}

// ============================================================================
// SEND DISCOVERY BEACON
// Broadcasts "I'm here!" to all devices
// ============================================================================
void sendDiscoveryBeacon() {
    // Build beacon message: [Type][DeviceNumber][Priority]
    txBuffer[0] = MSG_DISCOVERY_BEACON;
    txBuffer[1] = myDeviceNumber;
    txBuffer[2] = myPriority;
    
    // Send the beacon
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 3);
    DW1000.startTransmit();
    
    Serial.print(".");  // Simple progress indicator
}

// ============================================================================
// PROCESS DISCOVERY BEACON
// Another device has announced its presence
// ============================================================================
void processDiscoveryBeacon(uint8_t senderDevice, uint8_t senderPriority) {
    // Don't process our own beacons
    if (senderDevice == myDeviceNumber) {
        return;
    }
    
    Serial.print("\n>>> Discovered Device ");
    Serial.print(senderDevice);
    Serial.print(" (Priority ");
    Serial.print(senderPriority);
    Serial.println(")");
    
    // Check if we already know about this device
    int deviceIndex = findDeviceByNumber(senderDevice);
    
    if (deviceIndex >= 0) {
        // Update last seen time
        linkedDevices[deviceIndex].lastSeenTime = millis();
    } else {
        // New device - add to our list
        deviceIndex = findEmptyDeviceSlot();
        if (deviceIndex >= 0) {
            linkedDevices[deviceIndex].deviceNumber = senderDevice;
            linkedDevices[deviceIndex].priority = senderPriority;
            linkedDevices[deviceIndex].isActive = true;
            linkedDevices[deviceIndex].hasBeenMeasured = false;
            linkedDevices[deviceIndex].measurementCount = 0;
            linkedDevices[deviceIndex].lastSeenTime = millis();
            linkedDeviceCount++;
            
            Serial.print("    Added to device list (");
            Serial.print(linkedDeviceCount);
            Serial.println(" devices total)");
        } else {
            Serial.println("    ERROR: Device list full!");
        }
    }
}

// ============================================================================
// INITIALIZE DW1000 CHIP
// ============================================================================
void initializeDW1000() {
    Serial.println("Initializing DW1000...");
    
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    
    // Verify communication
    char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: ");
    Serial.println(msg);
    
    // Configure
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(myDeviceNumber);
    DW1000.setNetworkId(10);
    
    DW1000.setChannel(DW1000.CHANNEL_5);
    DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_1024);
    DW1000.setPulseFrequency(DW1000.TX_PULSE_FREQ_64MHZ);
    DW1000.setDataRate(DW1000.TRX_RATE_110KBPS);
    DW1000.setAntennaDelay(ANTENNA_DELAY);
    
    DW1000.commitConfiguration();
    
    // Attach interrupt handlers
    DW1000.attachSentHandler([]() {
        messageSent = true;
    });
    
    DW1000.attachReceivedHandler([]() {
        messageReceived = true;
    });
    
    // Start in receive mode
    enterReceiveMode();
    
    Serial.println("DW1000 initialized successfully\n");
}

// ============================================================================
// ENTER RECEIVE MODE
// ============================================================================
void enterReceiveMode() {
    DW1000.newReceive();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

// ============================================================================
// HANDLE RECEIVED MESSAGE
// ============================================================================
void handleReceivedMessage() {
    int messageLength = DW1000.getDataLength();
    DW1000.getData(rxBuffer, messageLength);
    
    // Parse message header
    uint8_t messageType = rxBuffer[0];
    uint8_t senderDevice = rxBuffer[1];
    uint8_t targetDevice = (messageLength > 2) ? rxBuffer[2] : 0xFF;
    
    // Process based on message type
    switch (messageType) {
        case MSG_DISCOVERY_BEACON:
            // Beacon only has [Type][DeviceNumber][Priority]
            processDiscoveryBeacon(senderDevice, rxBuffer[2]);
            break;
            
        case MSG_HANDSHAKE_REQUEST:
            // Check if message is for us
            if (targetDevice == myDeviceNumber || targetDevice == 0xFF) {
                processHandshakeRequest(senderDevice);
            }
            break;
            
        case MSG_HANDSHAKE_ACCEPT:
            if (targetDevice == myDeviceNumber) {
                processHandshakeAccept(senderDevice);
            }
            break;
            
        case MSG_HANDSHAKE_REJECT:
            if (targetDevice == myDeviceNumber) {
                processHandshakeReject(senderDevice);
            }
            break;
            
        case MSG_PING_REQUEST:
            if (targetDevice == myDeviceNumber) {
                processPingRequest();
            }
            break;
            
        case MSG_PING_RESPONSE:
            if (targetDevice == myDeviceNumber) {
                processPingResponse();
            }
            break;
            
        case MSG_DISTANCE_COMPLETE:
            if (targetDevice == myDeviceNumber) {
                processDistanceComplete();
            }
            break;
            
        case MSG_UNLINK:
            if (targetDevice == myDeviceNumber) {
                processUnlink();
            }
            break;
    }
    
    if (currentState == STATE_DISCOVERY ||
    currentState == STATE_IDLE)
    {
    restartReceiver();
    }
}

// ============================================================================
// HANDLE SENT MESSAGE
// Called when a message has been successfully transmitted
// FIXED: Properly return DW1000 to RX mode after EVERY transmission
// ============================================================================
void handleSentMessage() {
    Serial.println("Message sent successfully");

    // ALWAYS return to RX mode after TX
    restartReceiver();

    // Handshake accept fully transmitted
    if (lastTxMessageType == MSG_HANDSHAKE_ACCEPT) {
        Serial.println("Handshake accept transmitted successfully");
        currentState = STATE_LINKED;
        linkStartTime = millis();
        Serial.println("Link established!");
    }
}

// ============================================================================
// PRINT DISCOVERED DEVICES
// ============================================================================
void printDiscoveredDevices() {
    Serial.println("\nDiscovered Devices:");
    Serial.println("------------------");
    
    bool foundAny = false;
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (linkedDevices[i].isActive) {
            foundAny = true;
            Serial.print("  Device ");
            Serial.print(linkedDevices[i].deviceNumber);
            Serial.print(" (Priority ");
            Serial.print(linkedDevices[i].priority);
            Serial.println(")");
        }
    }
    
    if (!foundAny) {
        Serial.println("  No other devices found");
        Serial.println("  Make sure other devices are powered on!");
    }
    Serial.println();
}

// ============================================================================
// SCAN FOR DEVICES TO MEASURE
// CRITICAL: Only initiate if we have LOWER device number to avoid collisions
// ============================================================================
void scanForDevices() {
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (linkedDevices[i].isActive && !linkedDevices[i].hasBeenMeasured) {
            
            // CRITICAL RULE: Only initiate handshake if we have LOWER device number
            // This prevents both devices from trying to initiate simultaneously
            if (myDeviceNumber < linkedDevices[i].deviceNumber) {
                Serial.print("\n>>> Initiating measurement with Device ");
                Serial.print(linkedDevices[i].deviceNumber);
                Serial.println(" (I have lower device number)");
                startMeasurementSession(i);
                return;
            } else {
                Serial.print("\n>>> Waiting for Device ");
                Serial.print(linkedDevices[i].deviceNumber);
                Serial.println(" to initiate (they have lower device number)");
                // Don't initiate - wait for other device to send handshake
            }
        }
    }
    
    // All devices measured
    if (linkedDeviceCount > 0) {
        bool allMeasured = true;
        for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
            if (linkedDevices[i].isActive && !linkedDevices[i].hasBeenMeasured) {
                allMeasured = false;
                break;
            }
        }
        
        if (allMeasured) {
            Serial.println("\n>>> All devices measured. Resetting for new cycle...\n");
            printDistances();
            
            // Reset measured flags
            for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
                linkedDevices[i].hasBeenMeasured = false;
            }
        }
    }
}

// ============================================================================
// REMAINING FUNCTIONS (Same as before, but shortened for space)
// ============================================================================

void sendHandshakeRequest(uint8_t targetDevice) {
    Serial.print("Sending handshake request to device ");
    Serial.println(targetDevice);
    lastTxMessageType = MSG_HANDSHAKE_REQUEST;
    txBuffer[0] = MSG_HANDSHAKE_REQUEST;
    txBuffer[1] = myDeviceNumber;
    txBuffer[2] = targetDevice;
    txBuffer[3] = myPriority;
    
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 4);
    DW1000.startTransmit();
    
    currentState = STATE_LINKING;
    linkStartTime = millis();
}

// ============================================================================
// SEND HANDSHAKE ACCEPT
// Accepts a link request from another device
// FIXED: Don't enter LINKED state until TX completes
// ============================================================================
void sendHandshakeAccept(uint8_t targetDevice) {
    Serial.print("Sending handshake accept to device ");
    Serial.println(targetDevice);

    // Build message: [Type][Sender][Target]
    lastTxMessageType = MSG_HANDSHAKE_ACCEPT;
    txBuffer[1] = myDeviceNumber;
    txBuffer[2] = targetDevice;

    // Transmit the message
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 3);
    DW1000.startTransmit();
    delay(2);

    // DO NOT enter LINKED state yet.
    // Wait until TX complete interrupt fires.
    linkStartTime = millis();
}

void sendHandshakeReject(uint8_t targetDevice) {
    Serial.print("Sending handshake reject to device ");
    Serial.println(targetDevice);
    lastTxMessageType = MSG_HANDSHAKE_REJECT;
    txBuffer[1] = myDeviceNumber;
    txBuffer[2] = targetDevice;
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 3);
    DW1000.startTransmit();
    delay(2);
}

void sendPingRequest() {
    lastTxMessageType = MSG_PING_REQUEST;
    txBuffer[0] = MSG_PING_REQUEST;
    txBuffer[1] = myDeviceNumber;
    txBuffer[2] = linkedDevices[currentLinkedDevice].deviceNumber;
    
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 3);
    
    DW1000Time deltaTime = DW1000Time(10, DW1000Time::MILLISECONDS);
    timePollSent = DW1000.setDelay(deltaTime);
    
    DW1000.startTransmit();
    
    currentState = STATE_WAITING_RESPONSE;
    linkStartTime = millis();  // reset timeout
   
}

void sendPingResponse() {
    txBuffer[0] = MSG_PING_RESPONSE;
    txBuffer[1] = myDeviceNumber;
    txBuffer[2] = linkedDevices[currentLinkedDevice].deviceNumber;
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 3);
    DW1000Time deltaTime = DW1000Time(3, DW1000Time::MILLISECONDS);
    timePollAckSent = DW1000.setDelay(deltaTime);
    DW1000.startTransmit();
    delay(2);
}

void sendDistanceComplete() {
    txBuffer[0] = MSG_DISTANCE_COMPLETE;
    txBuffer[1] = myDeviceNumber;
    txBuffer[2] = linkedDevices[currentLinkedDevice].deviceNumber;
    float avgDist = linkedDevices[currentLinkedDevice].distanceToDeviceX;
    memcpy(&txBuffer[3], &avgDist, sizeof(float));
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 7);
    DW1000.startTransmit();
    delay(2);
}

void sendUnlink() {
    txBuffer[0] = MSG_UNLINK;
    txBuffer[1] = myDeviceNumber;
    txBuffer[2] = linkedDevices[currentLinkedDevice].deviceNumber;
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 3);
    DW1000.startTransmit();
    delay(2);
}

void processHandshakeRequest(uint8_t senderDevice) {
    Serial.print("<<< Received handshake request from device ");
    Serial.println(senderDevice);
    
    if (currentState == STATE_LINKED || currentState == STATE_MEASURING) {
        Serial.println("    Already busy - rejecting");
        sendHandshakeReject(senderDevice);
        return;
    }
    
    int deviceIndex = findDeviceByNumber(senderDevice);
    if (deviceIndex >= 0 && linkedDevices[deviceIndex].hasBeenMeasured) {
        Serial.println("    Already measured this device - rejecting");
        sendHandshakeReject(senderDevice);
        return;
    }
    
    if (deviceIndex < 0) {
        deviceIndex = findEmptyDeviceSlot();
        if (deviceIndex < 0) {
            Serial.println("    Device list full - rejecting");
            sendHandshakeReject(senderDevice);
            return;
        }
        linkedDevices[deviceIndex].deviceNumber = senderDevice;
        linkedDevices[deviceIndex].priority = rxBuffer[3];
        linkedDevices[deviceIndex].isActive = true;
        linkedDevices[deviceIndex].hasBeenMeasured = false;
        linkedDevices[deviceIndex].measurementCount = 0;
        linkedDeviceCount++;
    }
    
    currentLinkedDevice = deviceIndex;
    Serial.println("    Accepting handshake");
    sendHandshakeAccept(senderDevice);
}

void processHandshakeAccept(uint8_t senderDevice) {
    Serial.print("Handshake accepted by device ");
    Serial.println(senderDevice);
    if (linkedDevices[currentLinkedDevice].deviceNumber == senderDevice) {
        currentState = STATE_LINKED;
        linkStartTime = millis();
        Serial.println("Link established");
    }
}

void processHandshakeReject(uint8_t senderDevice) {
    Serial.print("Handshake rejected by device ");
    Serial.println(senderDevice);
    restartReceiver();
currentState = STATE_IDLE;
}

void processPingRequest() {
    Serial.println("<<< Received PING REQUEST");
    
    // Capture timestamp as soon as possible
    DW1000.getReceiveTimestamp(timePollReceived);
    
    // Send response
    delayMicroseconds(250);  // Small fixed delay
    sendPingResponse();
    
    // Do NOT increment measurement index here — let the initiator drive it
}

void processPingResponse() {
    DW1000.getReceiveTimestamp(timePollAckReceived);
    
    float distance = calculateDistance();
    
    // Store using current index BEFORE incrementing
    if (currentMeasurementIndex < DISTANCE_MEASUREMENTS) {
        linkedDevices[currentLinkedDevice].measurements[currentMeasurementIndex] = distance;
        
        Serial.print("Measurement ");
        Serial.print(currentMeasurementIndex + 1);
        Serial.print(": ");
        Serial.print(distance, 3);
        Serial.println(" m");
        
        currentMeasurementIndex++;
    }
    
    if (currentMeasurementIndex < DISTANCE_MEASUREMENTS) {
        delay(50);
        performDistanceMeasurement();  // send next ping
    } else {
        currentState = STATE_MEASURING;  // will trigger averaging
    }
}

void processDistanceComplete() {
    float peerDistance;
    memcpy(&peerDistance, &rxBuffer[3], sizeof(float));
    Serial.print("Peer measured distance: ");
    Serial.print(peerDistance);
    Serial.println(" meters");
}

void processUnlink() {
    Serial.println("Processing unlink");
    unlinkCurrentDevice();
}

void startMeasurementSession(uint8_t deviceIndex) {
    Serial.print("Starting measurement session with device ");
    Serial.println(linkedDevices[deviceIndex].deviceNumber);
    currentLinkedDevice = deviceIndex;
    currentState = STATE_LINKING;
    sendHandshakeRequest(linkedDevices[deviceIndex].deviceNumber);
}

void performDistanceMeasurement() {
    Serial.print("Performing measurement ");
    Serial.print(currentMeasurementIndex + 1);
    Serial.print(" of ");
    Serial.println(DISTANCE_MEASUREMENTS);
    sendPingRequest();
}

float calculateDistance() {
    DW1000Time round1 = timePollAckReceived - timePollSent;
    DW1000Time reply1 = timePollAckSent - timePollReceived;
    DW1000Time tof;
    tof.setTimestamp((round1.getTimestamp() - reply1.getTimestamp()) / 2);
    return tof.getAsMeters();
}

void averageAndStoreDistance() {
    Serial.println("Averaging measurements...");
    float sum = 0.0;
    uint16_t validMeasurements = 0;
    
    for (int i = 0; i < currentMeasurementIndex; i++) {
        float measurement = linkedDevices[currentLinkedDevice].measurements[i];
        if (measurement > 0.0 && measurement < 300.0) {
            sum += measurement;
            validMeasurements++;
        }
    }
    
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
    }
}

void unlinkCurrentDevice() {
    Serial.println("Unlinking from current device");
    restartReceiver();
    currentState = STATE_IDLE;
    currentMeasurementIndex = 0;
    
    if (linkedDevices[currentLinkedDevice].distanceToDeviceX > 0) {
        printDistances();
    }
    
    
}

int findDeviceByNumber(uint8_t deviceNumber) {
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (linkedDevices[i].isActive && linkedDevices[i].deviceNumber == deviceNumber) {
            return i;
        }
    }
    return -1;
}

int findEmptyDeviceSlot() {
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (!linkedDevices[i].isActive) {
            return i;
        }
    }
    return -1;
}

void updateDeviceActivity() {
    // Placeholder for timeout tracking
}
// ============================================================================
// FORCE RADIO BACK TO RECEIVE MODE
// ============================================================================
void restartReceiver() {
    DW1000.newReceive();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void printDistances() {
    Serial.println("\n====================================");
    Serial.println("DISTANCE MEASUREMENTS SUMMARY");
    Serial.println("====================================");
    Serial.print("My Device Number: ");
    Serial.println(myDeviceNumber);
    Serial.println();
    
    for (int i = 0; i < MAX_LINKED_DEVICES; i++) {
        if (linkedDevices[i].isActive && linkedDevices[i].hasBeenMeasured) {
            Serial.print("Device ");
            Serial.print(linkedDevices[i].deviceNumber);
            Serial.print(": ");
            if (linkedDevices[i].distanceToDeviceX > 0) {
                Serial.print(linkedDevices[i].distanceToDeviceX, 2);
                Serial.println(" meters");
            } else {
                Serial.println("No valid distance");
            }
        }
    }
    Serial.println("====================================\n");
}