/*
 * ============================================================================
 * DW1000 STATELESS NEIGHBOR-BASED RANGING PROTOCOL
 * ============================================================================
 * 
 * ARCHITECTURE:
 * - Fully distributed (no central controller)
 * - Stateless ranging (no persistent links/sessions)
 * - Neighbor discovery via periodic ANNOUNCE broadcasts
 * - Deterministic initiator rule (lower ID always initiates)
 * - Double-Sided Two-Way Ranging (DS-TWR) for accuracy
 * - Random backoff for collision avoidance
 * - Handles power cycling gracefully
 * 
 * MESSAGE FLOW:
 * 1. All nodes periodically broadcast ANNOUNCE
 * 2. Nodes build neighbor table from received ANNOUNCEs
 * 3. Lower ID node initiates ranging: POLL → RESP → FINAL
 * 4. Distance calculated from DS-TWR timestamps
 * 
 * PROTOCOL MESSAGES:
 * - ANNOUNCE: Periodic broadcast "I'm here!"
 * - POLL: Start ranging exchange
 * - RESP: Response to POLL
 * - FINAL: Complete ranging exchange
 * 
 * Author: Custom Implementation
 * Platform: PlatformIO with ESP32 + DW1000
 * Library: thotro/arduino-dw1000
 * ============================================================================
 */

#include <SPI.h>
#include <DW1000.h>
#include <DW1000Time.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================
#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS  4

// ============================================================================
// DEVICE CONFIGURATION - SET UNIQUE ID FOR EACH DEVICE
// ============================================================================
uint8_t myDeviceID = 2;  // CHANGE THIS: 1, 2, 3, 4, etc.

// ============================================================================
// PROTOCOL TIMING PARAMETERS
// ============================================================================
#define ANNOUNCE_INTERVAL_MS 1000    // Broadcast presence every 1 second
#define RANGE_INTERVAL_MS 500        // Attempt ranging every 500ms (was 200ms - too fast!)
#define NEIGHBOR_TIMEOUT_MS 3000     // Consider neighbor lost after 3 seconds
#define BACKOFF_MAX_MS 20            // Random backoff 0-20ms to avoid collisions
#define RANGING_TIMEOUT_MS 150       // Give up on ranging after 150ms

// ============================================================================
// NEIGHBOR TABLE CONFIGURATION
// ============================================================================
#define MAX_NEIGHBORS 4              // Track up to 4 neighbors

// ============================================================================
// MESSAGE TYPES
// ============================================================================
#define MSG_ANNOUNCE 0x01  // "I'm here!" discovery broadcast
#define MSG_POLL     0x02  // Start of DS-TWR exchange
#define MSG_RESP     0x03  // Response in DS-TWR exchange
#define MSG_FINAL    0x04  // Final message in DS-TWR exchange

// ============================================================================
// DEVICE STATE (Simplified - only 2 states!)
// ============================================================================
enum DeviceState {
    STATE_IDLE,      // Normal operation: announcing, listening, ranging
    STATE_RANGING    // Actively performing a DS-TWR exchange
};

// ============================================================================
// NEIGHBOR STRUCTURE
// ============================================================================
struct Neighbor {
    uint8_t id;              // Neighbor's device ID
    uint32_t lastSeen;       // Last time we heard from them (millis)
    float lastDistance;      // Most recent distance measurement (meters)
    bool valid;              // Is this slot occupied?
    uint32_t rangeCount;     // Number of successful ranges
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
DeviceState currentState = STATE_IDLE;
Neighbor neighbors[MAX_NEIGHBORS];

// Timing variables
uint32_t lastAnnounceTime = 0;
uint32_t lastRangeTime = 0;
uint32_t rangingStartTime = 0;

// Current ranging target
uint8_t rangingTarget = 0;

// DS-TWR Timestamps
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;

// Message buffers - CRITICAL: Separate TX and RX buffers!
byte txBuffer[128];  // Transmit buffer
byte rxBuffer[128];  // Receive buffer
volatile bool messageReceived = false;
volatile bool messageSent = false;
volatile bool txInProgress = false;  // Track TX state

// DIAGNOSTIC COUNTERS
uint32_t pollsSent = 0;
uint32_t respsReceived = 0;
uint32_t finalsSent = 0;
uint32_t pollsReceived = 0;
uint32_t respsSent = 0;
uint32_t finalsReceived = 0;
uint32_t totalMessagesReceived = 0;

// Statistics
uint32_t totalRanges = 0;
uint32_t failedRanges = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initializeDW1000();
void handleReceivedMessage();
void handleSentMessage();
void sendAnnounce();
void sendPoll(uint8_t target);
void sendResp(uint8_t target);
void sendFinal(uint8_t target);
void processAnnounce(uint8_t senderID);
void processPoll(uint8_t senderID);
void processResp(uint8_t senderID);
void processFinal(uint8_t senderID);
void attemptRanging();
void computeDistance(uint8_t neighborID);
void updateNeighbor(uint8_t id);
int findNeighbor(uint8_t id);
int findFreeNeighborSlot();
void cleanupNeighbors();
void printNeighbors();
void printStatistics();
void enterReceiveMode();

// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n========================================");
    Serial.println("DW1000 STATELESS RANGING PROTOCOL");
    Serial.println("========================================");
    Serial.print("Device ID: ");
    Serial.println(myDeviceID);
    Serial.println("========================================");
    Serial.println("Protocol: DS-TWR");
    Serial.println("Mode: Distributed, Stateless");
    Serial.println("========================================\n");
    
    // Initialize neighbor table
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        neighbors[i].valid = false;
        neighbors[i].lastDistance = 0.0;
        neighbors[i].rangeCount = 0;
    }
    
    // Initialize DW1000
    initializeDW1000();
    
    // Start in idle state
    currentState = STATE_IDLE;
    
    // Seed random number generator for backoff
    randomSeed(analogRead(0) + myDeviceID);
    
    Serial.println("System ready. Starting operation...\n");
}

// ============================================================================
// MAIN LOOP - CORE PROTOCOL LOGIC
// ============================================================================
void loop() {
    uint32_t now = millis();
    
    // ========================================
    // HANDLE RECEIVED MESSAGES
    // ========================================
    if (messageReceived) {
        messageReceived = false;
        handleReceivedMessage();
    }
    
    // ========================================
    // HANDLE SENT MESSAGE CONFIRMATION
    // ========================================
    if (messageSent) {
        messageSent = false;
        handleSentMessage();
    }
    
    // ========================================
    // PERIODIC ANNOUNCE (DISCOVERY)
    // ========================================
    if (now - lastAnnounceTime > ANNOUNCE_INTERVAL_MS) {
        sendAnnounce();
        lastAnnounceTime = now;
    }
    
    // ========================================
    // PERIODIC RANGING ATTEMPTS
    // ========================================
    if (currentState == STATE_IDLE && 
        (now - lastRangeTime > RANGE_INTERVAL_MS)) {
        attemptRanging();
        lastRangeTime = now;
    }
    
    // ========================================
    // RANGING TIMEOUT
    // ========================================
    if (currentState == STATE_RANGING && 
        (now - rangingStartTime > RANGING_TIMEOUT_MS)) {
        Serial.println("⚠ Ranging timeout");
        failedRanges++;
        currentState = STATE_IDLE;
        enterReceiveMode();
    }
    
    // ========================================
    // PERIODIC NEIGHBOR CLEANUP
    // ========================================
    static uint32_t lastCleanup = 0;
    if (now - lastCleanup > 1000) {
        cleanupNeighbors();
        lastCleanup = now;
    }
    
    // ========================================
    // PERIODIC STATISTICS
    // ========================================
    static uint32_t lastStats = 0;
    if (now - lastStats > 10000) {
        printStatistics();
        lastStats = now;
    }
    
    delay(1);
}

// ============================================================================
// INITIALIZE DW1000
// ============================================================================
void initializeDW1000() {
    Serial.println("Initializing DW1000...");
    
    // Begin DW1000
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    
    // Verify communication
    char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    Serial.print("  Device ID: ");
    Serial.println(msg);
    
    // Configure DW1000
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(myDeviceID);
    DW1000.setNetworkId(10);
    
    // Optimize for ranging accuracy
    DW1000.setChannel(DW1000.CHANNEL_5);
    DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_1024);
    DW1000.setPulseFrequency(DW1000.TX_PULSE_FREQ_64MHZ);
    DW1000.setDataRate(DW1000.TRX_RATE_110KBPS);
    DW1000.setAntennaDelay(16384);  // CALIBRATE THIS!
    
    DW1000.commitConfiguration();
    
    // Attach interrupt handlers
    DW1000.attachSentHandler([]() {
        messageSent = true;
        txInProgress = false;
    });
    
    DW1000.attachReceivedHandler([]() {
        messageReceived = true;
    });
    
    // Start receiver
    enterReceiveMode();
    
    Serial.println("  DW1000 initialized\n");
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
// SEND ANNOUNCE (DISCOVERY BROADCAST)
// ============================================================================
void sendAnnounce() {
    // Message format: [Type][SenderID]
    txBuffer[0] = MSG_ANNOUNCE;
    txBuffer[1] = myDeviceID;
    
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 2);
    DW1000.startTransmit();
    
    // Wait for TX to complete
    delay(5);
    
    // Return to RX mode
    enterReceiveMode();
    
    // Visual indicator (don't flood serial)
    static int announceCount = 0;
    if (++announceCount % 10 == 0) {
        Serial.print(".");
        if (announceCount % 100 == 0) {
            Serial.println();
        }
    }
}

// ============================================================================
// SEND POLL (START DS-TWR)
// ============================================================================
void sendPoll(uint8_t target) {
    pollsSent++;
    Serial.println("\n========================================");
    Serial.print("→ POLL #");
    Serial.print(pollsSent);
    Serial.print(" to device ");
    Serial.println(target);
    
    // Message format: [Type][SenderID][TargetID]
    txBuffer[0] = MSG_POLL;
    txBuffer[1] = myDeviceID;
    txBuffer[2] = target;
    
    // Send the message
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 3);
    DW1000.startTransmit();
    
    // Simple blocking wait for TX complete (just like the test code)
    delay(10);
    
    // Capture transmit timestamp
    DW1000.getTransmitTimestamp(timePollSent);
    
    Serial.println("  ✓ TX complete, entering RX mode...");
    
    // EXACTLY like the test code that worked
    DW1000.newReceive();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
    
    Serial.println("  ✓ RX mode active");
    Serial.print("  Waiting for RESP (timeout in ");
    Serial.print(RANGING_TIMEOUT_MS);
    Serial.println("ms)...");
    
    // Enter ranging state
    currentState = STATE_RANGING;
    rangingTarget = target;
    rangingStartTime = millis();
}

// ============================================================================
// SEND RESP (RESPOND TO POLL)
// ============================================================================
void sendResp(uint8_t target) {
    respsSent++;
    Serial.print("→ RESP #");
    Serial.print(respsSent);
    Serial.print(" to device ");
    Serial.print(target);
    Serial.print(" (waiting 20ms)...");
    
    // Message format: [Type][SenderID][TargetID]
    txBuffer[0] = MSG_RESP;
    txBuffer[1] = myDeviceID;
    txBuffer[2] = target;
    
    // REDUCED delay - just enough for Device 1 to enter RX
    delay(20);  // Was 50ms - TOO LONG!
    
    Serial.println(" sending now");
    
    // Send (just like test code)
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 3);
    DW1000.startTransmit();
    
    // Wait for TX
    delay(10);
    
    // Capture transmit timestamp
    DW1000.getTransmitTimestamp(timePollAckSent);
    
    Serial.println("  ✓ RESP sent, back to RX mode");
    
    // Back to RX (just like test code)
    DW1000.newReceive();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

// ============================================================================
// SEND FINAL (COMPLETE DS-TWR)
// ============================================================================
void sendFinal(uint8_t target) {
    Serial.print("→ FINAL to device ");
    Serial.println(target);
    
    // Message format: [Type][SenderID][TargetID]
    txBuffer[0] = MSG_FINAL;
    txBuffer[1] = myDeviceID;
    txBuffer[2] = target;
    
    // Give responder time to enter RX
    delay(50);
    
    // Send (just like test code)
    DW1000.newTransmit();
    DW1000.setData(txBuffer, 3);
    DW1000.startTransmit();
    
    // Wait for TX
    delay(10);
    
    // Capture transmit timestamp
    DW1000.getTransmitTimestamp(timeRangeSent);
    
    Serial.println("  FINAL sent, back to RX mode");
    
    // Back to RX (just like test code)
    DW1000.newReceive();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

// ============================================================================
// HANDLE RECEIVED MESSAGE
// ============================================================================
void handleReceivedMessage() {
    totalMessagesReceived++;
    
    // Read message into RX buffer
    int len = DW1000.getDataLength();
    DW1000.getData(rxBuffer, len);
    
    // Parse header
    uint8_t msgType = rxBuffer[0];
    uint8_t senderID = rxBuffer[1];
    uint8_t targetID = (len > 2) ? rxBuffer[2] : 0xFF;
    
    Serial.println("\n<<< MESSAGE RECEIVED <<<");
    Serial.print("  Total RX count: ");
    Serial.println(totalMessagesReceived);
    Serial.print("  Type: ");
    Serial.print(msgType);
    Serial.print(" (");
    switch(msgType) {
        case MSG_ANNOUNCE: Serial.print("ANNOUNCE"); break;
        case MSG_POLL: Serial.print("POLL"); break;
        case MSG_RESP: Serial.print("RESP"); break;
        case MSG_FINAL: Serial.print("FINAL"); break;
        default: Serial.print("UNKNOWN"); break;
    }
    Serial.println(")");
    Serial.print("  From: Device ");
    Serial.println(senderID);
    Serial.print("  To: Device ");
    Serial.println(targetID);
    Serial.print("  Current state: ");
    Serial.println(currentState == STATE_IDLE ? "IDLE" : "RANGING");
    
    // Update neighbor table (all messages indicate presence)
    updateNeighbor(senderID);
    
    // Process based on message type
    switch (msgType) {
        case MSG_ANNOUNCE:
            processAnnounce(senderID);
            break;
            
        case MSG_POLL:
            pollsReceived++;
            if (targetID == myDeviceID) {
                processPoll(senderID);
            } else {
                Serial.println("  → Not for me, ignoring");
            }
            break;
            
        case MSG_RESP:
            if (targetID == myDeviceID && currentState == STATE_RANGING) {
                respsReceived++;
                Serial.println("  → RESP for me! Processing...");
                processResp(senderID);
            } else {
                Serial.print("  → Ignoring (targetID=");
                Serial.print(targetID);
                Serial.print(", myID=");
                Serial.print(myDeviceID);
                Serial.print(", state=");
                Serial.print(currentState);
                Serial.println(")");
            }
            break;
            
        case MSG_FINAL:
            if (targetID == myDeviceID) {
                finalsReceived++;
                processFinal(senderID);
            } else {
                Serial.println("  → Not for me, ignoring");
            }
            break;
            
        default:
            Serial.println("  → Unknown message type!");
    }
    
    Serial.println(">>> END MESSAGE <<<\n");
}

// ============================================================================
// HANDLE SENT MESSAGE
// ============================================================================
void handleSentMessage() {
    // Message transmitted successfully
    // enterReceiveMode is now called in each send function
    // This handler is just for confirmation
}

// ============================================================================
// PROCESS ANNOUNCE
// ============================================================================
void processAnnounce(uint8_t senderID) {
    // Neighbor discovery happens in updateNeighbor()
    // Nothing else needed here
}

// ============================================================================
// PROCESS POLL (Received ranging request)
// ============================================================================
void processPoll(uint8_t senderID) {
    Serial.print("← POLL from device ");
    Serial.print(senderID);
    Serial.print(" (state: ");
    Serial.print(currentState);
    Serial.println(")");
    
    // Capture receive timestamp immediately
    DW1000.getReceiveTimestamp(timePollReceived);
    
    // Send response
    sendResp(senderID);
}

// ============================================================================
// PROCESS RESP (Received response to our poll)
// ============================================================================
void processResp(uint8_t senderID) {
    Serial.print("← RESP from device ");
    Serial.print(senderID);
    Serial.print(" (expected from: ");
    Serial.print(rangingTarget);
    Serial.println(")");
    
    // Verify this is from the device we're ranging with
    if (senderID != rangingTarget) {
        Serial.println("⚠ RESP from unexpected device, ignoring");
        return;
    }
    
    // Capture receive timestamp
    DW1000.getReceiveTimestamp(timePollAckReceived);
    
    // Send final message
    sendFinal(senderID);
}

// ============================================================================
// PROCESS FINAL (Received final message - compute distance!)
// ============================================================================
void processFinal(uint8_t senderID) {
    Serial.print("← FINAL from device ");
    Serial.print(senderID);
    Serial.println();
    
    // Capture receive timestamp
    DW1000.getReceiveTimestamp(timeRangeReceived);
    
    // Compute distance using DS-TWR
    computeDistance(senderID);
    
    // Return to idle state
    currentState = STATE_IDLE;
    Serial.println("  Ranging complete, back to IDLE");
}

// ============================================================================
// COMPUTE DISTANCE (DS-TWR Algorithm)
// ============================================================================
void computeDistance(uint8_t neighborID) {
    // DS-TWR Formula:
    // tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)
    // where:
    //   Ra = round trip time at initiator
    //   Rb = round trip time at responder
    //   Da = reply time at initiator
    //   Db = reply time at responder
    
    // Get timestamps as doubles (in DW1000 time units)
    double Ra = (timePollAckReceived - timePollSent).getAsFloat();
    double Rb = (timeRangeReceived - timePollAckSent).getAsFloat();
    double Da = (timeRangeSent - timePollAckReceived).getAsFloat();
    double Db = (timePollAckSent - timePollReceived).getAsFloat();
    
    // Calculate time-of-flight
    double tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);
    
    // Convert to distance (speed of light = 299,702,547 m/s)
    // DW1000 time unit = 1/(499.2 MHz * 128) = ~15.65 picoseconds
    double distance = tof * 0.0046917639786159;  // Convert DW1000 units to meters
    
    // Validate measurement
    if (distance > 0.0 && distance < 300.0) {
        // Update neighbor table
        int idx = findNeighbor(neighborID);
        if (idx >= 0) {
            neighbors[idx].lastDistance = distance;
            neighbors[idx].rangeCount++;
        }
        
        totalRanges++;
        
        Serial.print("✓ Distance to device ");
        Serial.print(neighborID);
        Serial.print(": ");
        Serial.print(distance, 3);
        Serial.println(" m");
    } else {
        Serial.println("⚠ Invalid distance measurement");
        failedRanges++;
    }
}

// ============================================================================
// ATTEMPT RANGING
// Try to range with a neighbor (only if we have lower ID)
// ============================================================================
void attemptRanging() {
    // Only attempt if we're idle
    if (currentState != STATE_IDLE) {
        return;
    }
    
    // Find a valid neighbor to range with
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].valid) continue;
        
        uint8_t neighborID = neighbors[i].id;
        
        // CRITICAL: Only initiate if our ID is lower
        // This prevents both devices from initiating simultaneously
        if (myDeviceID < neighborID) {
            // Random backoff to avoid collisions
            delay(random(0, BACKOFF_MAX_MS));
            
            // Initiate ranging
            sendPoll(neighborID);
            return;
        }
    }
}

// ============================================================================
// UPDATE NEIGHBOR
// Add or update a neighbor in the table
// ============================================================================
void updateNeighbor(uint8_t id) {
    // Don't add ourselves
    if (id == myDeviceID) {
        return;
    }
    
    // Check if neighbor already exists
    int idx = findNeighbor(id);
    
    if (idx >= 0) {
        // Update existing neighbor
        neighbors[idx].lastSeen = millis();
    } else {
        // Add new neighbor
        idx = findFreeNeighborSlot();
        if (idx >= 0) {
            neighbors[idx].id = id;
            neighbors[idx].valid = true;
            neighbors[idx].lastSeen = millis();
            neighbors[idx].lastDistance = 0.0;
            neighbors[idx].rangeCount = 0;
            
            Serial.print("\n✓ New neighbor discovered: Device ");
            Serial.println(id);
            printNeighbors();
        }
    }
}

// ============================================================================
// FIND NEIGHBOR
// Search for a neighbor by ID
// ============================================================================
int findNeighbor(uint8_t id) {
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].valid && neighbors[i].id == id) {
            return i;
        }
    }
    return -1;
}

// ============================================================================
// FIND FREE NEIGHBOR SLOT
// Find an empty slot in the neighbor table
// ============================================================================
int findFreeNeighborSlot() {
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].valid) {
            return i;
        }
    }
    return -1;
}

// ============================================================================
// CLEANUP NEIGHBORS
// Remove neighbors we haven't heard from recently
// ============================================================================
void cleanupNeighbors() {
    uint32_t now = millis();
    
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].valid && 
            (now - neighbors[i].lastSeen > NEIGHBOR_TIMEOUT_MS)) {
            
            Serial.print("\n⚠ Neighbor timeout: Device ");
            Serial.println(neighbors[i].id);
            
            neighbors[i].valid = false;
            printNeighbors();
        }
    }
}

// ============================================================================
// PRINT NEIGHBORS
// Display current neighbor table
// ============================================================================
void printNeighbors() {
    Serial.println("\n--- Neighbor Table ---");
    
    bool foundAny = false;
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].valid) {
            foundAny = true;
            Serial.print("  Device ");
            Serial.print(neighbors[i].id);
            Serial.print(": ");
            
            if (neighbors[i].lastDistance > 0.0) {
                Serial.print(neighbors[i].lastDistance, 3);
                Serial.print(" m (");
                Serial.print(neighbors[i].rangeCount);
                Serial.println(" ranges)");
            } else {
                Serial.println("No distance yet");
            }
        }
    }
    
    if (!foundAny) {
        Serial.println("  No neighbors");
    }
    Serial.println("----------------------\n");
}

// ============================================================================
// PRINT STATISTICS
// Display system statistics
// ============================================================================
void printStatistics() {
    Serial.println("\n======== STATISTICS ========");
    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
    Serial.print("Total ranges: ");
    Serial.println(totalRanges);
    Serial.print("Failed ranges: ");
    Serial.println(failedRanges);
    
    if (totalRanges + failedRanges > 0) {
        float successRate = 100.0 * totalRanges / (totalRanges + failedRanges);
        Serial.print("Success rate: ");
        Serial.print(successRate, 1);
        Serial.println("%");
    }
    
    Serial.println("============================\n");
    
    printNeighbors();
}

/*
 * ============================================================================
 * USAGE INSTRUCTIONS
 * ============================================================================
 * 
 * 1. Set myDeviceID to unique value for each device (1, 2, 3, etc.)
 * 2. Upload to all devices
 * 3. Power on devices
 * 4. Open Serial Monitor (115200 baud)
 * 
 * EXPECTED BEHAVIOR:
 * - Devices discover each other automatically (dots appear)
 * - "New neighbor discovered" messages appear
 * - Lower ID device initiates ranging
 * - Distance measurements appear continuously
 * - Statistics printed every 10 seconds
 * 
 * TUNING PARAMETERS:
 * - ANNOUNCE_INTERVAL_MS: How often to broadcast presence
 * - RANGE_INTERVAL_MS: How often to attempt ranging
 * - BACKOFF_MAX_MS: Maximum random delay (collision avoidance)
 * 
 * CALIBRATION:
 * - Antenna delay (16384) MUST be calibrated for accurate distances
 * - Place devices at known distance (1m, 5m, 10m)
 * - Adjust antenna delay until measurement matches reality
 * 
 * ============================================================================
 */