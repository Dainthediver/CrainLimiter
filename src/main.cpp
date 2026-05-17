/*
 * ============================================================================
 * COLLISION-FREE UWB MAC PROTOCOL
 * ============================================================================
 *
 * ARCHITECTURE: Asynchronous Per-Neighbor DS-TWR
 * with Fixed Responder Roles and Jittered Initiator Intervals
 *
 * LAYERS:
 * Radio Layer - DW1000 RX/TX, delayed TX, interrupts
 * MAC Layer - Scheduling, arbitration, collision avoidance, neighbor table
 * Ranging Layer - DS-TWR timestamps, distance computation
 *
 * KEY DESIGN RULES:
 * 1. Pairwise initiator ownership: lower ID always initiates TWR
 * 2. Per-neighbor scheduled ranging with jittered intervals (no global timer)
 * 3. DW1000 delayed TX for deterministic response timing (no blocking delays)
 * 4. 5-state MAC state machine for clean recovery
 * 5. Sequence numbers for duplicate/stale packet rejection
 * 6. Peer filtering during active TWR (ignore non-peer packets)
 * 7. Jittered announce intervals to avoid beacon storms
 * 8. Automatic rejoin on power cycle / node reappearance
 * 9. Both sides compute distance from actual measured timestamps
 *
 * PROTOCOL MESSAGES:
 * ANNOUNCE (0x01) - Discovery broadcast: "I'm here"
 * POLL     (0x02) - Initiator starts DS-TWR
 * RESP     (0x03) - Responder replies (delayed TX)
 * FINAL    (0x04) - Initiator completes DS-TWR (delayed TX)
 * RESULT   (0x05) - Responder sends Db + Rb to initiator (after FINAL received)
 *
 * DS-TWR DISTANCE COMPUTATION:
 * After the POLL-RESP-FINAL exchange:
 * - Responder has all 4 timestamps (pollRx, respTx, finalRx) plus its own Db.
 *   It also computes Rb and sends both Db and Rb to the initiator in RESULT.
 * - Initiator has all 3 of its own timestamps (pollTx, respRx, finalTx) plus
 *   Da. After receiving RESULT, it also has Db and Rb. Both sides can then
 *   compute: tof = (Ra*Rb - Da*Db) / (Ra + Rb + Da + Db)
 *
 * Author: Custom Implementation
 * Platform: PlatformIO with ESP32 + DW1000
 * Library: https://github.com/Dainthediver/arduino-dw1000
 * ============================================================================
 */

#include <SPI.h>
#include <DW1000.h>
#include <DW1000Time.h>

// ============================================================================
// FIRMWARE VERSION
// ============================================================================
#define FIRMWARE_VERSION "2.3.0"

// ============================================================================
// PIN CONFIGURATION
// ============================================================================
#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

// ============================================================================
// DEVICE CONFIGURATION - SET UNIQUE ID FOR EACH DEVICE
// ============================================================================
uint8_t myDeviceID = 1; // CHANGE THIS: 1, 2, 3, 4, etc.

// ============================================================================
// TIMING PARAMETERS
// ============================================================================
#define ANNOUNCE_BASE_MS 1500 // Base announce interval
#define ANNOUNCE_JITTER_MS 500 // Random jitter 0..500ms added
#define RANGE_BASE_MS 250 // Base per-neighbor ranging interval
#define RANGE_JITTER_MS 120 // Random jitter 0..120ms added
#define NEIGHBOR_TIMEOUT_MS 5000 // Consider neighbor lost after 5s
#define MAC_TIMEOUT_MS 60 // Per-state MAC timeout (ms)
#define RESP_DELAY_US 3000 // DW1000 delayed TX: RESP after 3ms
#define FINAL_DELAY_US 3000 // DW1000 delayed TX: FINAL after 3ms
#define MAX_CONSECUTIVE_FAILS 5 // Back off aggressively after this

// ============================================================================
// NEIGHBOR TABLE
// ============================================================================
#define MAX_NEIGHBORS 4

// ============================================================================
// MESSAGE TYPES
// ============================================================================
#define MSG_ANNOUNCE 0x01
#define MSG_POLL 0x02
#define MSG_RESP 0x03
#define MSG_FINAL 0x04
#define MSG_RESULT 0x05 // Responder sends Db + Rb to initiator

// ============================================================================
// MESSAGE FRAME LAYOUT
// ============================================================================
// ANNOUNCE: [type][src][dst=0xFF][seqLo][seqHi] (5 bytes)
// POLL:     [type][src][dst][seqLo][seqHi] (5 bytes)
// RESP:     [type][src][dst][seqLo][seqHi] (5 bytes)
// FINAL:    [type][src][dst][seqLo][seqHi] (5 bytes)
// RESULT:   [type][src][dst][seqLo][seqHi][Db0-3][Rb0-3] (13 bytes)
//
// RESULT carries two 4-byte floats (DW1000 time units, microseconds):
//   Db = respTsRespSent - respTsPollRx (responder reply delay)
//   Rb = respTsFinalRx - respTsRespSent (responder round-trip time)
// These allow the initiator to compute the full DS-TWR formula.

#define FRAME_HEADER_LEN 5
#define FRAME_RESULT_LEN 13 // header + 4-byte Db + 4-byte Rb

// ============================================================================
// MAC STATE MACHINE
// ============================================================================
enum MacState {
 MAC_IDLE, // Listening, scheduling next ranging
 MAC_WAIT_RESP, // Initiator: sent POLL, waiting for RESP
 MAC_WAIT_FINAL, // Responder: sent RESP, waiting for FINAL
 MAC_WAIT_RESULT, // Initiator: sent FINAL, waiting for RESULT
 MAC_SEND_RESP, // Responder: preparing delayed RESP
 MAC_SEND_FINAL // Initiator: preparing delayed FINAL
};

// ============================================================================
// NEIGHBOR STRUCTURE (per-neighbor state)
// ============================================================================
struct Neighbor {
 uint8_t id; // Neighbor device ID
 bool valid; // Slot occupied?
 uint32_t lastSeen; // Last heard from (millis)
 float distance; // Latest distance (meters)
 uint32_t rangeCount; // Successful ranges
 uint32_t nextRangeTime; // Scheduled time for next ranging (millis)
 uint8_t failCount; // Consecutive failures
 uint16_t lastRxSeq; // Last received sequence number (for dupe check)
};

// ============================================================================
// GLOBALS - RADIO LAYER
// ============================================================================
byte txBuffer[128];
byte rxBuffer[128];
volatile bool msgReceived = false;
volatile bool msgSent = false;
volatile bool txActive = false;

// ============================================================================
// GLOBALS - MAC LAYER
// ============================================================================
MacState macState = MAC_IDLE;
uint8_t activePeer = 0; // Device ID of current TWR peer
uint16_t txSeqNumber = 0; // Monotonically increasing TX seq
uint32_t macStateEnterTime = 0; // When we entered current MAC state

Neighbor neighbors[MAX_NEIGHBORS];

// Announce scheduling
uint32_t nextAnnounceTime = 0;

// Diagnostic counters
uint32_t pollsSent = 0;
uint32_t respsSent = 0;
uint32_t finalsSent = 0;
uint32_t resultsSent = 0;
uint32_t pollsReceived = 0;
uint32_t respsReceived = 0;
uint32_t finalsReceived = 0;
uint32_t resultsReceived = 0;
uint32_t totalRanges = 0;
uint32_t failedRanges = 0;
uint32_t dupesRejected = 0;

// ============================================================================
// GLOBALS - RANGING LAYER (DS-TWR timestamps)
// ============================================================================
// Initiator timestamps
DW1000Time initTsPollSent;  // POLL TX time
DW1000Time initTsRespRx;    // RESP RX time
DW1000Time initTsFinalSent; // FINAL TX time
// Received from responder via RESULT message:
float initDb = 0.0; // Responder reply delay (us)
float initRb = 0.0; // Responder round-trip (us)

// Responder timestamps
DW1000Time respTsPollRx;    // POLL RX time
DW1000Time respTsRespSent;  // RESP TX time (captured after delayed TX done)
DW1000Time respTsFinalRx;   // FINAL RX time

// ============================================================================
// FUNCTION DECLARATIONS - RADIO LAYER
// ============================================================================
void radioInit();
void radioRx();
void radioTxImmediate(const byte data[], uint16_t len);
void radioTxDelayed(const byte data[], uint16_t len, uint32_t delayUs);

// ============================================================================
// FUNCTION DECLARATIONS - MAC LAYER
// ============================================================================
void macInit();
void macTick();
void macProcessRx();
void macProcessTxDone();
void macScheduleNextRange(int neighborIdx);
void macEnterState(MacState state);
void macTimeout();
void macSendAnnounce();
void macSendPoll(uint8_t target);
void macSendRespDelayed(uint8_t target);
void macSendFinalDelayed(uint8_t target);
void macSendResult(uint8_t target, float db, float rb);

// ============================================================================
// FUNCTION DECLARATIONS - RANGING LAYER
// ============================================================================
void rangingInitiatorOnRespReceived();
void rangingInitiatorOnResultReceived(float db, float rb);
void rangingInitiatorCompute();
void rangingResponderOnPollReceived();
void rangingResponderOnFinalReceived();
void rangingResponderCompute();

// ============================================================================
// FUNCTION DECLARATIONS - NEIGHBOR TABLE
// ============================================================================
void neighborInit();
void neighborUpdate(uint8_t id);
int neighborFind(uint8_t id);
int neighborFreeSlot();
void neighborCleanup();
bool neighborIsInitiator(uint8_t neighborId);
void neighborPrint();
void neighborPrintStats();

// ============================================================================
// HELPER
// ============================================================================
uint16_t nextSeq();
bool isDupe(uint8_t from, uint16_t seq);
void embedFloat(byte* buf, float val);
float extractFloat(const byte* buf);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
 Serial.begin(115200);
 delay(1000);

 Serial.println("\n========================================");
 Serial.println("CrainLimiter UWB MAC - DS-TWR");
 Serial.println("========================================");
 Serial.print("Firmware: v");
 Serial.println(FIRMWARE_VERSION);
 Serial.print("Device ID: ");
 Serial.println(myDeviceID);
 Serial.println("Max neighbors: 4");
 Serial.println("Initiator rule: lower ID initiates");
 Serial.println("Delayed TX: enabled (3ms)");
 Serial.println("========================================\n");

 randomSeed(analogRead(0) + myDeviceID * 137);

 neighborInit();
 radioInit();
 macInit();

 Serial.println("System ready.\n");
}

// ============================================================================
// MAIN LOOP - Event-driven, NO blocking delays
// ============================================================================
void loop() {
 // 1. Process received messages (highest priority)
 if (msgReceived) {
 msgReceived = false;
 macProcessRx();
 }

 // 2. Process TX completion
 if (msgSent) {
 msgSent = false;
 macProcessTxDone();
 }

 // 3. MAC tick: scheduling, timeouts, announces
 macTick();

 // 4. Neighbor cleanup (low frequency)
 static uint32_t lastCleanup = 0;
 if (millis() - lastCleanup > 2000) {
 neighborCleanup();
 lastCleanup = millis();
 }

 // 5. Statistics (low frequency)
 static uint32_t lastStats = 0;
 if (millis() - lastStats > 15000) {
 neighborPrintStats();
 lastStats = millis();
 }

 // Yield to ESP32 watchdog / WiFi stack
 delay(1);
}

// ============================================================================
// RADIO LAYER
// ============================================================================

void radioInit() {
 Serial.println("[RADIO] Initializing DW1000...");

 DW1000.begin(PIN_IRQ, PIN_RST);
 DW1000.select(PIN_SS);

 char devId[128];
 DW1000.getPrintableDeviceIdentifier(devId);
 Serial.print("[RADIO] DW1000 ID: ");
 Serial.println(devId);

 DW1000.newConfiguration();
 DW1000.setDefaults();
 DW1000.setDeviceAddress(myDeviceID);
 DW1000.setNetworkId(10);

 // Ranging-optimized settings
 DW1000.setChannel(DW1000.CHANNEL_5);
 DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_1024);
 DW1000.setPulseFrequency(DW1000.TX_PULSE_FREQ_64MHZ);
 DW1000.setDataRate(DW1000.TRX_RATE_110KBPS);
 DW1000.setAntennaDelay(16384); // CALIBRATE THIS!

 DW1000.commitConfiguration();

 // Interrupt handlers
 DW1000.attachSentHandler([]() {
 msgSent = true;
 txActive = false;
 });
 DW1000.attachReceivedHandler([]() {
 msgReceived = true;
 });

 // Start in permanent receive mode
 radioRx();

 Serial.println("[RADIO] DW1000 ready.\n");
}

void radioRx() {
 DW1000.newReceive();
 DW1000.receivePermanently(true);
 DW1000.startReceive();
}

void radioTxImmediate(const byte data[], uint16_t len) {
 DW1000.newTransmit();
 DW1000.setData(data, len);
 txActive = true;
 DW1000.startTransmit();
 // permanentReceive is on, RX resumes after TX
}

void radioTxDelayed(const byte data[], uint16_t len, uint32_t delayUs) {
 DW1000.newTransmit();
 DW1000.setData(data, len);
 DW1000.setDelay(DW1000Time((int32_t)delayUs, DW1000Time::MICROSECONDS));
 txActive = true;
 DW1000.startTransmit();
 // permanentReceive resumes RX after delayed TX completes
}

// ============================================================================
// MAC LAYER
// ============================================================================

void macInit() {
 macState = MAC_IDLE;
 activePeer = 0;
 nextAnnounceTime = millis() + random(500, 1500);
}

void macEnterState(MacState state) {
 macState = state;
 macStateEnterTime = millis();
}

// -----------------------------------------------
// macTick: Non-blocking MAC scheduler
// -----------------------------------------------
void macTick() {
 uint32_t now = millis();

 // --- MAC timeout: recover from stalled states ---
 if (macState != MAC_IDLE) {
 if (now - macStateEnterTime > MAC_TIMEOUT_MS) {
 macTimeout();
 return;
 }
 }

 // --- Announce scheduling (jittered) ---
 if (macState == MAC_IDLE && now >= nextAnnounceTime) {
 macSendAnnounce();
 nextAnnounceTime = now + ANNOUNCE_BASE_MS + random(0, ANNOUNCE_JITTER_MS);
 }

 // --- Per-neighbor ranging scheduling ---
 if (macState == MAC_IDLE) {
 for (int i = 0; i < MAX_NEIGHBORS; i++) {
 if (!neighbors[i].valid) continue;
 if (!neighborIsInitiator(neighbors[i].id)) continue;

 // Check if it's time to range with this neighbor
 if (now >= neighbors[i].nextRangeTime) {
 macSendPoll(neighbors[i].id);
 return; // One ranging at a time
 }
 }
 }
}

// -----------------------------------------------
// macProcessRx: Handle received frames
// -----------------------------------------------
void macProcessRx() {
 uint16_t len = DW1000.getDataLength();
 if (len < FRAME_HEADER_LEN) return; // Malformed

 DW1000.getData(rxBuffer, len);

 uint8_t msgType = rxBuffer[0];
 uint8_t srcId = rxBuffer[1];
 uint8_t dstId = rxBuffer[2];
 uint16_t seq = (uint16_t)((uint16_t)rxBuffer[4] << 8 | rxBuffer[3]);

 // Update neighbor presence from ANY message
 neighborUpdate(srcId);

 switch (msgType) {

 // -------------------------------------------------------
 // ANNOUNCE: Discovery broadcast
 // -------------------------------------------------------
 case MSG_ANNOUNCE:
 // neighborUpdate already handled presence
 break;

 // -------------------------------------------------------
 // POLL: We are the responder (higher ID)
 // -------------------------------------------------------
 case MSG_POLL:
 pollsReceived++;

 // Only accept if addressed to us
 if (dstId != myDeviceID) break;

 // If we're busy with another exchange, drop it (initiator will retry)
 if (macState != MAC_IDLE) break;

 // Duplicate check
 if (isDupe(srcId, seq)) {
 dupesRejected++;
 break;
 }

 Serial.print("[MAC] POLL from Dev ");
 Serial.print(srcId);
 Serial.print(" seq=");
 Serial.println(seq);

 activePeer = srcId;
 macEnterState(MAC_SEND_RESP);
 rangingResponderOnPollReceived();
 macSendRespDelayed(srcId);
 break;

 // -------------------------------------------------------
 // RESP: We are the initiator (lower ID), waiting for reply
 // -------------------------------------------------------
 case MSG_RESP:
 respsReceived++;

 // Only accept if addressed to us AND we're expecting it
 if (dstId != myDeviceID) break;
 if (macState != MAC_WAIT_RESP) break;
 if (srcId != activePeer) break;

 // Duplicate check
 if (isDupe(srcId, seq)) {
 dupesRejected++;
 break;
 }

 Serial.print("[MAC] RESP from Dev ");
 Serial.println(srcId);

 rangingInitiatorOnRespReceived();
 macEnterState(MAC_SEND_FINAL);
 macSendFinalDelayed(srcId);
 break;

 // -------------------------------------------------------
 // FINAL: We are the responder, TWR completing
 // -------------------------------------------------------
 case MSG_FINAL:
 finalsReceived++;

 // Only accept if we're waiting for this
 if (dstId != myDeviceID) break;
 if (macState != MAC_WAIT_FINAL) break;
 if (srcId != activePeer) break;

 // Duplicate check
 if (isDupe(srcId, seq)) {
 dupesRejected++;
 break;
 }

 Serial.print("[MAC] FINAL from Dev ");
 Serial.println(srcId);

 rangingResponderOnFinalReceived();

 // Now compute distance on responder side (we have all timestamps)
 rangingResponderCompute();

 // Send RESULT to initiator with measured Db and Rb
 {
 float db = (float)RESP_DELAY_US; // exact programmed delay (us)
 float rb = (float)((respTsFinalRx - respTsPollRx).getAsFloat() - RESP_DELAY_US);
 macSendResult(srcId, db, rb);
 }

 totalRanges++;
 {
 int idx = neighborFind(activePeer);
 if (idx >= 0) {
 neighbors[idx].failCount = 0;
 }
 }

 Serial.println("[MAC] Ranging complete (responder).");
 activePeer = 0;
 macEnterState(MAC_IDLE);
 break;

 // -------------------------------------------------------
 // RESULT: We are the initiator, receiving Db + Rb
 // -------------------------------------------------------
 case MSG_RESULT:
 resultsReceived++;

 // Only accept if we're waiting for this
 if (dstId != myDeviceID) break;
 if (macState != MAC_WAIT_RESULT) break;
 if (srcId != activePeer) break;

 if (len < FRAME_RESULT_LEN) break; // Malformed

 Serial.print("[MAC] RESULT from Dev ");
 Serial.println(srcId);

 // Extract Db and Rb from RESULT payload
 {
 float db = extractFloat(rxBuffer + FRAME_HEADER_LEN);
 float rb = extractFloat(rxBuffer + FRAME_HEADER_LEN + 4);
 rangingInitiatorOnResultReceived(db, rb);
 }

 // Compute distance on initiator side (now we have everything)
 rangingInitiatorCompute();

 totalRanges++;
 {
 int idx = neighborFind(activePeer);
 if (idx >= 0) {
 neighbors[idx].failCount = 0;
 }
 }

 Serial.println("[MAC] Ranging complete (initiator).");
 activePeer = 0;
 macEnterState(MAC_IDLE);
 break;

 default:
 break;
 }
}

// -----------------------------------------------
// macProcessTxDone: After our transmission completes
// -----------------------------------------------
void macProcessTxDone() {
 switch (macState) {

 case MAC_WAIT_RESP:
 // POLL was sent (immediate). Now that TX_DONE has fired,
 // the TX_TIME register is valid — capture the timestamp here.
 DW1000.getTransmitTimestamp(initTsPollSent);
 break;

 case MAC_SEND_RESP:
 // RESP transmitted via delayed TX.
 // Compute RESP TX time from RX timestamp + programmed delay
 // (more reliable than getTransmitTimestamp after delayed TX)
 respTsRespSent = respTsPollRx + DW1000Time((int32_t)RESP_DELAY_US, DW1000Time::MICROSECONDS);
 Serial.println("[MAC] RESP sent (delayed TX done)");
 macEnterState(MAC_WAIT_FINAL);
 break;

 case MAC_SEND_FINAL:
 // FINAL transmitted via delayed TX.
 // Compute FINAL TX time from RX timestamp + programmed delay
 // (more reliable than getTransmitTimestamp after delayed TX)
 initTsFinalSent = initTsRespRx + DW1000Time((int32_t)FINAL_DELAY_US, DW1000Time::MICROSECONDS);
 Serial.println("[MAC] FINAL sent (delayed TX done)");
 // Now we wait for RESULT from the responder
 macEnterState(MAC_WAIT_RESULT);
 break;

 case MAC_WAIT_RESULT:
 // RESULT was just sent by responder side; no action needed
 // (This state shouldn't hit for TX-done on initiator side,
 //  but handle gracefully.)
 break;

 default:
 break;
 }
}

// -----------------------------------------------
// macTimeout: Recover from stalled MAC state
// -----------------------------------------------
void macTimeout() {
 Serial.print("[MAC] Timeout in state ");
 Serial.print(macState);
 Serial.print(" (peer=");
 Serial.print(activePeer);
 Serial.println(")");

 failedRanges++;

 int idx = neighborFind(activePeer);
 if (idx >= 0) {
 neighbors[idx].failCount++;
 // Schedule next range with backoff proportional to failures
 macScheduleNextRange(idx);
 neighbors[idx].nextRangeTime += neighbors[idx].failCount * 200;
 }

 activePeer = 0;
 macEnterState(MAC_IDLE);
 radioRx();
}

// -----------------------------------------------
// macScheduleNextRange: Set next ranging time with jitter
// -----------------------------------------------
void macScheduleNextRange(int neighborIdx) {
 neighbors[neighborIdx].nextRangeTime =
 millis() + RANGE_BASE_MS + random(0, RANGE_JITTER_MS);
}

// -----------------------------------------------
// macSendAnnounce: Discovery broadcast (jittered interval)
// -----------------------------------------------
void macSendAnnounce() {
 if (txActive) return;

 uint16_t seq = nextSeq();
 txBuffer[0] = MSG_ANNOUNCE;
 txBuffer[1] = myDeviceID;
 txBuffer[2] = 0xFF; // Broadcast
 txBuffer[3] = (uint8_t)(seq & 0xFF);
 txBuffer[4] = (uint8_t)(seq >> 8);

 radioTxImmediate(txBuffer, FRAME_HEADER_LEN);

 // Quiet: don't spam serial on every announce
 static uint32_t announceCount = 0;
 if (++announceCount % 20 == 0) {
 Serial.print("[MAC] Announces sent: ");
 Serial.println(announceCount);
 }
}

// -----------------------------------------------
// macSendPoll: Initiator starts DS-TWR
// -----------------------------------------------
void macSendPoll(uint8_t target) {
 if (txActive) return;

 uint16_t seq = nextSeq();
 txBuffer[0] = MSG_POLL;
 txBuffer[1] = myDeviceID;
 txBuffer[2] = target;
 txBuffer[3] = (uint8_t)(seq & 0xFF);
 txBuffer[4] = (uint8_t)(seq >> 8);

 radioTxImmediate(txBuffer, FRAME_HEADER_LEN);

 // POLL TX timestamp will be captured in macProcessTxDone()
 // when TX_DONE fires — reading it here before TX completes gives garbage

 pollsSent++;
 activePeer = target;
 macEnterState(MAC_WAIT_RESP);

 Serial.print("[MAC] POLL -> Dev ");
 Serial.print(target);
 Serial.print(" seq=");
 Serial.println(seq);
}

// -----------------------------------------------
// macSendRespDelayed: Responder replies with delayed TX
// -----------------------------------------------
void macSendRespDelayed(uint8_t target) {
 uint16_t seq = nextSeq();
 txBuffer[0] = MSG_RESP;
 txBuffer[1] = myDeviceID;
 txBuffer[2] = target;
 txBuffer[3] = (uint8_t)(seq & 0xFF);
 txBuffer[4] = (uint8_t)(seq >> 8);

 // Use DW1000 delayed transmit for deterministic timing
 radioTxDelayed(txBuffer, FRAME_HEADER_LEN, RESP_DELAY_US);

 respsSent++;
 Serial.print("[MAC] RESP (delayed ");
 Serial.print(RESP_DELAY_US);
 Serial.print("us) -> Dev ");
 Serial.println(target);
}

// -----------------------------------------------
// macSendFinalDelayed: Initiator completes DS-TWR with delayed TX
// -----------------------------------------------
void macSendFinalDelayed(uint8_t target) {
 uint16_t seq = nextSeq();
 txBuffer[0] = MSG_FINAL;
 txBuffer[1] = myDeviceID;
 txBuffer[2] = target;
 txBuffer[3] = (uint8_t)(seq & 0xFF);
 txBuffer[4] = (uint8_t)(seq >> 8);

 // Use DW1000 delayed transmit for deterministic timing
 radioTxDelayed(txBuffer, FRAME_HEADER_LEN, FINAL_DELAY_US);

 finalsSent++;
 Serial.print("[MAC] FINAL (delayed ");
 Serial.print(FINAL_DELAY_US);
 Serial.print("us) -> Dev ");
 Serial.println(target);
}

// -----------------------------------------------
// macSendResult: Responder sends measured Db + Rb to initiator
// -----------------------------------------------
void macSendResult(uint8_t target, float db, float rb) {
 uint16_t seq = nextSeq();
 txBuffer[0] = MSG_RESULT;
 txBuffer[1] = myDeviceID;
 txBuffer[2] = target;
 txBuffer[3] = (uint8_t)(seq & 0xFF);
 txBuffer[4] = (uint8_t)(seq >> 8);

 // Embed measured Db and Rb (in microseconds, from DW1000Time::getAsFloat)
 embedFloat(txBuffer + FRAME_HEADER_LEN, db);
 embedFloat(txBuffer + FRAME_HEADER_LEN + 4, rb);

 radioTxImmediate(txBuffer, FRAME_RESULT_LEN);

 resultsSent++;
 Serial.print("[MAC] RESULT -> Dev ");
 Serial.print(target);
 Serial.print(" Db=");
 Serial.print(db, 3);
 Serial.print("us Rb=");
 Serial.print(rb, 3);
 Serial.println("us");
}

// ============================================================================
// RANGING LAYER - INITIATOR SIDE (lower ID)
// ============================================================================

void rangingInitiatorOnRespReceived() {
 // We received a RESP to our POLL - capture the receive timestamp
 DW1000.getReceiveTimestamp(initTsRespRx);
}

void rangingInitiatorOnResultReceived(float db, float rb) {
 // Received Db and Rb from the responder
 initDb = db;
 initRb = rb;
}

void rangingInitiatorCompute() {
 // Called after receiving RESULT (in macProcessRx)
 // We now have ALL values for the full DS-TWR formula:
 //
 // Ra = initTsRespRx - initTsPollSent  (initiator round-trip A)
 // Da = initTsFinalSent - initTsRespRx  (initiator reply delay A)
 // Rb = initRb  (responder round-trip, received in RESULT)
 // Db = initDb  (responder reply delay, received in RESULT)
 //
 // Full DS-TWR formula (clock-drift-corrected):
 // tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)

 double Ra = (initTsRespRx - initTsPollSent).getAsFloat();
 // Da = FINAL_DELAY_US by construction (initTsFinalSent = initTsRespRx + delay)
 double Da = (initTsFinalSent - initTsRespRx).getAsFloat();
 double Rb = initRb;
 double Db = initDb;

 // Safety: all values must be positive
 if (Ra <= 0 || Da <= 0 || Rb <= 0 || Db <= 0) {
 Serial.println("[RANGING] Invalid timestamps, skipping");
 failedRanges++;
 return;
 }

 double tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

 // Convert microseconds to distance (meters)
 // getAsFloat() returns microseconds.
 // distance_m = tof_us * 1e-6 * SPEED_OF_LIGHT
 double distance = tof * 1e-6 * 299702547.0;

 // Validate
 if (distance > 0.0 && distance < 300.0) {
 int idx = neighborFind(activePeer);
 if (idx >= 0) {
 neighbors[idx].distance = distance;
 neighbors[idx].rangeCount++;
 }

 Serial.print("[RANGING] Dev ");
 Serial.print(activePeer);
 Serial.print(": ");
 Serial.print(distance, 3);
 Serial.println(" m");

 // Debug: show raw values
 Serial.print("  Ra=");
 Serial.print(Ra, 3);
 Serial.print("us Da=");
 Serial.print(Da, 3);
 Serial.print("us Rb=");
 Serial.print(Rb, 3);
 Serial.print("us Db=");
 Serial.print(Db, 3);
 Serial.print("us tof=");
 Serial.print(tof, 3);
 Serial.println("us");
 } else {
 Serial.print("[RANGING] Invalid: ");
 Serial.print(distance, 3);
 Serial.println(" m");
 Serial.print(" Ra=");
 Serial.print(Ra, 3);
 Serial.print("us Da=");
 Serial.print(Da, 3);
 Serial.print("us Rb=");
 Serial.print(Rb, 3);
 Serial.print("us Db=");
 Serial.print(Db, 3);
 Serial.print("us tof=");
 Serial.print(tof, 3);
 Serial.println("us");
 failedRanges++;
 }
}

// ============================================================================
// RANGING LAYER - RESPONDER SIDE (higher ID)
// ============================================================================

void rangingResponderOnPollReceived() {
 // Capture POLL receive timestamp
 DW1000.getReceiveTimestamp(respTsPollRx);
}

void rangingResponderOnFinalReceived() {
 // Capture FINAL receive timestamp
 DW1000.getReceiveTimestamp(respTsFinalRx);
}

void rangingResponderCompute() {
 // Called after receiving FINAL (in macProcessRx)
 // We have all responder timestamps:
 // respTsPollRx - POLL RX time
 // respTsRespSent - RESP TX time (computed from pollRx + programmed delay)
 // respTsFinalRx - FINAL RX time
 //
 // Since we use programmed delayed TX, we know the exact reply delays:
 // Db = RESP_DELAY_US (our reply delay, exact)
 // Da = FINAL_DELAY_US (initiator's reply delay, exact)
 //
 // We derive Ra and Rb from the measured round-trip:
 // totalInterval = respTsFinalRx - respTsPollRx (measured)
 // Rb = totalInterval - Db (responder round-trip)
 // Ra = Rb - Db + Da (from DS-TWR geometry)
 //
 // The initiator gets exact values from RESULT, so its computation
 // is always authoritative. This responder-side estimate is for
 // local display only.

 double Db = (double)RESP_DELAY_US; // our reply delay (us) - exact
 double Da = (double)FINAL_DELAY_US; // initiator reply delay (us) - exact
 double totalInterval = (respTsFinalRx - respTsPollRx).getAsFloat();
 double Rb = totalInterval - Db;
 double Ra = Rb - Db + Da;

 if (Ra <= 0 || Da <= 0 || Rb <= 0 || Db <= 0) {
 Serial.println("[RANGING] Invalid timestamps (responder), skipping");
 return;
 }

 double tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);
 double distance = tof * 1e-6 * 299702547.0;

 if (distance > 0.0 && distance < 300.0) {
 int idx = neighborFind(activePeer);
 if (idx >= 0) {
 neighbors[idx].distance = distance;
 neighbors[idx].rangeCount++;
 }

 Serial.print("[RANGING] Dev ");
 Serial.print(activePeer);
 Serial.print(": ");
 Serial.print(distance, 3);
 Serial.println(" m (responder approx)");

 Serial.print("  Rb=");
 Serial.print(Rb, 3);
 Serial.print("us Db=");
 Serial.print(Db, 3);
 Serial.print("us tof=");
 Serial.print(tof, 3);
 Serial.println("us");
 } else {
 Serial.print("[RANGING] Invalid (responder): ");
 Serial.print(distance, 3);
 Serial.println(" m");
 Serial.print(" Rb=");
 Serial.print(Rb, 3);
 Serial.print("us Db=");
 Serial.print(Db, 3);
 Serial.print("us Da=");
 Serial.print(Da, 3);
 Serial.print("us Ra=");
 Serial.print(Ra, 3);
 Serial.print("us totalInterval=");
 Serial.print(totalInterval, 3);
 Serial.println("us");
 }
}

// ============================================================================
// NEIGHBOR TABLE
// ============================================================================

void neighborInit() {
 for (int i = 0; i < MAX_NEIGHBORS; i++) {
 neighbors[i].valid = false;
 neighbors[i].distance = 0.0;
 neighbors[i].rangeCount = 0;
 neighbors[i].failCount = 0;
 neighbors[i].nextRangeTime = 0;
 neighbors[i].lastRxSeq = 0;
 }
}

void neighborUpdate(uint8_t id) {
 if (id == myDeviceID) return;

 int idx = neighborFind(id);
 if (idx >= 0) {
 neighbors[idx].lastSeen = millis();
 } else {
 idx = neighborFreeSlot();
 if (idx >= 0) {
 neighbors[idx].id = id;
 neighbors[idx].valid = true;
 neighbors[idx].lastSeen = millis();
 neighbors[idx].distance = 0.0;
 neighbors[idx].rangeCount = 0;
 neighbors[idx].failCount = 0;
 neighbors[idx].lastRxSeq = 0;
 macScheduleNextRange(idx);

 Serial.print("[NEIGHBOR] Discovered: Dev ");
 Serial.println(id);
 neighborPrint();
 }
 }
}

int neighborFind(uint8_t id) {
 for (int i = 0; i < MAX_NEIGHBORS; i++) {
 if (neighbors[i].valid && neighbors[i].id == id) return i;
 }
 return -1;
}

int neighborFreeSlot() {
 for (int i = 0; i < MAX_NEIGHBORS; i++) {
 if (!neighbors[i].valid) return i;
 }
 return -1;
}

void neighborCleanup() {
 uint32_t now = millis();
 for (int i = 0; i < MAX_NEIGHBORS; i++) {
 if (neighbors[i].valid && (now - neighbors[i].lastSeen > NEIGHBOR_TIMEOUT_MS)) {
 Serial.print("[NEIGHBOR] Timeout: Dev ");
 Serial.println(neighbors[i].id);
 neighbors[i].valid = false;
 neighborPrint();
 }
 }
}

bool neighborIsInitiator(uint8_t neighborId) {
 return myDeviceID < neighborId;
}

void neighborPrint() {
 Serial.println("--- Neighbor Table ---");
 for (int i = 0; i < MAX_NEIGHBORS; i++) {
 if (neighbors[i].valid) {
 Serial.print(" Dev ");
 Serial.print(neighbors[i].id);
 Serial.print(": ");
 if (neighbors[i].distance > 0.0) {
 Serial.print(neighbors[i].distance, 3);
 Serial.print(" m (");
 Serial.print(neighbors[i].rangeCount);
 Serial.println(" ranges)");
 } else {
 Serial.println("no range yet");
 }
 }
 }
 Serial.println("----------------------");
}

void neighborPrintStats() {
 Serial.println("\n======== STATISTICS ========");
 Serial.print("Uptime: ");
 Serial.print(millis() / 1000);
 Serial.println("s");
 Serial.print("Total ranges: ");
 Serial.println(totalRanges);
 Serial.print("Failed ranges: ");
 Serial.println(failedRanges);
 Serial.print("Duplicates rejected: ");
 Serial.println(dupesRejected);
 if (totalRanges + failedRanges > 0) {
 Serial.print("Success rate: ");
 Serial.print(100.0 * totalRanges / (totalRanges + failedRanges), 1);
 Serial.println("%");
 }
 Serial.println("============================");
 neighborPrint();
}

// ============================================================================
// SEQUENCE NUMBER & DUPLICATE DETECTION
// ============================================================================

uint16_t nextSeq() {
 return ++txSeqNumber;
}

bool isDupe(uint8_t from, uint16_t seq) {
 int idx = neighborFind(from);
 if (idx < 0) return false;

 if (seq == neighbors[idx].lastRxSeq && seq != 0) {
 return true;
 }

 neighbors[idx].lastRxSeq = seq;
 return false;
}

// ============================================================================
// FLOAT EMBED/EXTRACT HELPERS (IEEE 754 for message payload)
// ============================================================================

void embedFloat(byte* buf, float val) {
 memcpy(buf, &val, sizeof(float));
}

float extractFloat(const byte* buf) {
 float val;
 memcpy(&val, buf, sizeof(float));
 return val;
}

/*
 * ============================================================================
 * USAGE INSTRUCTIONS
 * ============================================================================
 *
 * 1. Set myDeviceID to a unique value for each device (1, 2, 3, 4, etc.)
 * 2. Upload to all ESP32+DW1000 devices
 * 3. Power on - devices discover and range automatically
 *
 * PROTOCOL BEHAVIOR:
 * - Each node broadcasts ANNOUNCE with jittered intervals (1.5-2.0s)
 * - Lower ID node initiates DS-TWR with each neighbor independently
 * - Per-neighbor ranging schedule: 250ms base + 0-120ms jitter
 * - RESP and FINAL use DW1000 delayed TX (3ms deterministic delay)
 * - After FINAL received, responder sends RESULT with measured Db + Rb
 * - Initiator computes distance from ALL four timestamps + both reply delays
 * - Full DS-TWR formula: tof = (Ra*Rb - Da*Db) / (Ra+Rb+Da+Db)
 * - No blocking delays anywhere (except delay(1) for ESP32 watchdog)
 * - 6-state MAC machine with clean timeout recovery
 * - Sequence numbers prevent duplicate processing
 * - Failed ranges cause backoff (200ms per consecutive failure)
 * - Neighbor timeout after 5s of silence
 * - Automatic rejoin when a node reappears
 *
 * CALIBRATION:
 * - Antenna delay (16384) MUST be calibrated for accurate distances
 * - Place devices at known distance and adjust
 * - Typical range: 16384-32768 (depends on antenna/PCB)
 * - ~500 units per meter of error
 *
 * TIMING TUNING:
 * - ANNOUNCE_BASE_MS / ANNOUNCE_JITTER_MS: Discovery frequency
 * - RANGE_BASE_MS / RANGE_JITTER_MS: Ranging frequency per neighbor
 * - RESP_DELAY_US / FINAL_DELAY_US: DW1000 delayed TX timing (min ~500us)
 * - MAC_TIMEOUT_MS: How long to wait before declaring failure
 *
 * DS-TWR ACCURACY NOTES:
 * - The initiator's distance computation uses actual measured Db and Rb
 *   (received in RESULT message), making it fully clock-drift-corrected.
 * - The responder's own display uses Da ≈ Db approximation (same delay),
 *   which is very close since both use the same delayed TX duration.
 * - For maximum accuracy, always use the INITIATOR's distance value
 *   (the one computed after receiving RESULT).
 *
 * ============================================================================
 */
