// main.cpp — ESP32 Treadmill BLE Bridge BUILD 6
// Changes from BUILD 5:
//   [FIX] Client leak: deleteClient() before creating new client (crash after ~3 reconnects)
//   [FIX] Dangling pointer: store NimBLEAddress instead of NimBLEAdvertisedDevice*
//   [FIX] volatile on shared flags (doConnect, treadmillConnected, watchConnected)
//   [FIX] Connect timeout: 5 seconds (prevents infinite hang)
//   [FIX] Treadmill MAC extracted to constant
//   [FIX] readU8/readU16/readS16/readU24 — always advance idx even on bounds error
//   [FIX] platformio.ini comment was wrong ("RSC" → "FTMS proxy")
//   [CLEANUP] BUILD number bumped, log prefix [B6]

#include <Arduino.h>
#include <NimBLEDevice.h>

#define ESP32_BUILD 6

// ── Treadmill identity ──────────────────────────────────────────────────────
// EXILE TREX SPORT TX-400WP — random static BLE address
static const char* TREADMILL_MAC = "e3:97:52:37:c6:30";

// ── BLE UUIDs ────────────────────────────────────────────────────────────────
static const char* FTMS_SERVICE_UUID        = "00001826-0000-1000-8000-00805f9b34fb";
static const char* TREADMILL_DATA_CHAR_UUID = "00002acd-0000-1000-8000-00805f9b34fb";
static const char* CCCD_UUID                = "00002902-0000-1000-8000-00805f9b34fb";

// ── Global state ─────────────────────────────────────────────────────────────
// These flags are set in NimBLE callbacks (separate FreeRTOS task)
// and read in loop() (main task) — volatile prevents compiler optimizations
static NimBLEClient*         pClient     = nullptr;
static NimBLECharacteristic* p2acdChar   = nullptr;  // server-side char for watch notifications
static NimBLEAddress         treadmillAddress;        // copied from scan result (safe after rescan)
static volatile bool         doConnect          = false;
static volatile bool         treadmillConnected = false;
static volatile bool         watchConnected     = false;

// ── Treadmill data (written in FTMS callback, read in sendToWatch) ───────────
static uint16_t gRawSpeed   = 0;   // 0.01 km/h resolution
static uint32_t gDistanceM  = 0;   // meters
static uint16_t gElapsedSec = 0;   // seconds

// ── Forward declarations ─────────────────────────────────────────────────────
void sendToWatch();
bool connectToTreadmill();

// =============================================================================
// Watch notification — 7-byte custom packet over FTMS proxy
// =============================================================================
void sendToWatch() {
    if (!watchConnected || p2acdChar == nullptr) return;

    // 7-byte packet: speed(2) + distance(3) + elapsed(2), all little-endian
    uint8_t pkt[7];
    pkt[0] = (uint8_t)(gRawSpeed & 0xFF);
    pkt[1] = (uint8_t)((gRawSpeed >> 8) & 0xFF);
    pkt[2] = (uint8_t)(gDistanceM & 0xFF);
    pkt[3] = (uint8_t)((gDistanceM >> 8) & 0xFF);
    pkt[4] = (uint8_t)((gDistanceM >> 16) & 0xFF);
    pkt[5] = (uint8_t)(gElapsedSec & 0xFF);
    pkt[6] = (uint8_t)((gElapsedSec >> 8) & 0xFF);

    p2acdChar->setValue(pkt, sizeof(pkt));
    p2acdChar->notify();
}

// =============================================================================
// FTMS 0x2ACD parser — extracts speed, distance, elapsed time from treadmill
// =============================================================================
// Packet format: flags(2 bytes) + variable fields based on flag bits.
// See PROJECT.md "FTMS 0x2ACD Packet Format" for full field table.
bool parseTreadmillData(uint8_t* p, size_t len) {
    if (len < 2) return false;

    uint16_t flags = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    size_t idx = 2;

    // Helper lambdas for little-endian reads with bounds safety.
    // On out-of-bounds: return 0 but ALWAYS advance idx so subsequent
    // fields parse at correct offsets (even if garbage).
    auto readU8  = [&]() -> uint8_t  {
        if (idx >= len) { idx++; return 0; }
        return p[idx++];
    };
    auto readU16 = [&]() -> uint16_t {
        if (idx + 1 >= len) { idx += 2; return 0; }
        uint16_t v = (uint16_t)p[idx] | ((uint16_t)p[idx+1] << 8);
        idx += 2;
        return v;
    };
    auto readS16 = [&]() -> int16_t  { return (int16_t)readU16(); };
    auto readU24 = [&]() -> uint32_t {
        if (idx + 2 >= len) { idx += 3; return 0; }
        uint32_t v = (uint32_t)p[idx] | ((uint32_t)p[idx+1]<<8) | ((uint32_t)p[idx+2]<<16);
        idx += 3;
        return v;
    };

    // Bit 0 = 0: Instantaneous Speed present (mandatory when bit is 0)
    if (!(flags & 0x0001)) { gRawSpeed   = readU16(); }
    // Bit 1: Average Speed
    if (flags & 0x0002) readU16();
    // Bit 2: Total Distance (uint24)
    if (flags & 0x0004) { gDistanceM  = readU24(); }
    // Bit 3: Inclination + Ramp Angle
    if (flags & 0x0008) { readS16(); readS16(); }
    // Bit 4: Elevation Gain (positive + negative)
    if (flags & 0x0010) { readU16(); readU16(); }
    // Bit 5: Instantaneous Pace
    if (flags & 0x0020) readU8();
    // Bit 6: Average Pace
    if (flags & 0x0040) readU8();
    // Bit 7: Expended Energy (total + per hour + per minute)
    if (flags & 0x0080) { readU16(); readU16(); readU8(); }
    // Bit 8: Heart Rate
    if (flags & 0x0100) readU8();
    // Bit 9: Metabolic Equivalent
    if (flags & 0x0200) readU8();
    // Bit 10: Elapsed Time
    if (flags & 0x0400) { gElapsedSec = readU16(); }

    return true;
}

// =============================================================================
// FTMS notification callback — called by NimBLE when treadmill sends data
// =============================================================================
void onTreadmillData(NimBLERemoteCharacteristic* pChar,
                     uint8_t* pData, size_t length, bool isNotify)
{
    if (!parseTreadmillData(pData, length)) return;

    Serial.printf("[B%d][FTMS] spd=%.2f dst=%u t=%u watch=%s\n",
        ESP32_BUILD,
        gRawSpeed * 0.01f,
        gDistanceM,
        gElapsedSec,
        watchConnected ? "OK" : "waiting");

    sendToWatch();
}

// =============================================================================
// GATT Server callbacks — watch (CIQ DataField) connects/disconnects
// =============================================================================
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server) override {
        watchConnected = true;
        Serial.printf("[B%d][GATT] *** Watch connected! ***\n", ESP32_BUILD);
        // Restart advertising so other devices can still discover ESP32
        NimBLEDevice::getAdvertising()->start();
    }
    void onDisconnect(NimBLEServer* server) override {
        watchConnected = false;
        Serial.printf("[B%d][GATT] Watch disconnected.\n", ESP32_BUILD);
        NimBLEDevice::getAdvertising()->start();
    }
};

// =============================================================================
// BLE Client callbacks — ESP32 ↔ treadmill connection state
// =============================================================================
class TreadmillClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* client) override {
        Serial.printf("[B%d][FTMS] Connected to treadmill.\n", ESP32_BUILD);
        treadmillConnected = true;
    }
    void onDisconnect(NimBLEClient* client) override {
        Serial.printf("[B%d][FTMS] Disconnected from treadmill. Restarting scan...\n", ESP32_BUILD);
        treadmillConnected = false;

        // Reset data so watch shows zeros (= treadmill offline)
        gRawSpeed = 0; gDistanceM = 0; gElapsedSec = 0;
        sendToWatch();

        // Restart scanning to find treadmill again
        NimBLEDevice::getScan()->start(0, nullptr, false);
    }
};

// =============================================================================
// BLE Scan callback — filters by treadmill MAC address
// =============================================================================
class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* device) override {
        if (device->getAddress().toString() != TREADMILL_MAC) return;

        Serial.printf("[B%d][SCAN] Treadmill found by MAC — stopping scan.\n", ESP32_BUILD);
        NimBLEDevice::getScan()->stop();

        // Store ADDRESS, not the device pointer.
        // The pointer becomes invalid after scan buffer is reused on next scan.
        treadmillAddress = device->getAddress();
        doConnect = true;
    }
};

// =============================================================================
// Connect to treadmill — GATT discovery + FTMS subscription
// =============================================================================
bool connectToTreadmill() {
    // Clean up previous client to avoid NimBLE client pool exhaustion.
    // NimBLE has a pool of ~3 clients — without cleanup, after 3 reconnects
    // createClient() fails and ESP32 hangs.
    if (pClient != nullptr) {
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
    }

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new TreadmillClientCallbacks(), true);
    pClient->setConnectTimeout(5);  // 5 seconds — prevents infinite hang

    // Connect using stored address (safe across rescans)
    if (!pClient->connect(treadmillAddress)) {
        Serial.printf("[B%d][FTMS] ERROR: connect() failed.\n", ESP32_BUILD);
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
        return false;
    }

    Serial.printf("[B%d][FTMS] Connected. MTU=%d\n", ESP32_BUILD, pClient->getMTU());

    // Wait for BLE connection parameters to settle.
    // Without this delay, service discovery sometimes fails silently.
    delay(1000);

    // ── Discover FTMS service and Treadmill Data characteristic ──────────
    NimBLERemoteService* pSvc = pClient->getService(FTMS_SERVICE_UUID);
    if (!pSvc) {
        Serial.printf("[B%d][FTMS] ERROR: FTMS service not found.\n", ESP32_BUILD);
        pClient->disconnect();
        return false;
    }

    NimBLERemoteCharacteristic* pChar = pSvc->getCharacteristic(TREADMILL_DATA_CHAR_UUID);
    if (!pChar || !pChar->canNotify()) {
        Serial.printf("[B%d][FTMS] ERROR: 0x2ACD not found or not notifiable.\n", ESP32_BUILD);
        pClient->disconnect();
        return false;
    }

    // Subscribe via NimBLE API (registers local callback)
    pChar->subscribe(true, onTreadmillData);

    // CRITICAL: NimBLE 1.4.x subscribe() returns true but the treadmill
    // ignores it silently. Must manually write CCCD descriptor (0x2902)
    // with [0x01, 0x00] to actually enable notifications on the treadmill.
    NimBLERemoteDescriptor* pCCCD = pChar->getDescriptor(NimBLEUUID(CCCD_UUID));
    if (pCCCD) {
        uint8_t v[] = {0x01, 0x00};
        pCCCD->writeValue(v, 2, true);
        Serial.printf("[B%d][FTMS] CCCD written. Notifications enabled.\n", ESP32_BUILD);
    } else {
        Serial.printf("[B%d][FTMS] WARNING: CCCD descriptor not found!\n", ESP32_BUILD);
    }

    Serial.printf("[B%d][FTMS] Subscribed to 0x2ACD. Data flowing.\n", ESP32_BUILD);
    return true;
}

// =============================================================================
// GATT Server setup — FTMS proxy for CIQ DataField on the watch
// =============================================================================
void setupFtmsProxyServer() {
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pSvc = pServer->createService(FTMS_SERVICE_UUID);
    p2acdChar = pSvc->createCharacteristic(
        TREADMILL_DATA_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );

    // Initial value: 7 bytes of zeros (no data yet)
    uint8_t zero[7] = {0};
    p2acdChar->setValue(zero, 7);
    pSvc->start();

    // Advertise FTMS UUID so CIQ scan finds us via getServiceUuids()
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(NimBLEUUID(FTMS_SERVICE_UUID));

    // "TreadBLE" = 8 chars — fits in BLE scan response without truncation.
    // Longer names (e.g. "TreadmillBridge") get truncated to ~8 chars by the stack.
    NimBLEAdvertisementData scanResp;
    scanResp.setName("TreadBLE");
    pAdv->setScanResponseData(scanResp);

    pAdv->start();

    Serial.printf("[B%d][GATT] FTMS proxy started. Address: %s\n",
        ESP32_BUILD, NimBLEDevice::getAddress().toString().c_str());
}

// =============================================================================
// setup() — runs once at boot
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    // ── BUILD BANNER — always the first lines in the log ─────────────────
    Serial.println("================================================");
    Serial.printf( "=== Treadmill BLE Bridge ESP32 BUILD %d      ===\n", ESP32_BUILD);
    Serial.println("================================================");

    NimBLEDevice::init("TreadBLE");

    // Start GATT server first — watch can connect anytime
    setupFtmsProxyServer();

    // Start scanning for the treadmill by MAC address
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallbacks(), false);
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);

    Serial.printf("[B%d][SCAN] Scanning for treadmill MAC %s ...\n",
        ESP32_BUILD, TREADMILL_MAC);
    pScan->start(0, nullptr, false);
}

// =============================================================================
// loop() — main loop, handles deferred connection
// =============================================================================
void loop() {
    // doConnect is set by ScanCallbacks::onResult() in the NimBLE task.
    // We handle the actual connection here in the main task to avoid
    // blocking the NimBLE stack during GATT discovery.
    if (doConnect) {
        doConnect = false;
        if (!connectToTreadmill()) {
            Serial.printf("[B%d][SCAN] Reconnect failed. Restarting scan...\n", ESP32_BUILD);
            NimBLEDevice::getScan()->start(0, nullptr, false);
        }
    }
    delay(10);
}
