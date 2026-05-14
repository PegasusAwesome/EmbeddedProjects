#include <stdint.h>
#include "Arcanet.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_bt.h"

#ifndef ARCANET_DEBUG
#define ARCANET_DEBUG 0
#endif

#if ARCANET_DEBUG
#define ARC_LOGF(...) Serial.printf(__VA_ARGS__)
#define ARC_LOG(x) Serial.println(x)
#else
#define ARC_LOGF(...)
#define ARC_LOG(x)
#endif

// Simple critical section for multi-task access
static portMUX_TYPE s_sqMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_rqMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_dedupeMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_xqMux = portMUX_INITIALIZER_UNLOCKED;

static bool timeReached(unsigned long now, unsigned long scheduledAt) {
    return (long)(now - scheduledAt) >= 0;
}

static bool elapsedAtLeast(unsigned long now, unsigned long startedAt, unsigned long intervalMs) {
    return (unsigned long)(now - startedAt) >= intervalMs;
}


Arcanet* Arcanet::_instance = nullptr;


Arcanet::Arcanet(String id, message_callback_t callback) {
    _id = id;
    _callback = callback;
    _legacyStringCallback = nullptr;
    _peerCount = 0;
    _lastBroadcastTime = 0;
    _dedupeHead = 0;
    _instance = this;
    _bestRssi = -127; // track max RSSI
    _lastRssi = -127;
    _channel = 0;
    _sqHead = 0;
    _sqTail = 0;
    _sqCount = 0;
    _rqHead = 0;
    _rqTail = 0;
    _rqCount = 0;
    _xqHead = 0;
    _xqTail = 0;
    _xqCount = 0;
    _lastSendMs = 0;
    _rssiWindowSize = 20;
    _rssiCount = 0;
    _rssiHead = 0;

    for (uint8_t i = 0; i < RSSI_WINDOW_MAX_SIZE; ++i) _rssiWindow[i] = -127;
    for (int i = 0; i < ARCANET_MAX_PEERS; ++i) _peerLastSeenMs[i] = 0;
}

Arcanet::Arcanet(String id, legacy_string_message_callback_t callback) {
    _id = id;
    _callback = nullptr;
    _legacyStringCallback = callback;
    _peerCount = 0;
    _lastBroadcastTime = 0;
    _dedupeHead = 0;
    _instance = this;
    _bestRssi = -127; // track max RSSI
    _lastRssi = -127;
    _channel = 0;
    _sqHead = 0;
    _sqTail = 0;
    _sqCount = 0;
    _rqHead = 0;
    _rqTail = 0;
    _rqCount = 0;
    _xqHead = 0;
    _xqTail = 0;
    _xqCount = 0;
    _lastSendMs = 0;
    _rssiWindowSize = 20;
    _rssiCount = 0;
    _rssiHead = 0;

    for (uint8_t i = 0; i < RSSI_WINDOW_MAX_SIZE; ++i) _rssiWindow[i] = -127;
    for (int i = 0; i < ARCANET_MAX_PEERS; ++i) _peerLastSeenMs[i] = 0;
}

void Arcanet::init() {
    static_assert(sizeof(struct_message) <= 240, "ESPNOW payload too large; shrink struct_message fields");
    // Default to channel 1 only if the user has not configured a channel
    if (_channel == 0) {
        _channel = 1;
        ARC_LOG("We default to channel 1, if the user has not configured a channel.");
    }

    #if CONFIG_IDF_TARGET_ESP32S3
        ARC_LOG("Build-time: ESP32-S3 -> skipping external-antenna enable");
    #elif CONFIG_IDF_TARGET_ESP32C6
        ARC_LOG("Build-time: ESP32-C6 -> enabling external antenna");
        pinMode(WIFI_ENABLE, OUTPUT);
        digitalWrite(WIFI_ENABLE, LOW); // Activate RF switch control
        delay(200);
        pinMode(WIFI_ANT_CONFIG, OUTPUT);
        digitalWrite(WIFI_ANT_CONFIG, HIGH); // Use external antenna
        delay(500);
    #else
        #warning "Unknown ESP-IDF target! Check antenna-pin logic."
        ARC_LOG("Build-time: UNKNOWN chip -> please verify antenna logic");
    #endif

    (void)esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    WiFi.mode(WIFI_STA);
    delay(1000);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_LR);

    if (_channel != 0) {
        // Set a fixed channel for ESP-NOW if requested
        esp_wifi_set_channel(_channel, WIFI_SECOND_CHAN_NONE);
    }

    WiFi.macAddress(_myMac);
    ARC_LOGF("My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", _myMac[0], _myMac[1], _myMac[2], _myMac[3], _myMac[4], _myMac[5]);

    delay(100);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (esp_now_init() != ESP_OK) {
        ARC_LOG("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    _rssiCount = 0;
    _rssiHead = 0;
    for (int i = 0; i < _rssiWindowSize; ++i) _rssiWindow[i] = -127;
    _bestRssi = -127;
    _lastRssi = -127;

    dedupeInit();
    ARC_LOG("######################");
    ARC_LOG("### Arcanet awakes ###");
    ARC_LOG("######################");

    _lastBroadcastTime = millis()-ARCANET_DISCOVERY_INTERVAL_MS+5000;
}

void Arcanet::loop() {
    unsigned long now = millis();
    if (elapsedAtLeast(now, _lastBroadcastTime, ARCANET_DISCOVERY_INTERVAL_MS)) {
        _lastBroadcastTime = now;
        broadcastDiscovery();
    }
    processRxFrames();
    agePeers();
    processRecvQueue();
    processSendQueue();
}

void Arcanet::sendCommand(const String& id, const String& command) {
    ARC_LOGF("Sending command to id: %s; command: %s\n", id.c_str(), command.c_str());

    struct_message msg = {};
    msg.type = 'C';
    id.toCharArray(msg.id, sizeof(msg.id));
    _id.toCharArray(msg.originId, sizeof(msg.originId));
    command.toCharArray(msg.command, sizeof(msg.command));

    memcpy(msg.originMac, _myMac, 6);
    memcpy(msg.mac, _myMac, 6);

    msg.msgUID = rand64();
    msg.hopCount = 0;

    isDuplicateAndRemember(msg.originMac, msg.msgUID);
    
    for (int i = 0; i < _peerCount; i++) {
        if (sameMac(_instance->_knownPeers[i], _instance->_myMac)) continue;
        uint32_t jitter = esp_random() % 5; // 0–4 ms to de-sync relays
        _instance->enqueueSend(_instance->_knownPeers[i], msg, jitter);
    }
}


void Arcanet::broadcastDiscovery() {
    struct_message msg = {};
    msg.type = 'D';
    _id.toCharArray(msg.originId, sizeof(msg.originId));
    memcpy(msg.mac, _myMac, 6);

    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = (_channel == 0 ? 0 : _channel);
    peerInfo.encrypt = false;
    if (!esp_now_is_peer_exist(broadcastAddress)) {
        esp_err_t err = esp_now_add_peer(&peerInfo);
        if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
            ARC_LOGF("Failed to add broadcast peer (err=%d)\n", (int)err);
        }
    }

    esp_err_t sendErr = esp_now_send(broadcastAddress, (const uint8_t *)&msg, sizeof(msg));
    if (sendErr != ESP_OK) {
        ARC_LOGF("Discovery send error: %d\n", (int)sendErr);
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,5,0)
void Arcanet::onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    (void)tx_info;
    (void)status;
    // Could implement peer health tracking based on status
}
#else
void Arcanet::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    (void)mac_addr;
    (void)status;
    // Could implement peer health tracking based on status
}
#endif


#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
void Arcanet::onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    if (!_instance || !info || len != sizeof(struct_message)) {
        return;
    }

    struct_message msg;
    memcpy(&msg, incomingData, sizeof(msg));
    msg.id[sizeof(msg.id) - 1] = '\0';
    msg.originId[sizeof(msg.originId) - 1] = '\0';
    msg.command[sizeof(msg.command) - 1] = '\0';
    int8_t rssi = (info->rx_ctrl ? info->rx_ctrl->rssi : -127);
    _instance->enqueueRxFrame(msg, info->src_addr, rssi);
}
#else
void Arcanet::onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    if (!_instance || len != sizeof(struct_message)) {
        return;
    }

    struct_message msg;
    memcpy(&msg, incomingData, sizeof(msg));
    // Ensure C-strings are terminated to avoid out-of-bounds reads/logging from malformed frames
    msg.id[sizeof(msg.id) - 1] = '\0';
    msg.originId[sizeof(msg.originId) - 1] = '\0';
    msg.command[sizeof(msg.command) - 1] = '\0';
    _instance->enqueueRxFrame(msg, mac_addr, -127);
}
#endif


void Arcanet::dedupeInit() {
  for (int i = 0; i < ARCANET_DEDUPE_SIZE; ++i) {
    memset(_dedupeBuf[i].originMac, 0, 6);
    _dedupeBuf[i].msgUID = 0;
  }
}

bool Arcanet::isDuplicateAndRemember(const uint8_t* origin, uint64_t msgUID) {
  taskENTER_CRITICAL(&s_dedupeMux);

  for (int i = 0; i < ARCANET_DEDUPE_SIZE; ++i) {
    if (_dedupeBuf[i].msgUID == msgUID && sameMac(_dedupeBuf[i].originMac, origin)) {
      taskEXIT_CRITICAL(&s_dedupeMux);
      return true;
    }
  }

  memcpy(_dedupeBuf[_dedupeHead].originMac, origin, 6);
  _dedupeBuf[_dedupeHead].msgUID = msgUID;
  _dedupeHead = (_dedupeHead + 1) % ARCANET_DEDUPE_SIZE;

  taskEXIT_CRITICAL(&s_dedupeMux);
  return false;
}

bool Arcanet::sameMac(const uint8_t* a, const uint8_t* b) {
    return memcmp(a, b, 6) == 0;
}

uint64_t Arcanet::rand64() {
    uint64_t hi = (uint64_t) esp_random();
    uint64_t lo = (uint64_t) esp_random();
    return (hi << 32) | lo;
}

bool Arcanet::isBroadcastMac(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

int Arcanet::getBestRssi() const {
    return _bestRssi;
}

void Arcanet::rssiPush(int8_t rssi) {
    _lastRssi = rssi;
    _rssiWindow[_rssiHead] = rssi;
    _rssiHead = (_rssiHead + 1) % _rssiWindowSize;
    if (_rssiCount < _rssiWindowSize) _rssiCount++;

    int8_t best = -127;
    for (int i = 0; i < _rssiCount; ++i) {
        int8_t v = _rssiWindow[i];
        if (v > best) best = v;
    }
    _bestRssi = best;
}

void Arcanet::setRssiWindowSize(uint8_t size) {
    if (size < 1) size = 1;
    if (size > RSSI_WINDOW_MAX_SIZE) size = RSSI_WINDOW_MAX_SIZE;
    _rssiWindowSize = size;
    _rssiCount = 0;
    _rssiHead = 0;
    for (uint8_t i = 0; i < _rssiWindowSize; ++i) _rssiWindow[i] = -127;
    _bestRssi = -127;
    _lastRssi = -127;
}

uint8_t Arcanet::getRssiWindowSize() const {
    return _rssiWindowSize;
}


void Arcanet::setChannel(uint8_t channel) {
    _channel = channel; // apply on next init and for future peers
}


bool Arcanet::enqueueSend(const uint8_t* mac, const struct_message &msg, uint32_t jitterMs) {
  // Try to push one item; drop if full (caller may choose to try again later)
  taskENTER_CRITICAL(&s_sqMux);
  if (_sqCount >= ARCANET_SEND_QUEUE_SIZE) {
    taskEXIT_CRITICAL(&s_sqMux);
    return false;
  }

  SendQueueItem &slot = _sendQ[_sqHead];
  memcpy(slot.mac, mac, 6);
  slot.msg = msg; // struct copy
  slot.notBeforeMs = millis() + jitterMs;

  _sqHead = (uint16_t)((_sqHead + 1) % ARCANET_SEND_QUEUE_SIZE);
  _sqCount = _sqCount+1;
  taskEXIT_CRITICAL(&s_sqMux);
  return true;
}

void Arcanet::processSendQueue() {
  uint8_t sent = 0;
  unsigned long now = millis();

  while (sent < ARCANET_MAX_SENDS_PER_LOOP) {
    // Peek
    taskENTER_CRITICAL(&s_sqMux);
    if (_sqCount == 0) {
      taskEXIT_CRITICAL(&s_sqMux);
      break; // empty
    }
    SendQueueItem item = _sendQ[_sqTail]; // copy to local
    taskEXIT_CRITICAL(&s_sqMux);

    // Wait for scheduled time (keeps order simple)
    if (!timeReached(now, item.notBeforeMs)) {
      break;
    }

    // Rate limit a bit to let the WiFi task breathe
    if (_lastSendMs && !elapsedAtLeast(now, _lastSendMs, ARCANET_MIN_SEND_GAP_MS)) {
      break;
    }

    esp_err_t err = esp_now_send(item.mac, (const uint8_t*)&item.msg, sizeof(item.msg));
    if (err == ESP_ERR_ESPNOW_NO_MEM) {
      ARC_LOGF("esp_now_send NO_MEM: %s (%d)\n", esp_err_to_name(err), (int)err);
      break;// Leave item in queue; try again next loop tick
    }

    // Pop on success or non-NOMEM failure (optional: you can handle other errors differently)
    taskENTER_CRITICAL(&s_sqMux);
    _sqTail = (uint16_t)((_sqTail + 1) % ARCANET_SEND_QUEUE_SIZE);
    _sqCount = _sqCount-1;
    taskEXIT_CRITICAL(&s_sqMux);

    now = millis();
    _lastSendMs = now;
    sent++;

    if (sent < ARCANET_MAX_SENDS_PER_LOOP) {
        delay(2);
    }

  }
}

bool Arcanet::enqueueRecv(const char* id, const char* command) {
  taskENTER_CRITICAL(&s_rqMux);
  if (_rqCount >= ARCANET_RECV_QUEUE_SIZE) {
    taskEXIT_CRITICAL(&s_rqMux);
    return false;
  }
  RecvEvent &slot = _recvQ[_rqHead];
  strncpy(slot.id, id, sizeof(slot.id));
  slot.id[sizeof(slot.id)-1] = '\0';
  strncpy(slot.command, command, sizeof(slot.command));
  slot.command[sizeof(slot.command)-1] = '\0';
  _rqHead = (uint16_t)((_rqHead + 1) % ARCANET_RECV_QUEUE_SIZE);
  _rqCount = _rqCount+1;
  taskEXIT_CRITICAL(&s_rqMux);
  return true;
}

bool Arcanet::enqueueRxFrame(const struct_message& msg, const uint8_t* senderMac, int8_t rssi) {
  taskENTER_CRITICAL(&s_xqMux);
  if (_xqCount >= ARCANET_RX_FRAME_QUEUE_SIZE) {
    taskEXIT_CRITICAL(&s_xqMux);
    return false;
  }

  RxFrame &slot = _rxQ[_xqHead];
  slot.msg = msg; // struct copy
  memcpy(slot.senderMac, senderMac, 6);
  slot.rssi = rssi;

  _xqHead = (uint16_t)((_xqHead + 1) % ARCANET_RX_FRAME_QUEUE_SIZE);
  _xqCount = _xqCount + 1;
  taskEXIT_CRITICAL(&s_xqMux);
  return true;
}

void Arcanet::processRxFrames() {
  while (true) {
    taskENTER_CRITICAL(&s_xqMux);
    if (_xqCount == 0) {
      taskEXIT_CRITICAL(&s_xqMux);
      break;
    }

    RxFrame frame = _rxQ[_xqTail];
    _xqTail = (uint16_t)((_xqTail + 1) % ARCANET_RX_FRAME_QUEUE_SIZE);
    _xqCount = _xqCount - 1;
    taskEXIT_CRITICAL(&s_xqMux);

    if (frame.rssi > -127) {
      rssiPush(frame.rssi);
    }

    struct_message msg = frame.msg;
    const uint8_t* sender_mac = frame.senderMac;

    if (msg.type == 'D') {
      if (!sameMac(msg.mac, _myMac)) {
        addPeer(msg.mac, msg.originId);
      }
      continue;
    }

    if (msg.type != 'C') {
      continue;
    }

    touchPeer(sender_mac);

    const char* relayId = lookupPeerId(sender_mac);
    ARC_LOGF("Received command %s, from originId: %s, via relayId: %s, for id: %s\n", msg.command, msg.originId, (relayId ? relayId : "unknown"), msg.id);

    if (isDuplicateAndRemember(msg.originMac, msg.msgUID)) {
      continue;
    }

    if (_callback || _legacyStringCallback) {
      enqueueRecv(msg.id, msg.command);
    }

    // Sanitize hopCount so negative values don't bypass the hop limit logic
    if (msg.hopCount < 0) {
      msg.hopCount = 0;
    }
    if (msg.hopCount < ARCANET_MAX_HOPS) {
      memcpy(msg.mac, _myMac, 6);
      msg.hopCount++;

      for (int i = 0; i < _peerCount; i++) {
        if (sameMac(_knownPeers[i], sender_mac)) continue;
        if (sameMac(_knownPeers[i], _myMac)) continue;
        uint32_t jitter = esp_random() % 30; // 0-29 ms to de-sync relays
        enqueueSend(_knownPeers[i], msg, jitter);
      }
    }
  }
}

void Arcanet::processRecvQueue() {
  while (true) {
    taskENTER_CRITICAL(&s_rqMux);
    if (_rqCount == 0) {
      taskEXIT_CRITICAL(&s_rqMux);
      break;
    }
    RecvEvent ev = _recvQ[_rqTail];
    _rqTail = (uint16_t)((_rqTail + 1) % ARCANET_RECV_QUEUE_SIZE);
    _rqCount = _rqCount-1;
    taskEXIT_CRITICAL(&s_rqMux);
    if (_callback) {
      _callback(ev.id, ev.command);
    } else if (_legacyStringCallback) {
      String idStr(ev.id);
      String commandStr(ev.command);
      _legacyStringCallback(idStr, commandStr);
    }
  }
}


void Arcanet::addPeer(const uint8_t* mac, const char* originId) {
    if (sameMac(mac, _myMac)) {
        return;
    }

    int existingIndex = findPeerIndex(mac);
    if (existingIndex >= 0) {
        touchPeer(mac, originId);
        return;
    }

    if (_peerCount >= ARCANET_MAX_PEERS) {
        return;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = (_channel == 0 ? 0 : _channel);
    peerInfo.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peerInfo);
    if (err == ESP_OK) {
        memcpy(_knownPeers[_peerCount], mac, 6);

        // Store human-readable peer ID safely
        if (originId) {
            strncpy(_peerIds[_peerCount], originId, sizeof(_peerIds[_peerCount]));
            _peerIds[_peerCount][sizeof(_peerIds[_peerCount]) - 1] = '\0';
        } else {
            _peerIds[_peerCount][0] = '\0';
        }
        _peerLastSeenMs[_peerCount] = millis();

        _peerCount++;
        ARC_LOGF("Added peer: %s\n", originId ? originId : "<null>");
    } else if (err == ESP_ERR_ESPNOW_EXIST) {
        ARC_LOG("Peer already exists");
    } else {
        ARC_LOGF("Failed to add peer (err=%d)\n", (int)err);
    }
}

bool Arcanet::isKnownPeer(const uint8_t* mac) {
    return findPeerIndex(mac) >= 0;
}

int Arcanet::findPeerIndex(const uint8_t* mac) {
    for (int i = 0; i < _peerCount; i++) {
        if (memcmp(_knownPeers[i], mac, 6) == 0) {
            return i;
        }
    }
    return -1;
}

void Arcanet::touchPeer(const uint8_t* mac, const char* id) {
    int index = findPeerIndex(mac);
    if (index < 0) {
        return;
    }

    _peerLastSeenMs[index] = millis();
    if (id && id[0] != '\0') {
        strncpy(_peerIds[index], id, sizeof(_peerIds[index]));
        _peerIds[index][sizeof(_peerIds[index]) - 1] = '\0';
    }
}

void Arcanet::agePeers() {
    unsigned long now = millis();

    for (int i = _peerCount - 1; i >= 0; --i) {
        if (elapsedAtLeast(now, _peerLastSeenMs[i], ARCANET_PEER_TIMEOUT_MS)) {
            removePeerAt(i);
        }
    }
}

void Arcanet::removePeerAt(int index) {
    if (index < 0 || index >= _peerCount) {
        return;
    }
    esp_now_del_peer(_knownPeers[index]);

    for (int i = index; i < _peerCount - 1; ++i) {
        memcpy(_knownPeers[i], _knownPeers[i + 1], sizeof(_knownPeers[i]));
        memcpy(_peerIds[i], _peerIds[i + 1], sizeof(_peerIds[i]));
        _peerLastSeenMs[i] = _peerLastSeenMs[i + 1];
    }

    _peerCount--;
}


const char* Arcanet::lookupPeerId(const uint8_t* mac) {
    for (int i = 0; i < _peerCount; i++) {
        if (memcmp(_knownPeers[i], mac, 6) == 0) {
            return _peerIds[i];
        }
    }
    return nullptr;
}
