#include "HardwareSerial.h"
#include "Arcanet.h"
#include "esp_wifi.h"

Arcanet* Arcanet::_instance = nullptr;

Arcanet::Arcanet(String id, message_callback_t callback) {
  _id = id;
  _callback = callback;
  _peerCount = 0;
  _lastBroadcastTime = 0;
  _msgCount = 0;
  _dedupeHead = 0;
  _instance = this;
}

void Arcanet::init() {
  WiFi.mode(WIFI_STA);
  WiFi.macAddress(_myMac);
  Serial.printf("My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", _myMac[0], _myMac[1], _myMac[2], _myMac[3], _myMac[4], _myMac[5]);

  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  dedupeInit();
  Serial.println("Arcanet awake");
}

void Arcanet::loop() {
  if (millis() - _lastBroadcastTime > 10000) {
    _lastBroadcastTime = millis();
    broadcastDiscovery();
  }
}

void Arcanet::sendCommand(const String& id, const String& command) {
Serial.println("sendingCommand to id: "+id+"; command: "+command);
Serial.printf("sendingCommand A: %02X:%02X:%02X:%02X:%02X:%02X\n", _myMac[0], _myMac[1], _myMac[2], _myMac[3], _myMac[4], _myMac[5]);

  struct_message msg;
  msg.type = 'C';
  id.toCharArray(msg.id, sizeof(msg.id));
  command.toCharArray(msg.command, sizeof(msg.command));
  memcpy(msg.originMac, _myMac, 6);
  msg.msgID = _msgCount++;
  msg.hopCount = 0;

  for (int i = 0; i < _peerCount; i++) {
    esp_now_send(_knownPeers[i], (uint8_t *) &msg, sizeof(msg));
  }
}

void Arcanet::addPeer(const uint8_t* mac) {
  if (_peerCount < 40 && !isKnownPeer(mac)) {
Serial.printf("Added peer: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      memcpy(_knownPeers[_peerCount], mac, 6);
      _peerCount++;
    }
  }
}

bool Arcanet::isKnownPeer(const uint8_t* mac) {
  for (int i = 0; i < _peerCount; i++) {
    if (memcmp(_knownPeers[i], mac, 6) == 0) {
      return true;
    }
  }
  return false;
}

void Arcanet::broadcastDiscovery() {
  struct_message msg;
  msg.type = 'D';
  memcpy(msg.mac, _myMac, 6);
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
  if (!esp_now_is_peer_exist(broadcastAddress)) {
      esp_now_add_peer(&peerInfo);
  }

  esp_now_send(broadcastAddress, (const uint8_t *)&msg, sizeof(msg));
}

void Arcanet::broadcast(const struct_message &message) {
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcastAddress, (const uint8_t *)&message, sizeof(message));
}

void Arcanet::onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // You can add logic here to handle send status
}

void Arcanet::onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_message)) {
    struct_message msg;
    memcpy(&msg, incomingData, sizeof(msg));

    if (msg.type == 'D') {
      _instance->addPeer(msg.mac);
    } else if (msg.type == 'C') {
      if (_instance->isDuplicateAndRemember(msg.originMac, msg.msgID)) {
        Serial.print("Duplicate message");
        return; // Duplicate message
      }

//Serial.print("Received command "+String(msg.command)+", for id: "+String(msg.id)+", ");
Serial.printf("sender mac: %02X:%02X:%02X:%02X:%02X:%02X ", msg.mac[0], msg.mac[1], msg.mac[2], msg.mac[3], msg.mac[4], msg.mac[5]);
Serial.printf("origin mac: %02X:%02X:%02X:%02X:%02X:%02X\n", msg.originMac[0], msg.originMac[1], msg.originMac[2], msg.originMac[3], msg.originMac[4], msg.originMac[5]);

      if (_instance->_id.equals(msg.id)) {
        if (_instance->_callback) {
          _instance->_callback(msg.id, msg.command);
        }
      }


      if (msg.hopCount < 40 - 1) {
        //repeat command over the network
        msg.hopCount++;
        for (int i = 0; i < _instance->_peerCount; i++) {
            esp_now_send(_instance->_knownPeers[i], (uint8_t *) &msg, sizeof(msg));
Serial.printf("resending msg with mac: %02X:%02X:%02X:%02X:%02X:%02X ", msg.mac[0], msg.mac[1], msg.mac[2], msg.mac[3], msg.mac[4], msg.mac[5]);
        }
      }
    }
  }
}

void Arcanet::dedupeInit() {
  for (int i = 0; i < 64; ++i) {
    memset(_dedupeBuf[i].originMac, 0, 6);
    _dedupeBuf[i].msgID = -1;
  }
}

bool Arcanet::isDuplicateAndRemember(const uint8_t* origin, int msgID) {
  for (int i = 0; i < 64; ++i) {
    if (_dedupeBuf[i].msgID == msgID && sameMac(_dedupeBuf[i].originMac, origin)) {
      return true;
    }
  }
  memcpy(_dedupeBuf[_dedupeHead].originMac, origin, 6);
  _dedupeBuf[_dedupeHead].msgID = msgID;
  _dedupeHead = (_dedupeHead + 1) % 64;
  return false;
}

bool Arcanet::sameMac(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

void Arcanet::formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength) {
    snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}
