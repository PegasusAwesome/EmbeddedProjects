#ifndef Arcanet_h
#define Arcanet_h

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Callback function type for handling received messages
typedef void (*message_callback_t)(const String& id, const String& command);

class Arcanet {
public:
  // Constructor
  Arcanet(String id, message_callback_t callback);

  // Initialize the network
  void init();

  // Main loop to be called in the sketch's loop()
  void loop();

  // Send a command to a specific ID
  void sendCommand(const String& id, const String& command);

private:
  // Message structure
  struct struct_message {
    char type;
    char id[32];
    char command[32];
    uint8_t originMac[6];
    uint8_t mac[6];
    int msgID;
    int hopCount;
  };

  // Peer management
  void addPeer(const uint8_t* mac);
  bool isKnownPeer(const uint8_t* mac);

  // Broadcasting
  void broadcastDiscovery();
  void broadcast(const struct_message &message);

  // ESP-NOW callbacks
  static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
  static void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len);

  // Deduplication
  struct DedupeEntry {
    uint8_t originMac[6];
    int msgID;
  };
  void dedupeInit();
  bool isDuplicateAndRemember(const uint8_t* origin, int msgID);
  static bool sameMac(const uint8_t* a, const uint8_t* b);

  // Member variables
  String _id;
  uint8_t _myMac[6];
  message_callback_t _callback;
  uint8_t _knownPeers[40][6];
  int _peerCount;
  unsigned long _lastBroadcastTime;
  int _msgCount;
  DedupeEntry _dedupeBuf[64];
  int _dedupeHead;

  // Singleton instance
  static Arcanet* _instance;
};

#endif
