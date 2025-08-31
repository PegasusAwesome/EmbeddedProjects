#ifndef Arcanet_h
#define Arcanet_h

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Callback function type for handling received messages
typedef void (*message_callback_t)(const String& id, const String& command);

// Default compile-time configuration values
// You can override these by defining them before including this header
#ifndef ARCANET_MAX_PEERS
#define ARCANET_MAX_PEERS 40
#endif

#ifndef ARCANET_DEDUPE_SIZE
#define ARCANET_DEDUPE_SIZE 64
#endif

#ifndef ARCANET_DISCOVERY_INTERVAL_MS
#define ARCANET_DISCOVERY_INTERVAL_MS 10000UL
#endif

#ifndef ARCANET_MAX_HOPS
#define ARCANET_MAX_HOPS 10
#endif

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

  // Configuration
  void setChannel(uint8_t channel); // 0 = current channel (default), 1..14 = fixed channel

  // Register a specific handler for an exact command string (optional QoL)
  bool registerCommand(const String& command, message_callback_t cb);

private:
  // Message structure (packed to minimize airtime and avoid padding issues)
  struct __attribute__((packed)) struct_message {
    char type;              // 'D' = discovery, 'C' = command
    char id[32];            // target id (for commands)
    char originId[32];      // originator id
    char command[256];      // command payload
    uint8_t originMac[6];   // originator MAC
    uint8_t mac[6];         // last-hop MAC
    uint64_t msgUID;        // 64-bit unique id
    int32_t hopCount;       // hop counter
  };

  // Peer management
  void addPeer(const uint8_t* mac, const String& id);
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
    uint64_t msgUID;
  };
  void dedupeInit();
  bool isDuplicateAndRemember(const uint8_t* origin, uint64_t msgID);
  static bool sameMac(const uint8_t* a, const uint8_t* b);
  static void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength);
  static uint64_t rand64();
  static bool isBroadcastMac(const uint8_t* mac);

  // Member variables
  String _id;
  uint8_t _myMac[6];
  message_callback_t _callback;
  uint8_t _knownPeers[ARCANET_MAX_PEERS][6];
  int _peerCount;
  unsigned long _lastBroadcastTime;
  DedupeEntry _dedupeBuf[ARCANET_DEDUPE_SIZE];
  int _dedupeHead;
  int8_t _bestRssi; // max RSSI seen
  int8_t _lastRssi; // last RSSI seen
  uint8_t _channel; // 0 = current, else fixed channel

  // Optional per-command handlers (QoL). If none match, fall back to global _callback.
  struct CommandHandler { String command; message_callback_t cb; };
  static const int MAX_HANDLERS = 16;
  CommandHandler _handlers[MAX_HANDLERS];
  int _handlerCount = 0;

  // Singleton instance
  static Arcanet* _instance;
};

#endif
