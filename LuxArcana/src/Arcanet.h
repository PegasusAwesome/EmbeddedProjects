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
#define ARCANET_MAX_HOPS 39
#endif

#ifndef ARCANET_SEND_QUEUE_SIZE
#define ARCANET_SEND_QUEUE_SIZE 32
#endif

#ifndef ARCANET_MAX_SENDS_PER_LOOP
#define ARCANET_MAX_SENDS_PER_LOOP 6
#endif

#ifndef ARCANET_MIN_SEND_GAP_MS
#define ARCANET_MIN_SEND_GAP_MS 2
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

  // Best (maximum) RSSI observed recently, in dBm.
  // Use a signed integer return type to avoid unsigned wrap-around.
  static int getBestRssi();


private:
  // Message structure (packed to minimize airtime and avoid padding issues)
  struct __attribute__((packed)) struct_message {
    char type;              // 'D' = discovery, 'C' = command
    char id[32];            // target id (for commands)
    char originId[28];      // originator id
    char command[128];      // command payload
    uint8_t originMac[6];   // originator MAC
    uint8_t mac[6];         // last-hop MAC
    uint64_t msgUID;        // 64-bit unique id
    int32_t hopCount;       // hop counter
  };


  //***** Message queue management ***** 
  struct SendQueueItem {
    uint8_t mac[6];
    struct_message msg;
    uint32_t notBeforeMs; // optional jitter scheduling
  };

  // Queue state
  volatile uint16_t _sqHead;
  volatile uint16_t _sqTail;
  volatile uint16_t _sqCount;
  SendQueueItem _sendQ[ARCANET_SEND_QUEUE_SIZE];
  unsigned long _lastSendMs;

  // Queue API
  bool enqueueSend(const uint8_t* mac, const struct_message &msg, uint32_t jitterMs = 0);
  void processSendQueue();


  //***** Peer management *****
  void addPeer(const uint8_t* mac, const String& id);
  bool isKnownPeer(const uint8_t* mac);

  void rssiPush(int8_t rssi);

  // Broadcasting
  void broadcastDiscovery();
  void broadcast(const struct_message &message);


  // ESP-NOW callbacks
  // -------- feature detection (IDF version) ----------
  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,5,0)
  static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
  #else
  static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
  #endif

  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
  static void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len);
  #else
  static void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
  #endif


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
  int _bestRssi; // max RSSI seen
  int _lastRssi; // last RSSI seen
  uint8_t _channel; // 0 = current, else fixed channel

  // Singleton instance
  static Arcanet* _instance;
};

#endif
