#ifndef Arcanet_h
#define Arcanet_h

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Callback function type for handling received messages
typedef void (*message_callback_t)(const char* id, const char* command);
typedef void (*legacy_string_message_callback_t)(const String& id, const String& command);

// Default compile-time configuration values
// You can override these by defining them before including this header
#ifndef ARCANET_MAX_PEERS
#define ARCANET_MAX_PEERS 32
#endif

#ifndef ARCANET_DEDUPE_SIZE
#define ARCANET_DEDUPE_SIZE 64
#endif

#ifndef ARCANET_DISCOVERY_INTERVAL_MS
#define ARCANET_DISCOVERY_INTERVAL_MS 60000UL
#endif

#ifndef ARCANET_MAX_HOPS
#define ARCANET_MAX_HOPS 39
#endif

#ifndef ARCANET_SEND_QUEUE_SIZE
#define ARCANET_SEND_QUEUE_SIZE 32
#endif

#ifndef ARCANET_MAX_SENDS_PER_LOOP
#define ARCANET_MAX_SENDS_PER_LOOP 2
#endif

#ifndef ARCANET_MIN_SEND_GAP_MS
#define ARCANET_MIN_SEND_GAP_MS 12
#endif

#ifndef ARCANET_RECV_QUEUE_SIZE
#define ARCANET_RECV_QUEUE_SIZE 32
#endif

#ifndef ARCANET_RX_FRAME_QUEUE_SIZE
#define ARCANET_RX_FRAME_QUEUE_SIZE 32
#endif





class Arcanet {
public:
  // Constructor
  Arcanet(String id, message_callback_t callback);
  Arcanet(String id, legacy_string_message_callback_t callback);

  // Initialize the network
  void init();

  // Main loop to be called in the sketch's loop()
  void loop();

  // Send a command to a specific ID
  void sendCommand(const String& id, const String& command);

  // Configuration
  void setChannel(uint8_t channel); // 0 = current channel (default), 1..14 = fixed channel

  // Best (maximum) RSSI observed recently, in dBm.
  // Use a signed integer return type to avoid unsigned wrap-around.
  static int getBestRssi();

  void processSendQueue();

  void setRssiWindowSize(uint8_t size);
  uint8_t getRssiWindowSize() const;

private:
  // Message structure (packed to minimize airtime and avoid padding issues) 134 bytes
  struct __attribute__((packed)) struct_message {
    char type;              // 'D' = discovery, 'C' = command
    char id[24];            // target id (for commands)
    char originId[24];      // originator id
    char command[64];       // command payload
    uint8_t originMac[6];   // originator MAC
    uint8_t mac[6];         // last-hop MAC
    uint64_t msgUID;        // 64-bit unique id
    int8_t hopCount;        // hop counter
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


  //***** Receive event queue *****
  struct RecvEvent {
    char id[24];
    char command[64];
  };

  struct RxFrame {
    struct_message msg;
    uint8_t senderMac[6];
    int8_t rssi;
  };

  volatile uint16_t _xqHead;
  volatile uint16_t _xqTail;
  volatile uint16_t _xqCount;
  RxFrame _rxQ[ARCANET_RX_FRAME_QUEUE_SIZE];

  bool enqueueRxFrame(const struct_message& msg, const uint8_t* senderMac, int8_t rssi);
  void processRxFrames();

  volatile uint16_t _rqHead;
  volatile uint16_t _rqTail;
  volatile uint16_t _rqCount;
  RecvEvent _recvQ[ARCANET_RECV_QUEUE_SIZE];

  bool enqueueRecv(const char* id, const char* command);
  void processRecvQueue();

  //***** Peer management *****
  void addPeer(const uint8_t* mac, const char* id);
  bool isKnownPeer(const uint8_t* mac);

  // Lookup helper to resolve a peer's human-readable id from its MAC address
  const char* lookupPeerId(const uint8_t* mac);


  void rssiPush(int8_t rssi);

  uint8_t _rssiWindowSize;

  // Broadcasting
  void broadcastDiscovery();

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

  static uint64_t rand64();
  static bool isBroadcastMac(const uint8_t* mac);

  // Member variables
  String _id;
  uint8_t _myMac[6];
  message_callback_t _callback;
  legacy_string_message_callback_t _legacyStringCallback;
  uint8_t _knownPeers[ARCANET_MAX_PEERS][6];
  char _peerIds[ARCANET_MAX_PEERS][24];
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
