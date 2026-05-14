# Arcanet Agent Notes

## Project Purpose

Arcanet is an Arduino ESP32 library for a small ESP-NOW mesh network used by devices during a LARP event. Nodes identify themselves with human-readable IDs such as `LANTERN17`, discover nearby peers, relay commands, and deduplicate messages so commands can spread through the network without looping forever.

The root `Arcanet.ino` sketch is an example of how application code should use the library in `src/`.

## Repository Layout

- `Arcanet.ino` - Example sketch showing device identity, command callback handling, serial command input, and the required service loop.
- `src/Arcanet.h` - Public API, compile-time configuration defaults, message structures, queues, peer tracking, and callback type declarations.
- `src/Arcanet.cpp` - ESP-NOW implementation: initialization, discovery, receive processing, send queue, relay logic, dedupe, RSSI tracking, and peer aging.
- `README.md` - Short user-facing feature and install notes.
- `library.properties` - Arduino library metadata.
- `keywords.txt` - Arduino IDE syntax highlighting keywords.

## Runtime Model

- Devices run in WiFi station mode and use ESP-NOW.
- Discovery messages are periodically broadcast.
- Discovered peers are added to ESP-NOW and aged out after a timeout.
- Commands are sent by target ID and relayed through known peers.
- Command frames include an origin MAC, message UID, and hop count.
- Deduplication uses `(originMac, msgUID)` in a ring buffer.
- Receive callbacks should not do ESP-NOW work directly; inbound frames are queued and processed from `Arcanet::loop()`.
- `Arcanet::loop()` must be called often. The example uses `serviceFor(ms)` instead of long `delay()` calls so serial input and network queues keep moving.

## Public API

Typical usage:

```cpp
#include "src/Arcanet.h"

void onCommandReceived(const String& id, const String& command) {
  // Application behavior here.
}

Arcanet arcanet("LANTERN17", onCommandReceived);

void setup() {
  Serial.begin(115200);
  arcanet.init();
}

void loop() {
  arcanet.loop();
}
```

Main methods:

- `Arcanet(String id, message_callback_t callback)` - Uses `const char*` callback arguments.
- `Arcanet(String id, legacy_string_message_callback_t callback)` - Uses `String` callback arguments; used by the example sketch.
- `init()` - Configures WiFi/ESP-NOW, registers callbacks, resets RSSI/dedupe state, and schedules initial discovery.
- `loop()` - Runs discovery, receive frame processing, peer aging, callback dispatch, and send queue processing.
- `sendCommand(const String& id, const String& command)` - Sends a command toward a target ID through known peers.
- `setChannel(uint8_t channel)` - Must be called before `init()` if a fixed ESP-NOW channel is needed. `0` means current channel, but `init()` currently defaults unset channel to `1`.
- `getBestRssi()`, `setRssiWindowSize()`, `getRssiWindowSize()` - RSSI helpers.
- `getTopPeersByRssi(PeerInfo* peers, uint8_t maxPeers)` - Fills a caller-provided array with known peers sorted by strongest last RSSI.

## Protocol Notes

The packed ESP-NOW payload is `struct_message` in `Arcanet.h` and must remain within ESP-NOW payload limits. It currently contains:

- `type`: `'D'` for discovery, `'C'` for command.
- `id[24]`: command target ID.
- `originId[24]`: human-readable origin node ID.
- `command[64]`: command payload.
- `originMac[6]`: original sender MAC for dedupe.
- `mac[6]`: last-hop MAC.
- `msgUID`: 64-bit unique message ID.
- `hopCount`: relay hop counter.

Keep ID and command size limits in mind when changing examples or docs.

## Example Sketch Behavior

`Arcanet.ino` defines:

- `TYPE`, `SERIAL_ID`, and `MY_ID` as the device identity.
- `onCommandReceived(id, command)` as the application callback.
- `readSerial()` as a tiny serial CLI. It expects input in the form `<id>_<command>`, for example `LANTERN17_ON`.
- If the serial command targets this node, the example handles it locally before forwarding it through Arcanet.
- `serviceFor(ms)` repeatedly calls `readSerial()`, `arcanet.loop()`, and `delay(1)`.
- `REQUESTPEERS` addressed to this node makes the example send a peer report to `CONTROLLER1`. Because command payloads are only 64 bytes, the response is split into numbered payloads such as `SENDPEERS:1/5:LANTERN12=-61`.

For application sketches, avoid long blocking delays. If a delay is needed, use the same service pattern so network queues do not stall.

## Configuration Defines

Defaults are in `src/Arcanet.h` and can be overridden before including the header:

- `ARCANET_MAX_PEERS`
- `ARCANET_DEDUPE_SIZE`
- `ARCANET_DISCOVERY_INTERVAL_MS`
- `ARCANET_MAX_HOPS`
- `ARCANET_SEND_QUEUE_SIZE`
- `ARCANET_MAX_SENDS_PER_LOOP`
- `ARCANET_MIN_SEND_GAP_MS`
- `ARCANET_RECV_QUEUE_SIZE`
- `ARCANET_RX_FRAME_QUEUE_SIZE`
- `ARCANET_PEER_TIMEOUT_MS`

Debug logging is controlled in `src/Arcanet.cpp` with `ARCANET_DEBUG`.

## Hardware and Framework Assumptions

- Target platform is ESP32-family Arduino with ESP-IDF headers available through the Arduino core.
- The code has conditional ESP-IDF callback signatures for IDF 5.x changes.
- ESP32-C6 builds enable external antenna pins using `WIFI_ENABLE` and `WIFI_ANT_CONFIG`.
- ESP32-S3 builds skip that antenna setup.
- Unknown targets emit a compile-time warning to verify antenna logic.
- Bluetooth controller memory is released during initialization.

## Coding Guidelines for Future Changes

- Preserve the non-blocking queue-driven design. ESP-NOW callbacks should stay short and avoid application callbacks directly.
- Be careful with fixed-size C strings in `struct_message`; always preserve null termination when copying or receiving data.
- Maintain wraparound-safe time comparisons using the existing helper style.
- Avoid increasing the ESP-NOW payload size without checking the `static_assert` in `init()`.
- Keep command relaying hop-limited and deduped.
- Do not remove peer aging unless replacing it with another bounded peer-management strategy.
- Prefer small, Arduino-compatible C++ changes. Avoid STL-heavy code unless the ESP32 Arduino environment clearly supports it and the added memory cost is justified.
- If editing examples, keep them practical for event devices: clear identity setup, simple callback behavior, and frequent `arcanet.loop()` calls.

## Verification Notes

There is no local build system in this folder beyond the Arduino library files. Best verification is compiling the sketch in Arduino IDE or Arduino CLI against the intended ESP32 board package. If Arduino CLI is available, compile the example for the actual target board after changes.
