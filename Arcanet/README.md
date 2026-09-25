# Arcanet

ESP32 ESP-NOW mini mesh with discovery, deduplication, hop-limited relaying, and a tiny command interpreter

## Features
- Discovery broadcast every Ns
- Peer auto-add on discovery
- Message deduplication (ring buffer)
- Optional hop-limited relaying (enabled by default)
- Serial CLI: `<id>_<command>` (e.g., `LANTERN12_ON`)
- Commands (example): `LANTERN_ON/OFF`, `WHITE|RED|GREEN|BLUE_ON/OFF`, `ON/OFF` (builtin LED)

## Install
Copy this folder into `Arduino/examples/Arcanet`, restart Arduino IDE, and open Arcanet.ino.

## Sending, receiving, and relaying

Both constructors accept an optional third argument, `bool relayEnabled = true`:

```cpp
Arcanet(String id, message_callback_t callback, bool relayEnabled = true);
Arcanet(String id, legacy_string_message_callback_t callback, bool relayEnabled = true);
```

Choose one of these when constructing your node:

```cpp
// Send, receive, and retransmit: existing behavior.
Arcanet arcanet("LANTERN17", onCommandReceived);
// Or explicitly enable retransmission:
Arcanet arcanet("LANTERN17", onCommandReceived, true);
// Send and receive without retransmitting received commands:
Arcanet arcanet("LANTERN17", onCommandReceived, false);
```

This works with both `const char*` callbacks and legacy `const String&` callbacks.
With relaying disabled, discovery, peer tracking, deduplication, receive callbacks,
and `sendCommand()` still work. Other relay-enabled nodes can forward this node's
outgoing commands, but this node cannot serve as an intermediate hop.
Keep calling `arcanet.loop()` frequently in either mode.

## Notes
- Adjust pins to your hardware.
- Builtin LED is often active-low!
- Ensure all nodes run the same channel/frequency/resolution.
