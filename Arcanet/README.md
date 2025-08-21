# Arcanet

ESP32 ESP-NOW mini mesh with discovery, deduplication, hop-limited relaying, and a tiny command interpreter

## Features
- Discovery broadcast every Ns
- Peer auto-add on discovery
- Message deduplication (ring buffer)
- Hop-limited relaying
- Serial CLI: `<id>_<command>` (e.g., `12_LANTERN_ON`)
- Commands (example): `LANTERN_ON/OFF`, `WHITE|RED|GREEN|BLUE_ON/OFF`, `ON/OFF` (builtin LED)

## Install
Copy this folder into `Arduino/libraries/EspNowMeshLamp`, restart Arduino IDE, and open *File -> Examples -> EspNowMeshLamp -> BasicMeshDemo*.

## Notes
- Adjust pins to your hardware.
- Builtin LED is often active-low!
- Ensure all nodes run the same channel/frequency/resolution.