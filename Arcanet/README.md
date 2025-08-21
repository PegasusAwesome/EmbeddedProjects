name=Arcanet
version=1.0.0
author=MWJ
maintainer=MWJ <marcel@marceloerlemans.com>
sentence=Do ESP Now meshing on ESP32 easily.
paragraph=Small helper library that wraps ESP Now meshing  with a simple API. Based on: https://protonestiot.medium.com/automating-node-integration-in-esp-now-mesh-networks-with-esp32-73bc9c0baa3f
category=Signal Input/Output
url=https://github.com/PegasusAwesome/EmbeddedProjects
architectures=*
depends=


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