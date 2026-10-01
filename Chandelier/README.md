# Chandelier

Five independent WS2812B candles on a Seeed XIAO ESP32-C6.

| Candle | Board pin | ESP32-C6 GPIO | LEDs |
| --- | --- | --- | --- |
| 1 | D1 | 1 | 8 |
| 2 | D2 | 2 | 8 |
| 3 | D3 | 21 | 8 |
| 4 | D4 | 22 | 8 |
| 5 | D5 | 23 | 8 |

Each candle is a separate data chain: its board pin connects to the first unit's
DIN, and that unit's DOUT connects to the second unit's DIN. Do not connect the
data outputs of separate candles together. Connect all units to their rated
5 V supply in parallel and share ground with the ESP32-C6.

`NUM_LED_UNITS` and `LEDS_PER_UNIT` in `Chandelier.ino` describe **each output**.
The defaults are two units of four LEDs per candle: eight LEDs per pin, forty in
total. All eight LEDs within a candle share one colour and brightness.

Each candle has its own random noise seeds, flicker speed, average brightness,
dip/flare timers, and white-temperature cycle. Starting colour phases are spread
out and each warm-to-cool transition takes 24-36 seconds, so the candles do not
move together. The existing 1600-7000 K range and one-third brightness cap are
preserved. Animation parameters are in `src/CandleEffect.cpp`.

## Commands

Send these Arcanet commands to `CHANDELIER32` (the current `MY_ID`), `CHANDELIER`,
or `ALL`. They control all five candles together.

| Command | Effect |
| --- | --- |
| `CHANDELIER_ON` | Turn on using the selected colour mode. |
| `CHANDELIER_OFF` | Turn all LEDs off; keep listening for commands. |
| `CHANDELIER_COLOR_RED` | Select red light with independent candle flicker. |
| `CHANDELIER_COLOR_NORMAL` | Restore the existing warm-to-cool candle colours. |
| `CHANDELIER_COLOR_GREEN` | Select green light with independent candle flicker. |
| `SEND_UPDATE` | Request the current status. |

Colour commands preserve the on/off state. A colour selected while off appears
on the next `CHANDELIER_ON`. Changing colour or turning the lights off does not
reset the independent animation timers. After a restart, the default is on with
normal colours; settings are not saved to flash.

For example, another Arcanet device can send
`arcanet.sendCommand("CHANDELIER", "CHANDELIER_COLOR_RED");`.

Arcanet sends status updates to `CONTROLLER` periodically and after each recognised
command. Status reports `STATE_ON` or `STATE_OFF`. Unknown commands and messages
addressed to other devices are ignored. The bundled Arcanet copy supports the
sketch's `relayEnabled = false` setting: sending and receiving still work, while
received commands are not forwarded to other devices.

`PIN_BATTERY` currently names GPIO1. Battery ADC setup and readings are skipped
if that pin matches **any** candle output, and status reports `BLVL_0` when no
measurement is available. To use battery monitoring, connect the voltage divider
to a separate suitable ADC pin and update `PIN_BATTERY`.

## Output driver and build

Tested build configuration: Arduino-ESP32 3.3.8, FastLED 3.10.3, board
`esp32:esp32:XIAO_ESP32C6`. FastLED supplies colour scaling and noise functions.

The ESP32-C6 has two RMT transmit channels; FastLED 3.10.3's default RMT5 driver
reserves one channel per strip. `src/Ws2812Output.h` instead uses the ESP-IDF RMT5
API to share one transmitter across all five pins. It sends each complete GRB
frame and a 300 us reset pulse, waits for completion, then changes pins. Inactive
outputs are held low. Five eight-LED frames take about 2.7 ms of signal time,
plus driver overhead, within each animation update. No global library changes
are needed.

```powershell
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6 .
```

After uploading, check that all five pairs animate, their flicker and colour
cycles differ, and they continue running through the periodic network updates.
Try all five commands, including selecting a colour while off and turning back on.
Compilation and host tests cannot verify physical wiring or waveform timing.

Host checks (Windows with Visual Studio C++ tools): `tests\run-host-tests.cmd`.
These compile the actual effect and output code against small Arduino/FastLED
and RMT substitutes. They check independent animation state, brightness bounds,
timer rollover, command addressing, on/off and colour changes, all five outputs,
GRB bit order, reset duration, inactive-pin
isolation, and completion before reusing the shared transmitter.


