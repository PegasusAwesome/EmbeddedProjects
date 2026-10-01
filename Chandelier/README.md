# Chandelier

ESP32-C6 candle effect for WS2812B units connected in a data daisy chain.

The WS2812 candle effect runs continuously, with a gradual warm-to-cool white
cycle and candle flicker. Arcanet sends status updates to `CONTROLLER` and
responds to `SEND_UPDATE` addressed to `MY_ID`, `CHANDELIER`, or `ALL`.
Status reports `STATE_ON` while this continuous effect is running.

In `Chandelier.ino`, set `NUM_LED_UNITS` to the number of connected units and
`LEDS_PER_UNIT` to the number of individual LEDs on each unit. The current
configuration is two units with four LEDs each: eight LEDs total. For five
four-LED units, set `NUM_LED_UNITS` to `5`.

Connect GPIO1 (`DATA_PIN`) to the first unit's DIN, then its DOUT to the second
unit's DIN. Connect each unit to its rated power supply and share ground with
the ESP32-C6. Only the data signal is daisy-chained; power is connected in parallel.

`PIN_BATTERY` currently also names GPIO1. Battery ADC setup and readings are
skipped while it matches `DATA_PIN`, and status messages report `BLVL_0` to
indicate that no battery measurement is available. To use battery monitoring,
connect the voltage divider to a separate suitable ADC pin and update
`PIN_BATTERY`. Do not connect the battery divider to the LED data wire.

After uploading, check that all configured LEDs on both units light and continue
animating for at least 20 seconds (past the first scheduled battery/status update).
If the first unit works but the second remains dark, verify the per-unit LED count
and the first unit's DOUT-to-second-unit's-DIN connection.


