# Leaf Remote Control Mesh

Firmware and tooling for a small ESP-NOW mesh built on ESP32-C6-DevKitC-1
boards. One master broadcasts parameters and multiple slaves apply them to
their PWM output while reporting liveness back to the master.

## Hardware target

- Board: ESP32-C6-DevKitC-1
- MCU: ESP32-C6 (RISC-V)
- Wireless: ESP-NOW (Wi-Fi 6 radio)
- Framework: ESP-IDF (PlatformIO)

## Wiring defaults (ESP32-C6-DevKitC-1)

- MASTER
  - POT_SPEED_GPIO: 0 (ADC1)
  - POT_PERIOD_GPIO: 1 (ADC1)
- SLAVE
  - PWM_GPIO: 5
- Onboard RGB LED: GPIO 8 (addressable)

Adjust pins in `include/config.h` per board.

## Behavior overview

- Master:
  - Broadcasts parameters every second and immediately on change.
  - Red LED blinks at 1s interval; fast white flash on TX/RX activity.
  - Logs peer liveness and radio stats every 5s.
- Slaves:
  - Apply speed to PWM output.
  - LED behavior:
    - White brightness mapped to effective speed (0..1 -> 0..255).
    - Double green flash on parameter update.
  - Persist last parameters in NVS and restore after reset.

## Parameters (mode behavior)

- `mode = 0` (continuous): runs at `speed` continuously (probability ignored).
- `mode = 1` (probabilistic): every 10 seconds, the slave decides ON/OFF using
  `probability`. When OFF, speed is 0 for that interval.

## Serial protocol (master)

Send a line of parameters over UART (USB serial) using key/value pairs:

- `speed=0.5,prob=0.2,mode=1`
- `speed=0.8 probability=1 mode=0`

Values for `speed` and `prob` are expected in the 0..1 range.

When a valid line is parsed, the master replies with:

- `ACK speed=... prob=... mode=...`

## Uploading master and slave firmware

This project uses a compile time role switch via `NODE_ROLE` in `platformio.ini`.

Option A: edit `include/config.h`
- Set `#define NODE_ROLE ROLE_MASTER`, then upload to the master board.
- Set `#define NODE_ROLE ROLE_SLAVE`, then upload to each slave board.

Option B: override from the build command
- Master:
  `pio run -e master -t upload --upload-port /dev/ttyACM0`
- Slave:
  `pio run -e slave -t upload --upload-port /dev/ttyACM1`

If multiple slaves are connected, you can flash all of them:

```
for p in /dev/ttyACM1 /dev/ttyACM2 /dev/ttyACM3 /dev/ttyACM4; do
  pio run -e slave -t upload --upload-port "$p"
done
```

## GUI utility

The GUI (Tkinter) lets you set parameters and view slave count/ack logs.

```
./gui/app.py
```

## Notes

- PWM is fixed direction only. Add external direction pin handling if needed.
- The mesh uses ESP-NOW broadcast plus ACK-based discovery.
- Peer liveness uses hysteresis to avoid false disconnects.
