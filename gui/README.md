# Leaf GUI Utility

Simple local GUI to monitor connected devices and update master parameters over
serial.

## Requirements

- Python 3.9+
- `pyserial`

Install:
```
python -m pip install pyserial
```

## Run

```
python gui/app.py
```

## Notes

- Default port is `/dev/ttyACM0` and baud `115200`.
- The GUI parses `slaves=<n>` from master logs.
- Parameters sent as `speed,probability,mode`.
