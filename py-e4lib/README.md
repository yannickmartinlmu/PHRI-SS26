# py-e4lib

Python library for streaming data from Empatica E4 wearable devices via Bluetooth Low Energy (BLE).

## Features

- Simple async API for BLE connection
- Stream BVP, GSR/EDA, Temperature, and Accelerometer data
- Automatic device discovery or connect to specific address
- Callback-based data handling
- Clean, minimal dependencies (just `bleak`)

## Installation

```bash
pip install py-e4lib
```

## Examples

See [examples/](examples/) for more usage patterns:
- `basic_usage.py` - Simple streaming example
- `save_to_csv.py` - Log data to CSV files

## Requirements

- Python 3.10+
- `bleak` for BLE communication
- An Empatica E4 device

## Credits

Based on reverse engineering work by:
- [ismaelwarnants/e4-python-server](https://github.com/ismaelwarnants)
- and myself!

## Disclaimer

This is an unofficial library. Empatica and E4 are trademarks of Empatica Inc.
Not affiliated with or endorsed by Empatica Inc.