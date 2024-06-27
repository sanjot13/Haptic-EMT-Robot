# Python interface

Python scripts for running admittance control with Bota Systems sensors.

## PySerial

### Dependencies

[pySerial](https://pyserial.readthedocs.io/en/latest/)

### Installation

```
  pip install pyserial
  pip install crc
```

### Usage

Find the port the device is connected to (e.g. COM1 on Windows or /dev/ttyUSB0 on Linux) and change it accordingly in the code. Run the examples like:
```
  python3 admittance_control_hg.py
```
