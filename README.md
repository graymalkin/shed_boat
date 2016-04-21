# Shed Boat

Project for the autonomous boat controller.

## Hardware

We have a uBlox M8 GPS and Compass module. The GPS runs at 9600Bd

| Pin colour   | Function |
|:-------------|:---------|
| Red          | `+5v`    |
| Black        | `Gnd`    |
| Green        | `SCL`    |
| Blue         | `SDA`    |
| Yellow       | `rx`     |
| White        | `tx`     |


## boatymcboatface
This contains some initial code for the Shed Boat. This includes:
1. i2c motor controller
2. PID control for the motor with sensible values
3. Protocol buffers (using nanopb) with the start of how to form the message.

Need doing:
1. Encoding a message and sending this via Zigbee.
