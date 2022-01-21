# BNO055-IMU

This idf-component provides a C Interface for Bosch-Sensortec's BNO055 compatible with Espressif's ESP32 (running esp-idf).
This code allows the esp32 to read vector measurements from the bon055 IMU sensor. If there are any questions or need further functionality
please feel free message me.

# Compatibility

Tested on ESP32-WROOM-32E with BNO055 Adafruit's Breakout Board.

# Supported Interfaces

I²C - fully supported\*
\*UART will be supported in the future.

# Getting Started

NOTE: this code is not production ready yet.

cd <YOUR_PROJECT_ROOT>
mkdir components/
cd components/

for more details see examples/

Wiring

I2C

ADR -> 3v0 (HIGH) -> logic high enables bno055 default address (0x29).
SCL -> SCL (Default: GPIO_NUM_19).
SDA -> SDA (Default: GPIO_NUM_18).
