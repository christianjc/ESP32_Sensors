# ESP32_Sensors

This repository contains idf-components for various sensors specifically designed to interact with the esp32 microcontroller running the esp-idf framework. The primary goal of this project is to provide an easy-to-use interface between popular sensor modules and the ESP32 platform.

## Content

The following sensor libraries/components are currently included in this repository:

- **BNO055 Adafruit's Breakout Board**: This library provides an interface to the BNO055 sensor via the i2c communication protocol. BNO055 is a System in Package (SiP), integrating a triaxial 14-bit accelerometer, a triaxial 16-bit gyroscope with a range of ±2000 degrees per second, a triaxial geomagnetic sensor and a 32-bit cortex M0+ microcontroller running Bosch Sensortec sensor fusion software, in a single package.

## Compatibility

This project has been tested on an ESP32-WROOM-32E running the esp-idf framework, provided by ESPRESSIF (version 4.3.2).

## Getting Started

1. **Prerequisites**: You will need to have the ESP-IDF installed and configured on your computer. Please refer to the [official ESP-IDF installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) to set up the environment.

2. **Installation**:

    - Clone the repository: `git clone https://github.com/[your_github_username]/ESP32_Sensors.git`
    - Navigate into the cloned project's directory: `cd ESP32_Sensors`
    - Build the project: `idf.py build`
    - Flash onto your ESP32 device: `idf.py -p /dev/ttyUSB0 flash` (you might need to replace `/dev/ttyUSB0` with the actual serial port of your ESP32 device)

3. **Usage**: Once you've flashed your ESP32 device with this repository's code, you can start interacting with the BNO055 sensor. 

Please ensure that your sensor is properly connected to your ESP32's I2C pins. Check out Adafruit's [BNO055 guide](https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/overview) for further information about the BNO055 sensor and how to connect it.

## Contributing

Contributions are welcome! Whether you want to add more sensor libraries, improve existing ones, or fix bugs, your help is greatly appreciated. Please make sure to read the contributing guide before making a pull request.

## License

This project is licensed under the MIT License. See `LICENSE` file for more details.

## Contact

If you have any questions or run into any trouble, feel free to open an issue or contact the repository owner. You can typically expect a response within a couple of business days.

