# Project with STM32F103C8

## Project Description
This project demonstrates the operation of the STM32F103C8 microcontroller using communication protocols and peripheral devices to expand functionality.

## Main Components

### Microcontroller
- **STM32F103C8**: 32-bit microcontroller with ARM Cortex-M3 core
- Clock frequency: up to 72 MHz
- Built-in Flash memory: 64 KB
- RAM: 20 KB

### Communication Interfaces
#### UART (Universal Asynchronous Receiver/Transmitter)
- Used for serial data transmission
- Configurable transmission speed
- Enables asynchronous information exchange between devices

#### I2C (Inter-Integrated Circuit)
- Serial bus protocol for connecting peripheral devices
- Uses PCF8574T chip to expand the number of available I/O ports

### Sensors and Measurement Components
#### Photoresistor
- Light intensity sensor
- Resistance changes based on light intensity
- Used for measuring illumination levels

#### Potentiometer
- Variable resistor
- Used for manual parameter adjustment
- Allows smooth voltage variation in the circuit

### Port Expansion Chip PCF8574T
- 8-bit input/output port
- Operates on I2C bus
- Expands the number of available GPIO lines of the microcontroller

## Demo Video
[https://www.youtube.com/watch?v=RPj-3OMf48E](https://youtu.be/7wL_6uBOmZM?feature=shared)
