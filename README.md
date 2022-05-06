# ![Satellite Logo](https://www.podfeet.com/blog/wp-content/uploads/2018/06/rocket-logo.png) Team Attitude Adjustment

Welcome to the code repository of U of I Senior Capstone team Attitude Adjustment.

#### The Project
Using a SAMD51 microcontroller to manage a modular attitude determination and adjustment system for a NASA TES satellite.

#### Hardware
* Processor: SAMD51
* 2 Magnetorquers
* 1 Flywheel Motor
* IMU
* Current IC
* 4 Sun Sensors
* ...

#### Usage
Please contact repository owner for permission before using code in this repository.

#### Directory Description
**[ADCS](https://github.com/pjpiedmont/ADCS/tree/master/adcs)**: Contains production code for the ADCS hardware

**Vx Directories**: Contain test code for the system with varying levels of functionality

**[satellite-esp32](https://github.com/pjpiedmont/ADCS/tree/master/satellite-esp32)**: Contains the test code that runs on a second microcontroller (ESP32) to simulate the command and data handling system of the satellite.

**[satellite](https://github.com/pjpiedmont/ADCS/tree/master/satellite)**: More simulation code but for the SAMD line of microcontrollers.
