# TiSAT CubeSat - Power Analytics Unit 
 
This repository contains the design and development files for the Power Analytics Unit of the **TiSAT CubeSat** project. The primary objective is to monitor and analyze the power consumption of various sensors integrated into the satellite. 
 
## Project Overview 
The core of this project is a custom-designed power measurement circuit. It utilizes a **shunt resistor** to measure minute currents and voltage levels across specific circuit branches, enabling precise power calculation.  
 
Since the satellite is equipped with **solar panels** for battery charging, the power management logic accounts for both energy harvesting and consumption cycles to ensure balanced power distribution. 
 
## Current Progress
- [x] **Circuit Design:** Completed the schematic and PCB layout using **KiCad**.
- [x] **Component Selection:** Selected specialized ICs for precision current sensing (INA199A1) and the STM32L431KC microcontroller.
- [x] **Manufacturing & Assembly:** The custom PCB has been fully manufactured, and all SMD and through-hole components (including critical high-precision IC packages) have been successfully soldered and assembled.
- [x] **Firmware Communication Loop:** Established a low-level hardware communication interface between the STM32 MCU and a Raspberry Pi Pico W using single-wire, half-duplex UART with inverted polarity (open-drain transmitter, 9600 baud). The firmware is bare-metal and uses blocking (polling) USART access.
- [x] **Ping-Pong Verification:** As the first step, confirmed working end-to-end UART communication on the shared single-wire bus — the STM32 replied `pong\n` to every `ping\n` sent by the Raspberry Pi Pico W, verifying the physical link between the two microcontrollers.
- [x] **Shunt Resistor Calibration:** Performed a 4-point (Kelvin) measurement to precisely determine the shunt resistor's true value (52.2865 mΩ, not the nominal 50 mΩ).
- [x] **TiSAT Protocol Implementation:** Replaced the raw ping-pong test with the full, checksum-validated TiSAT frame protocol (`$`/`#` direction, 3-character module ID, payload, checksum, terminator), matching the shared master-side specification used across the TiSAT project's other modules.
- [x] **ADC-Based Power Measurement:** Implemented calibrated ADC sampling of the shunt voltage — using the internal VREFINT reference to compensate for real VDDA deviation and 64x oversampling to suppress noise — converting the INA199 current-sense amplifier output into live current, voltage and power readings, delivered over the `GETDATA` command.
- [x] **Current Calibration & Verification:** Calibrated the current measurement against known reference currents with a straight-line (linear regression) correction, and added a no-load threshold so an unloaded module reports exactly `0`. Verified on 38 reference points between 1 mA and 200 mA: the worst absolute error is 0.194 mA (about 0.1 % of the 200 mA range), the relative error is below 0.3 % from 50 mA up and reaches 8.7 % only at 1 mA, where one ADC step (about 0.31 mA) limits the resolution.

## Software Architecture & Development 
The software side consists of the low-level firmware of the onboard microcontroller and a test master for the receiving subsystem: 
1. **Custom HAL (Hardware Abstraction Layer):** Bare-metal foundational drivers written from scratch in **C++** to manage the MCU peripherals (RCC, GPIO, ADC, USART), without any vendor HAL or RTOS. 
2. **Request-Response Communication:** The STM32 acts as a Slave: it waits for a command frame from the Master (`ping`, `GETDATA`) and answers it over the shared open-drain bus. Reception and transmission use polling in the main loop, which keeps the firmware simple and its timing predictable. 
3. **Calibrated Measurement Pipeline:** Every `GETDATA` request is answered with fresh, calibrated current, voltage and power values, based on the VREFINT-compensated, 64x oversampled ADC readings and the linear-regression current calibration. 
4. **Subsystem Master (Test):** A CircuitPython-based test master on the Raspberry Pi Pico W periodically sends `ping` and `GETDATA` frames, prints the replies, and toggles a diagnostic heartbeat LED. 
 
## About the Author 
This project is being developed as part of my **undergraduate thesis** at the **University of Szeged (SZTE)**, under the guidance of my supervisor.  
 
It serves as a practical application of my ongoing studies in **C, C++, and Assembly**. This mission allows me to bridge the gap between hardware design and low-level software engineering while contributing to the **TiSAT satellite mission**. 
 
--- 
*Thank you for visiting this repository!*
