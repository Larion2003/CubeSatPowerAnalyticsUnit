# TiSAT CubeSat - Power Analytics Unit 
 
This repository contains the design and development files for the Power Analytics Unit of the **TiSAT CubeSat** project. The primary objective is to monitor and analyze the power consumption of various sensors integrated into the satellite. 
 
## Project Overview 
The core of this project is a custom-designed power measurement circuit. It utilizes a **shunt resistor** to measure minute currents and voltage levels across specific circuit branches, enabling precise power calculation.  
 
Since the satellite is equipped with **solar panels** for battery charging, the power management logic accounts for both energy harvesting and consumption cycles to ensure balanced power distribution. 
 
## Current Progress
- [x] **Circuit Design:** Completed the schematic and PCB layout using **KiCad**.
- [x] **Component Selection:** Selected specialized ICs for precision current sensing and power management.
- [x] **Manufacturing & Assembly:** The custom PCB has been fully manufactured, and all SMD and through-hole components (including critical high-precision IC packages) have been successfully soldered and assembled.
- [x] **Firmware Communication Loop:** Successfully established a low-level, non-blocking hardware communication interface between the STM32 MCU and a Raspberry Pi Pico W using UART with inverted polarity.
- [x] **Ping-Pong Verification:** Confirmed working end-to-end UART communication on the shared single-wire bus — the STM32 now reliably replies `pong\n` to every `ping\n` sent by the Raspberry Pi Pico W, verifying the physical link between the two microcontrollers.
- [x] **Shunt Resistor Calibration:** Performed a 4-point (Kelvin) measurement to precisely determine the shunt resistor's true value (50 mΩ).
- [x] **TiSAT Protocol Implementation:** Replaced the raw ping-pong test with the full, checksum-validated TiSAT frame protocol (`$`/`#` direction, 3-character module ID, payload, checksum, terminator), matching the shared master-side specification used across the TiSAT project's other modules.
- [x] **ADC-Based Power Measurement:** Implemented calibrated ADC sampling of the shunt voltage — using the internal VREFINT reference to compensate for real VDDA deviation — converting the INA199 current-sense amplifier output into live current, voltage and power readings, delivered over the `GETDATA` command.
- [ ] **End-to-End Master Integration:** Validate live communication between this module and the actual TiSAT master OBC (currently verified against a Raspberry Pi Pico W test harness simulating the master).

## Technical Roadmap & Development 
The software architecture focuses on the low-level firmware development for the onboard microcontroller and the receiving subsystem: 
1. **Custom HAL (Hardware Abstraction Layer):** Writing bare-metal foundational drivers from scratch to manage MCU peripherals (RCC, GPIO, ADC, USART). 
2. **Interrupt-Driven Transmission:** Implemented an optimized background data streaming pipeline in **C++** using the `TXE` interrupt vector. This allows the STM32 to transmit real-time sensor metrics periodically every $X$ seconds without blocking main CPU execution.
3. **Subsystem Receiver:** Developed a non-blocking CircuitPython-based listener on the Raspberry Pi Pico W to process telemetry frames while maintaining a constant diagnostic heartbeat LED toggle.
4. **Bidirectional Half-Duplex Communication:** The immediate next phase is to extend the current single-wire bus setup to full **Half-Duplex bidirectional mode**. The system will be upgraded so that the Pico can parse the incoming `$TM` telemetry packages and safely reply back to the STM32 over the shared Open-Drain channel.
5. **Analytics & Logging:** Developing the final data processing logic to log and analyze historical sensor power consumption. 
 
## About the Author 
This project is being developed as part of my **undergraduate thesis** at the **University of Szeged (SZTE)**, under the guidance of my mentor.  
 
It serves as a practical application of my ongoing studies in **C, C++, and Assembly**. This mission allows me to bridge the gap between hardware design and low-level software engineering while contributing to the **TiSAT satellite mission**. 
 
--- 
*Thank you for visiting this repository!*
