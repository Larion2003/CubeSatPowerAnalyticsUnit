# TiSAT CubeSat - Power Analytics Unit

This repository contains the design and development files for the Power Analytics Unit of the **TiSAT CubeSat** project. The primary objective is to monitor and analyze the power consumption of various sensors integrated into the satellite.

## Project Overview
The core of this project is a custom-designed power measurement circuit. It utilizes a **shunt resistor** to measure minute currents and voltage levels across specific circuit branches, enabling precise power calculation. 

Since the satellite is equipped with **solar panels** for battery charging, the power management logic accounts for both energy harvesting and consumption cycles to ensure balanced power distribution.

## Current Progress
- [x] **Circuit Design:** Completed the schematic and PCB layout using **KiCad**.
- [x] **Component Selection:** Selected specialized ICs for precision current sensing and power management.
- [x] **Manufacturing:** The PCB is currently in the manufacturing phase. 
- [ ] **Assembly:** Critical high-precision components (IC packages) will be soldered using a hot-air station, while the remaining SMD and through-hole components will be hand-soldered.

## Technical Roadmap & Development
The next phase focuses on the low-level software development for the onboard microcontroller:
1. **Custom HAL (Hardware Abstraction Layer):** Writing the foundational drivers from scratch to manage MCU peripherals.
2. **Implementation:** Development is primarily in **C++**, with critical, performance-sensitive sections optimized in **Assembly**.
3. **Analytics Software:** Developing the logic to calculate and log real-time sensor power consumption.

## About the Author
This project is being developed as part of my **undergraduate thesis** at the **University of Szeged (SZTE)**, under the guidance of my mentor. 

It serves as a practical application of my ongoing studies in **C, C++, and Assembly**. This mission allows me to bridge the gap between hardware design and low-level software engineering while contributing to the **TiSAT satellite mission**.

---
*Thank you for visiting this repository!*