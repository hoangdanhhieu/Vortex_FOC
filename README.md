# Vortex FOC

High-performance, sensorless Field Oriented Control (FOC) firmware for PMSM/BLDC motors,
heavily optimized for STM32G4 microcontrollers (CORDIC, OPAMPs, Injected ADC DMA, CCMRAM).

[![Vortex FOC Demo](Hardware/Prototype/Image.jpg)](https://www.youtube.com/watch?v=LBecr3bSMa8)

[![Watch on YouTube](https://img.shields.io/badge/YouTube-Watch_Video-red?style=for-the-badge&logo=youtube&logoColor=white)](https://www.youtube.com/watch?v=LBecr3bSMa8)

## Key Features
- **Sensorless FOC**: - 48 kHz Control Loop: CCMRAM execution (~10 us compute time).
- STF-SMO Observer: Self-Tuning Filter + Adaptive PLL + 6th harmonic compensation.
- Speed Loop LADRC: 2nd-order Linear Extended State Observer (LESO) for active disturbance rejection.
- Automated Motor ID: Offline Rs, Ls, Flux Linkage, Motor KV, Rotor Inertia (J), and System Gain (b0).
- Advanced Flying Start & Braking: Direct BEMF capture + Active low-side dynamic brake for reverse recovery.
- High-Bandwidth Telemetry: 48kHz sampled telemetry streamed over USB CDC.

## Hardware
- **MCU**: STM32G431 (CORDIC, OPAMPs, ADC DMA).
- **Topology**: 3-Phase Gate Driver, 3-Shunt Low-side current sensing.

## Configurator GUI
Python-based (PySide6) tool for live tuning and visualization:
- **Telemetry**: Real-time plotting with adjustable EMA smoothing.
- **Parameter Tuning**: Real-time gain adjustment with built-in calculators for PI gains, LADRC bandwidth, and filter cutoff.
- **Persistence**: Save/Load parameters to MCU Flash.

## 🛠 Getting Started
### Firmware
```bash
make -j8  # Requires arm-none-eabi-gcc
```
### GUI
```bash
cd Tools/Configurator
pip install -r requirements.txt
python main.py
```

## 🤖 Acknowledgements
This project was developed with the assistance of advanced AI coding assistants. The AI acted as a pair-programmer, contributing to architectural decisions, complex mathematical implementations (FOC, SMO), and boilerplate generation. The core concepts, system integration, validation, and hardware-specific debugging were driven by human engineering.

---
*Developed for high-performance BLDC motor control.*
