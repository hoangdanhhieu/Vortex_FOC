# Vortex FOC

High-performance Sensorless Field Oriented Control (FOC) for BLDC/PMSM motors, optimized for STM32G4 microcontrollers.

## 🚀 Key Features
- **Sensorless FOC**: Sliding Mode Observer (SMO) + PLL for position/speed estimation.
- **Optimized Performance**: ~10µs control loop @ 48kHz (48% CPU load @ 170MHz) using CORDIC and CCMRAM.
- **Motor ID**: Automatic measurement of Phase Resistance ($R_s$) and Inductance ($L_s$).
- **Integrated BIST**: High-frequency command profiler (8kHz) for PID tuning.
- **Communication**: High-bandwidth USB CDC protocol for real-time telemetry (1kHz).

## 🧱 Hardware
- **MCU**: STM32G431 (CORDIC, OPAMPs, ADC DMA).
- **Topology**: 3-Phase Gate Driver, 3-Shunt Low-side current sensing.

## 💻 Configurator GUI
Python-based (PySide6) tool for live tuning and visualization:
- **Telemetry**: Real-time plotting with adjustable EMA smoothing.
- **Helpers**: Calculators for Flux Linkage, PI Gains, and Filter Alpha.
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

## 📊 Project Status / Roadmap

- [x] Basic FOC Algorithms & SVPWM
- [x] Sensorless SMO Observer
- [x] High-Speed USB CDC Protocol
- [x] Offline Motor ID (Rs, Ls)
- [x] High-Frequency PID Profiler (BIST)
- [x] Python Configurator GUI integration
- [ ] Field Weakening (Flux Weakening)
- [ ] Online RLS Inductance tracking
- [ ] PWM / DShot Input (1-Wire Interface)

## 🤖 Acknowledgements
This project was developed with the assistance of advanced AI coding assistants. The AI acted as a pair-programmer, contributing to architectural decisions, complex mathematical implementations (FOC, SMO), and boilerplate generation. The core concepts, system integration, validation, and hardware-specific debugging were driven by human engineering.

---
*Developed for high-performance BLDC motor control.*
