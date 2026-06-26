# Magnetic Attitude Control System for CubeSats

This repository contains the MATLAB scripts developed by the student Beatriz Gomes during the Bachelor's Final Project in Aerospace Engineering at the University of Aveiro.

The project focused on the development and preliminary experimental validation of a magnetic attitude control system for a 1U CubeSat based on magnetorquers and the B-dot detumbling algorithm. The developed code includes spacecraft dynamics post-processing, magnetorquer modelling using the Jiles–Atherton hysteresis model, UART communication with the DRV8411AEVM evaluation board, PWM generation, and Hardware-in-the-Loop (HIL) validation using the KitSat platform.

---

## Repository Contents

### Simulation

- `Orbits_propagation_values4.m`
  - Post-processing of the Simulink attitude control model.
  - Generates plots of orbital trajectory, magnetic field, angular velocity, magnetic moment, torque, electrical current and rotational kinetic energy.

- `Jiles_Antherton_myself.m`
  - MATLAB implementation of the Jiles–Atherton hysteresis model for ferromagnetic materials.

- `plot_JA_myself.m`
  - Generates the simulated magnetization curve (M-H).

### Hardware Validation

- `DRV8411A_uart_test_11_05.m`
  - Preliminary UART communication and PWM validation with the DRV8411AEVM evaluation board.

- `kitsat_magnetorquer_code.m`
  - Real-time B-dot controller implementation used during the air-bearing experiments.
  - Reads magnetometer data from the KitSat platform and sends PWM commands to the magnetorquer driver.

---

## Main Features

- CubeSat attitude dynamics analysis
- Magnetorquer modelling
- B-dot detumbling control
- Jiles–Atherton hysteresis model
- UART communication
- PWM generation
- Hardware-in-the-Loop validation
- Air-bearing experimental testing

---

## Requirements

- MATLAB
- Simulink
- Instrument Control Toolbox

---

## Author

**Beatriz de Ribeiro Gomes**

Bachelor's Degree in Aerospace Engineering

University of Aveiro

2026
