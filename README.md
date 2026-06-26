# Magnetic Attitude Control System for CubeSats

<p align="center">
  <img src="figures/air bearing.jpeg" width="700">
</p>

This repository contains the MATLAB implementation developed during my Bachelor's Final Project in Aerospace Engineering at the University of Aveiro. The project focused on the design, simulation and preliminary experimental validation of a magnetic attitude control system for a 1U CubeSat based on magnetorquer actuation and the B-dot detumbling algorithm.

The repository includes the MATLAB scripts developed for spacecraft dynamics modelling, orbital propagation, environmental perturbation modelling, magnetorquer analysis based on the Jiles–Atherton hysteresis model, simulation post-processing, UART communication with the DRV8411AEVM evaluation board, PWM generation and preliminary Hardware-in-the-Loop (HIL) validation using the KitSat platform.

---

## Repository Contents

### Simulation

- `gravitational_acceleration.m`
  - Computes the Earth's gravitational acceleration acting on the spacecraft according to Newton's law of universal gravitation.

- `drag_force_atm.m`
  - Computes the atmospheric drag force acting on the spacecraft during Low Earth Orbit (LEO) propagation.

- `Postprocess_simulink_result.m`
  - Post-processes the Simulink attitude dynamics simulation.
  - Generates representative plots of the orbital trajectory, magnetic field, angular velocity, magnetic moment, control torque, electrical current and rotational kinetic energy.

- `Jiles_Atherton.m`
  - MATLAB implementation of the Jiles–Atherton hysteresis model for nonlinear ferromagnetic materials.

- `Call_function_Jiles_Atherton.m`
  - Executes the Jiles–Atherton model and generates the simulated magnetization (M–H) curve.

### Hardware Validation

- `Preliminary_UART_PWM_testing.m`
  - Preliminary validation of UART communication and PWM generation using the DRV8411AEVM evaluation board.

- `kitsat_magnetorquer_code.m`
  - Real-time implementation of the B-dot controller used during the air-bearing experiments.
  - Reads magnetometer measurements from the KitSat platform and transmits PWM commands to the magnetorquer driver.

---

## Main Features

- CubeSat orbital and attitude dynamics
- Earth's gravitational acceleration modelling
- Atmospheric drag modelling
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
