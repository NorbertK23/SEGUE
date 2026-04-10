# SEGUE

SEGUE is an ATmega328P-based two-wheel self-balancing robot project built around an MPU6050 IMU and a TB6612FNG motor driver. The codebase is structured for deterministic embedded control, strict layer boundaries, and register-level AVR work.

## Hardware

- MCU: ATmega328P @ 16 MHz
- IMU: MPU6050 over I2C/TWI
- Motor driver: TB6612FNG
- Platform: two-wheel self-balancing rover

## Repository Layout

- `M2_HW/`: main application and balancing logic
- `libs/`: shared HAL, drivers, services, and utility code

## Architecture

The project follows a strict dependency chain:

`APP -> SVC -> DRV -> HAL`

- `HAL`: direct AVR register access
- `DRV`: hardware device drivers
- `SVC`: conversion, calibration, and control-support logic
- `APP`: balancing behavior, runtime sequencing, and integration

Only these external AVR headers are intended for use in the project:

```c
#include <avr/io.h>
#include <avr/interrupt.h>
```

## Build

The generated firmware build lives under `M2_HW/Debug/`.

Typical build command:

```powershell
cd M2_HW/Debug
make main-build -j1
```

## Current Focus

The repository is currently centered on the balancing controller in `M2_HW/`, with tuning, estimator work, telemetry control, and output-policy experiments for longer stable oscillation and eventual upright hold.
