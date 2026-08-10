# Multi Sensor Fusion Localization

**Multi Sensor Fusion Localization using Kalman Filters**

A modular C++/ROS 2 localization framework for estimating robot/vehicle state by combining measurements from multiple sensors using Kalman-filter-based state estimation.

The project is designed with a separation between the **localization/estimation core** and the **ROS 2 interface**, making the estimation algorithms reusable outside of ROS 2.

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420.svg)](https://ubuntu.com/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## Overview

Modern autonomous vehicles and mobile robots typically rely on several complementary sensors for localization.

For example:

- **IMU** provides high-rate motion information but accumulates drift.
- **GNSS** provides globally referenced position but may be noisy or temporarily unavailable.
- **LiDAR odometry** provides relative motion information but is affected by environmental conditions and accumulated drift.
- **Visual odometry** provides motion estimates from camera observations but can fail in challenging visual environments.

Multi-sensor fusion combines these measurements into a single state estimate that is generally more robust than relying on an individual sensor.

This project provides a framework for implementing and experimenting with such localization algorithms using **Kalman filtering and state estimation techniques**.

---

## Architecture

The project is divided into two main components:

```text
msf_localization/
├── msf_localization_core/
│   ├── apps/
│   ├── include/
│   │   └── filters/
│   ├── src/
│   │   └── filters/
│   └── CMakeLists.txt
│
├── msf_localization_ros2/
│   ├── config/
│   ├── include/
│   │   └── msf_localization_ros2/
│   ├── launch/
│   ├── rviz/
│   ├── src/
│   ├── CMakeLists.txt
│   └── package.xml
│
├── LICENSE
└── README.md
```

The repository currently contains a dedicated `msf_localization_core` package for the estimation algorithms and an `msf_localization_ros2` package containing the ROS 2 integration, launch files, configuration and RViz resources. 

### `msf_localization_core`

The core library contains the localization and filtering logic.

Its purpose is to remain independent from ROS-specific message types and communication mechanisms wherever possible.

This makes it possible to use the estimation algorithms in:

- ROS 2 nodes
- standalone C++ applications
- simulation environments
- offline dataset processing
- unit tests and research experiments

### `msf_localization_ros2`

The ROS 2 package provides the interface between the estimation core and the ROS ecosystem.

It contains:

- ROS 2 node implementation
- configuration files
- launch files
- RViz configuration
- ROS 2 publishers/subscribers

---

## State Estimation

The general sensor-fusion pipeline can be represented as:

```text
                  ┌─────────────┐
                  │     IMU     │
                  └──────┬──────┘
                         │
                         ▼
                  ┌─────────────┐
                  │ Prediction  │
                  │    Model    │
                  └──────┬──────┘
                         │
                         ▼
              ┌──────────────────────┐
              │   Kalman Filter      │
              │                      │
              │ State Prediction     │
              │ Measurement Update   │
              └──────────┬───────────┘
                         ▲
                         │
          ┌──────────────┼──────────────┐
          │              │              │
          │              │              │
     ┌────┴────┐    ┌────┴─────┐   ┌────┴─────┐
     │  GNSS   │    │  LiDAR   │   │  Camera  │
     │ Position│    │ Odometry │   │ Odometry │
     └─────────┘    └──────────┘   └──────────┘
                         │
                         ▼
                  Fused State
                         │
                         ▼
               Position / Orientation
                 / Velocity / etc.
```

The filter follows the standard two-stage estimation process:

### Prediction

The previous state is propagated using the process model:

```text
xₖ|ₖ₋₁ = f(xₖ₋₁|ₖ₋₁, uₖ)

Pₖ|ₖ₋₁ = Fₖ Pₖ₋₁|ₖ₋₁ Fₖᵀ + Qₖ
```

where:

- `x` — system state
- `u` — control/input vector
- `P` — state covariance
- `F` — state-transition Jacobian
- `Q` — process-noise covariance

### Measurement Update

When a sensor measurement becomes available:

```text
yₖ = h(xₖ) + vₖ
```

the filter incorporates the measurement:

```text
Kₖ = Pₖ Hₖᵀ (Hₖ Pₖ Hₖᵀ + Rₖ)⁻¹

xₖ = xₖ + Kₖ(yₖ - h(xₖ))

Pₖ = (I - Kₖ Hₖ)Pₖ
```

This structure allows measurements from different sensors to contribute to the same state estimate.

---

## Supported / Planned Filters

The project is intended as a platform for implementing and comparing different state-estimation algorithms.

Potential filters include:

- Linear Kalman Filter (LKF)
- Extended Kalman Filter (EKF)
- Iterated Extended Kalman Filter (IEKF)
- Unscented Kalman Filter (UKF)
- Ensemble Kalman Filter (EnKF)
- Cubature Kalman Filter (CKF)
- Sigma-Point Kalman Filter (SPKF)
- Iterated Sigma-Point Kalman Filter (ISPKF)

The architecture is intended to allow additional filtering algorithms to be introduced without coupling them directly to the ROS 2 layer.

---

## Sensor Fusion

The framework is intended for multi-sensor localization applications such as:

```text
              IMU
               │
               ▼
          ┌──────────┐
          │          │
GNSS ────►│          │◄──── LiDAR Odometry
          │  Fusion  │
Camera ──►│          │
          │          │
          └────┬─────┘
               │
               ▼
        Vehicle State
```

Possible state variables include:

```text
Position
    x, y, z

Orientation
    roll, pitch, yaw

Velocity
    vx, vy, vz
```

The exact state representation depends on the filter and motion model being implemented.

---

## Requirements

### Operating System

- Ubuntu 22.04

### ROS 2

- ROS 2 Humble

### Requirements

- Eigen
- GeographicLib

The ROS 2 package follows the standard ROS 2 package structure with `package.xml`, CMake configuration, launch files and RViz resources.

---

## Installation

Clone the repository:

```bash
git clone -b humble-dev https://github.com/aliaydinkucukcollu/msf_localization.git
cd msf_localization
```

Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

Build the workspace:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Source the workspace:

```bash
source install/setup.bash
```

For VS Code IntelliSense, the repository currently uses:

```bash
ln -s build/compile_commands.json compile_commands.json
```

These are also the build steps currently documented by the repository.

---

## Running

Launch the ROS 2 localization node with:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch msf_localization_ros2 msf_localization_ros2.launch.py
```

The ROS 2 package contains dedicated directories for configuration, launch files and RViz visualization.

---

## Linear Kalman Filter Results

<table>
  <tr>
    <td align="center">
      <img src="docs/images/rviz_lkf_output_from_kitti_2011_09_26_0014.png" width="100%" />
      <br />
      <b>Kitti 2011_09_26_0014 result for LKF</b>
    </td>
    <td align="center">
      <img src="docs/images/rviz_lkf_output_from_kitti_2011_10_03_0042.png" width="100%" />
      <br />
      <b>Kitti 2011_10_03_0042 result for LKF</b>
    </td>
  </tr>
</table>

---
For offline development, recorded datasets can be replayed repeatedly while tuning:

- Process noise `Q`
- Measurement noise `R`
- Initial covariance `P₀`
- Initial state
- Sensor update rates
- Motion-model parameters

---

## Design Goals

The project is being developed around several goals.

### 1. Modular Estimation Algorithms

Filtering algorithms should be independent of the ROS 2 interface.

```text
                ┌────────────────────┐
                │ Localization Core  │
                │                    │
                │  Kalman Filters    │
                │  State Models      │
                │  Measurement Models│
                └─────────▲──────────┘
                          │
                          │
                ┌─────────┴──────────┐
                │      ROS 2         │
                │                    │
                │ Subscribers        │
                │ Publishers         │
                │ Parameters         │
                │ Launch             │
                └────────────────────┘
```

### 2. Sensor Independence

The estimator should be capable of incorporating measurements from different sensor sources without tightly coupling the filtering implementation to a particular sensor driver.

### 3. Research and Development

The project is intended to make it straightforward to experiment with different:

- State representations
- Motion models
- Measurement models
- Kalman-filter variants
- Noise models
- Sensor combinations

### 4. Autonomous Vehicle Applications

The framework is particularly suitable for robotics and autonomous-vehicle localization research where GNSS, IMU and relative odometry sources need to be combined.

---

## Development Roadmap

Planned development areas include:

### Filtering

- [x] Kalman filter framework
- [ ] Extended Kalman Filter
- [ ] Iterated Extended Kalman Filter
- [ ] Unscented Kalman Filter
- [ ] Error-State Kalman Filter
- [ ] Invariant Extended Kalman Filter
- [ ] Sigma-point filters

### Sensor Models

- [ ] IMU propagation
- [ ] GNSS position update
- [ ] GNSS velocity update
- [ ] LiDAR odometry update
- [ ] Visual odometry update
- [ ] Wheel odometry update

### Localization

- [ ] ENU coordinate representation
- [ ] ECEF / geodetic conversion
- [ ] Quaternion-based orientation representation
- [ ] IMU preintegration
- [ ] Sensor extrinsic calibration support
- [ ] Time synchronization handling

### Evaluation

- [ ] Automated trajectory evaluation
- [ ] ATE calculation
- [ ] RPE calculation
- [ ] RMSE statistics
- [ ] Covariance consistency evaluation
- [ ] Dataset examples

### Software Quality

- [ ] Unit tests
- [ ] Integration tests
- [ ] Continuous integration
- [ ] clang-format
- [ ] clang-tidy
- [ ] Doxygen documentation

---

## Project Structure

### Core Library

```text
msf_localization_core/
├── apps/
├── include/
│   └── filters/
├── src/
│   └── filters/
└── CMakeLists.txt
```

This package is responsible for the underlying estimation implementation.

### ROS 2 Interface

```text
msf_localization_ros2/
├── config/
├── include/
│   └── msf_localization_ros2/
├── launch/
├── rviz/
├── src/
├── CMakeLists.txt
└── package.xml
```

This package contains the ROS 2-specific application layer and visualization/configuration resources.

---


## License

This project is licensed under the **MIT License**.

See [LICENSE](LICENSE) for details.

---

## Author

**Ali Aydın Küçükçöllü**

---

## Acknowledgements

This project is developed as an educational and research-oriented framework for understanding and implementing multi-sensor state estimation techniques for robotics and autonomous vehicles.

---

## References

Useful topics for understanding the algorithms implemented in this project include:

- Kalman Filtering
- Bayesian State Estimation
- Nonlinear State Estimation
- Inertial Navigation Systems
- GNSS/INS Sensor Fusion
- Error-State Kalman Filtering
- Lie-group-based State Estimation
- Multi-Sensor Localization

---

**If you find this project useful, consider giving it a ⭐ on GitHub.**