# MPC Simulation Archive

This directory contains the main MPC simulation and development experiments created during the F1TENTH project.

Most folders are chronological development snapshots and follow approximately:

```text
YYMMDD_description
```

The code evolved from simple Cartesian kinematic MPC simulations to curvilinear formulations, Dynamic Single-Track models, steering actuator modelling and later simplified Nonlinear Least-Squares controllers used for real-vehicle testing.

## Overview

| Folder | Main purpose | Vehicle model | Framework / Notes |
|---|---|---|---|
| `250103_MPC_sim` | Legacy MPC simulation environment | Kinematic bicycle | MATLAB / Simulink / acados. Early Cartesian trajectory-tracking experiments. |
| `250404_MPC_0_test` | First standalone MPC tests | Kinematic bicycle | MATLAB `fmincon` + early acados tests. Simple reference trajectories and solver experiments. |
| `250414_MPC_test` | Improved trajectory-tracking MPC | Kinematic bicycle | MATLAB + acados. Adds arc-length trajectory parametrization, splines, curvature and heading references. |
| `250720_MPC_curv` | First major curvilinear MPC development | Kinematic and Dynamic Single-Track variants | MATLAB + acados. Introduces curvilinear states, track boundaries, tire-force modelling and progress maximization. |
| `250720_MPC_curv_off` | Development of the Dynamic Single-Track curvilinear MPC | Dynamic Single-Track | MATLAB + acados. Includes lateral velocity, yaw dynamics, tire forces, rolling resistance and friction constraints. |
| `250720_MPC_steering_test` | Steering actuator modelling experiments | Bicycle model + steering dynamics | MATLAB + acados. Tests continuous/discrete servo models and steering time constants. |
| `MPC_eth` | ETH-inspired Single-Track MPC implementation | Dynamic Single-Track | MATLAB + acados. Separate implementation based on an ETH-style vehicle model and MPC formulation. |
| `250916_MPC_curv_eth_py` | Python implementation of the ETH-style controller | Dynamic Single-Track | Python + acados. Curvilinear model with lateral dynamics, yaw dynamics and Pacejka-like tire forces. |
| `250916_MPC_curv_off_py` | Python port of the custom curvilinear controller | Dynamic Single-Track | Python + acados. Uses steering/throttle states, tire-force dynamics and curvilinear track constraints. |
| `250916_MPC_curv_off_ls_py` | NonlinearLS experiment based on the previous dynamic controller | Dynamic Single-Track | Python + acados `NONLINEAR_LS`. Intermediate step toward the later LS controllers. |
| `251007_MPC_curv_ls` | First explicit conversion to Nonlinear Least-Squares MPC | Based on `250916_MPC_curv_off_py` | Python + acados. Conversion of the previous controller to a NonlinearLS cost formulation. |
| `251007_MPC_curv_ls_v1` | Simplified MPC for real-vehicle testing | Kinematic bicycle | Python + acados NonlinearLS. Removes force dynamics. Stable in simulation but unstable on the real vehicle above approximately 2 m/s. |
| `251010_MPC_curv_ls_v2_d_str` | Add steering actuator behaviour to `v1` | Simplified bicycle + steering dynamics | Python + acados NonlinearLS. Includes steering actuation delay/dynamics in the prediction model. |
| `251016_MPC_curv_ls_v3_min_s` | Further simplified curvilinear MPC development | Simplified curvilinear bicycle | Python + acados NonlinearLS. Focuses on progress, velocity control and a simpler prediction model. |
| `251016_MPC_curv_ls_v3_1_min_s` | Evolution of the previous controller | Simplified curvilinear bicycle | Python + acados NonlinearLS. Further changes to the simplified vehicle and cost formulation. |
| `torque_vectoring` | Separate Learning MPCC development | Learning-based / MPCC formulation | MATLAB + FORCESPRO. Includes solver generation, model-learning tools and controller tuning. Not part of the main acados development chain. |

## Main development stages

### 1. Initial kinematic MPC

The first folders contain the initial trajectory-tracking experiments:

```text
250103_MPC_sim
250404_MPC_0_test
250414_MPC_test
```

These versions mainly use a **kinematic bicycle model** and were used to test MPC formulation, trajectory generation, solver configuration and acados integration.

The development progresses from basic Cartesian trajectory tracking to trajectory parametrization using arc length, curvature and heading references.

---

### 2. Curvilinear MPC and Dynamic Single-Track model

The next major development stage is:

```text
250720_MPC_curv
250720_MPC_curv_off
```

These folders introduce the curvilinear representation used throughout most of the later MPC work.

Typical states include quantities such as:

```text
s          progress along the track
n          lateral displacement
mu/theta   heading error
vx         longitudinal velocity
vy         lateral velocity
r          yaw rate
delta      steering angle
T          throttle / longitudinal input state
```

The Dynamic Single-Track implementations include:

- front and rear tire slip angles;
- lateral tire forces;
- longitudinal motor force;
- rolling resistance;
- lateral vehicle dynamics;
- yaw dynamics;
- curvilinear track dynamics;
- track-boundary constraints;
- input and state constraints.

A representative Dynamic Single-Track implementation can be found in:

```text
250720_MPC_curv/car_model_mpc_curv.m
```

and its later variants in:

```text
250720_MPC_curv_off/
```

---

### 3. Steering actuator modelling

```text
250720_MPC_steering_test
```

This branch was used specifically to study the effect of the steering actuator on the MPC prediction model.

It contains several steering-model variants, including:

```text
car_model_servo_v*
car_model_servo_disc_v*
```

with continuous, discrete and first-order actuator models.

This work later motivated the explicit steering dynamics used in:

```text
251010_MPC_curv_ls_v2_d_str
```

---

### 4. Python Dynamic Single-Track implementations

The dynamic controller was later moved to Python.

Two important branches exist:

```text
250916_MPC_curv_eth_py
250916_MPC_curv_off_py
```

#### `250916_MPC_curv_eth_py`

ETH-inspired implementation using a Dynamic Single-Track model.

Typical state vector:

```text
[s, n, theta, vx, vy, delta, yaw_rate, accel]
```

Typical inputs:

```text
[jerk, delta_rate]
```

The model includes tire slip angles, Pacejka-like tire forces, lateral dynamics, yaw dynamics and curvilinear motion.

The main model is:

```text
bicycle_model.py
```

This is one of the clearest Python references for the full Dynamic Single-Track formulation.

#### `250916_MPC_curv_off_py`

Python evolution of the custom MATLAB controller.

Typical state vector:

```text
[s, n, theta, vx, vy, yaw_rate, delta, throttle]
```

with inputs based on steering and throttle rates.

This version remains closer to the original F1TENTH steering/throttle representation.

---

### 5. Nonlinear Least-Squares MPC

The next development stage moves toward acados' `NONLINEAR_LS` formulation:

```text
250916_MPC_curv_off_ls_py
251007_MPC_curv_ls
```

`251007_MPC_curv_ls` is explicitly a conversion of:

```text
250916_MPC_curv_off_py
```

to a Nonlinear Least-Squares MPC formulation.

---

### 6. Simplified models for the real vehicle

Starting with:

```text
251007_MPC_curv_ls_v1
```

the controller was simplified again.

The README of this version states that it uses a **simple kinematic car model without force dynamics**.

The main motivation was to obtain a simpler and more robust controller for real-vehicle experiments.

Simulation performance was good, but the real vehicle became unstable above approximately:

```text
2 m/s
```

This led to further investigation of steering dynamics.

---

### 7. Steering dynamics and later simplified controllers

```text
251010_MPC_curv_ls_v2_d_str
251016_MPC_curv_ls_v3_min_s
251016_MPC_curv_ls_v3_1_min_s
```

`251010_MPC_curv_ls_v2_d_str` adds a steering actuator model with actuation delay to the previous simplified controller.

The later `v3` folders continue experimenting with simplified curvilinear dynamics, progress maximization, longitudinal acceleration and steering behaviour.

These later models should **not** be confused with the full Dynamic Single-Track model: several lateral tire-force states and equations were intentionally removed or simplified.

## Development lineage

The main development sequence can be roughly represented as:

```text
250103_MPC_sim
        ↓
250404_MPC_0_test
        ↓
250414_MPC_test
        ↓
250720_MPC_curv
        ↓
250720_MPC_curv_off
        ↓
250916_MPC_curv_off_py
        ↓
250916_MPC_curv_off_ls_py
        ↓
251007_MPC_curv_ls
        ↓
251007_MPC_curv_ls_v1
        ↓
251010_MPC_curv_ls_v2_d_str
        ↓
251016_MPC_curv_ls_v3_min_s
        ↓
251016_MPC_curv_ls_v3_1_min_s
```

Two relevant side branches are:

```text
250720_MPC_steering_test
        └── steering actuator modelling
```

and:

```text
MPC_eth
   ↓
250916_MPC_curv_eth_py
        └── ETH-inspired Dynamic Single-Track formulation
```

## Dynamic Single-Track simulations

For anyone specifically looking for the **Dynamic Single-Track model**, the most relevant folders are:

```text
250720_MPC_curv
250720_MPC_curv_off
MPC_eth
250916_MPC_curv_eth_py
250916_MPC_curv_off_py
250916_MPC_curv_off_ls_py
```

The later `251007_*` and `251016_*` folders mainly correspond to the subsequent simplified NonlinearLS development and should not automatically be considered full Dynamic Single-Track implementations.

## Typical files

Depending on the version, the most relevant files are usually:

```text
bicycle_model.py
bicycle_model.m
car_model*.m
acados_settings.py
acados_settings.m
setup_ocp*.m
sim_*.py
sim_*.m
mpc_config.yaml
car_model.yaml
```

Generated or experimental files may also be present:

```text
acados_ocp*.json
c_generated_code/
Results/
*.mat
*.npz
```

The individual `Readme.md` files present in some of the later folders should be considered the reference description for those specific experiments.
