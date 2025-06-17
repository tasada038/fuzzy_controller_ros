# fuzzy_controller

## Overview

This package implements a Fuzzy Self-Tuning PID Controller in ROS2 C++ for attitude control using IMU yaw angle feedback. The controller uses fuzzy logic to adaptively tune PID parameters based on error and error derivative.

## Mathematical Theory

### 1. PID Control Foundation

The classical PID controller output is given by:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

where:
- $u(t)$: control output
- $e(t) = r(t) - y(t)$: error signal
- $r(t)$: reference (target) signal
- $y(t)$: plant output (actual yaw angle)
- $K_p, K_i, K_d$: proportional, integral, and derivative gains

### 2. Discrete-Time Implementation

For digital implementation with sampling time $T_s$:

$$u[k] = K_p e[k] + K_i T_s \sum_{j=0}^k e[j] + K_d \frac{e[k] - e[k-1]}{T_s}$$

### 3. Fuzzy Logic System

#### 3.1 Fuzzy Sets and Membership Functions

The fuzzy system uses triangular membership functions for input variables:

$$\mu_{A_i}(x) = \max\left(0, 1 - \frac{|x - c_i|}{w_i}\right)$$

where $c_i$ is the center and $w_i$ is the width of the $i$-th membership function.

#### 3.2 Input Variables

**Error**: $e[k] = \theta_{target} - \theta_{actual}$

**Error Derivative**: $\dot{e}[k] = \frac{e[k] - e[k-1]}{T_s}$

#### 3.3 Linguistic Variables

Both error and error derivative are fuzzified using seven linguistic terms:

- **NB**: Negative Big
- **NM**: Negative Medium
- **NS**: Negative Small
- **ZO**: Zero
- **PS**: Positive Small
- **PM**: Positive Medium
- **PB**: Positive Big

#### 3.4 Fuzzy Rule Base

The fuzzy rules follow the general form:

$$\text{IF } e \text{ is } A_i \text{ AND } \dot{e} \text{ is } B_j \text{ THEN } K_p \text{ is } C_{ij}^{Kp}, K_i \text{ is } C_{ij}^{Ki}, K_d \text{ is } C_{ij}^{Kd}$$

#### 3.5 Inference and Defuzzification

Update adaptive gains:
$$
K_p = K_p^{\prime} + \Delta K_p \\
K_i = K_i^{\prime} + \Delta K_i \\
K_d = K_d^{\prime} + \Delta K_d
$$

Using Mamdani inference with center of gravity defuzzification:

$$\Delta K_p = \frac{\sum_{i,j} \mu_{A_i}(e) \cdot \mu_{B_j}(\dot{e}) \cdot K_p^{(i,j)}}{\sum_{i,j} \mu_{A_i}(e) \cdot \mu_{B_j}(\dot{e})}$$

$$\Delta K_i = \frac{\sum_{i,j} \mu_{A_i}(e) \cdot \mu_{B_j}(\dot{e}) \cdot K_i^{(i,j)}}{\sum_{i,j} \mu_{A_i}(e) \cdot \mu_{B_j}(\dot{e})}$$

$$\Delta K_d = \frac{\sum_{i,j} \mu_{A_i}(e) \cdot \mu_{B_j}(\dot{e}) \cdot K_d^{(i,j)}}{\sum_{i,j} \mu_{A_i}(e) \cdot \mu_{B_j}(\dot{e})}$$

### 4. Output Saturation

The final control output is saturated to the range $[-1, 1]$:

$$u_{final} = \text{sat}(u, [-1, 1])$$


## Appendi

1. Fuzzy Self-Tuning PID Control of Hydrogen-Driven Pneumatic
Artificial Muscle Actuator# fuzzy_controller
