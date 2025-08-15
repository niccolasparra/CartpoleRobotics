# 🤖 Cart-Pole System: Mathematical Modeling & Control

## 📋 Project Overview

This project presents a comprehensive mathematical analysis of the **Cart-Pole System** (inverted pendulum on a cart) using **Lagrangian Mechanics**. The cart-pole system is a foundational model in robotics and control systems, serving as an excellent platform for understanding dynamic coupling and control strategies for underactuated systems.

### 🎯 Objectives
- Derive equations of motion using systematic Lagrangian approach
- Model the dynamic coupling between linear and angular motion
- Provide foundation for control system design and simulation
- Demonstrate advanced mathematical modeling in robotics

## 🔧 System Description

The cart-pole system consists of:
- **Cart**: Mass M = 2 kg, moving horizontally
- **Rod**: Mass m = 0.5 kg, length 2l = 0.5 m
- **Control Input**: Horizontal force F_ext applied to the cart
- **Objective**: Stabilize the rod in upright position (θ = 0)

### 📐 Key Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| M | 2 kg | Cart mass |
| m | 0.5 kg | Rod mass |
| 2l | 0.5 m | Rod total length |
| l | 0.25 m | Rod half-length |
| I | ⅓ml² | Rod moment of inertia |
| g | 9.81 m/s² | Gravitational acceleration |

## 🔬 Mathematical Approach

### Lagrangian Mechanics
The system dynamics are derived using the energy-based Lagrangian approach:

**L = T - V**

Where:
- **T**: Total kinetic energy (translational + rotational)
- **V**: Potential energy (gravitational)

### 🧮 Key Results

**Kinetic Energy:**
```
T = ½(M + m)ẋ² + mlẋθ̇cos(θ) + ⅔ml²θ̇²
```

**Potential Energy:**
```
V = mgl cos(θ)
```

**Equations of Motion:**
```
(M + m)ẍ + mlθ̈cos(θ) - mlθ̇²sin(θ) = F_ext
⁴⁄₃ml²θ̈ + mgl sin(θ) = -mlẍcos(θ)
```

## 🛠️ Tech Stack

**🔧 Tech:** ![Azure](https://img.shields.io/badge/Azure-%230078D4.svg?style=flat-square&logo=microsoftazure&logoColor=white) ![Python](https://img.shields.io/badge/Python-%233776AB.svg?style=flat-square&logo=python&logoColor=white) ![Notion](https://img.shields.io/badge/Notion-%23000000.svg?style=flat-square&logo=notion&logoColor=white)

## 🌟 Key Features

- ✅ **Systematic Derivation**: Complete mathematical modeling using Lagrangian mechanics
- ✅ **Energy-Based Approach**: Avoids complex constraint force calculations
- ✅ **Nonlinear Dynamics**: Captures full system complexity and coupling effects
- ✅ **Control Foundation**: Provides base for advanced control system design
- ✅ **Underactuated System**: Demonstrates handling of systems with fewer actuators than DOFs

## 📊 System Characteristics

- **🎮 Control Type**: Underactuated (1 input, 2 DOF)
- **⚡ Dynamics**: Nonlinear, inherently unstable
- **🔄 Coupling**: Strong dynamic coupling between cart and pendulum motion
- **🎯 Challenge**: Stabilization of unstable equilibrium point

## 🚀 Applications

This mathematical foundation enables:
- **Control System Design**: LQR, MPC, sliding mode control
- **Simulation Studies**: MATLAB/Simulink, Python implementations
- **Hardware Implementation**: Real-time control systems
- **Educational Platform**: Understanding of advanced robotics concepts

## 📈 Future Work

- Implementation of linear and nonlinear control strategies
- Real-time simulation and visualization
- Hardware prototype development
- Comparative control performance analysis

## 👨‍🎓 Author

**Niccolás Parra**  
