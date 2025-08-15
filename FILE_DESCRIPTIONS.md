# 📁 Project Files Documentation

## 📋 File Overview

This repository contains the complete mathematical modeling and simulation of a Cart-Pole system for the MECH 483 Robotics course.

### 🔬 Core Files

| File | Type | Description |
|------|------|-------------|
| `NP_2025951018.pdf` | Document | **Complete mathematical derivation** - Detailed Lagrangian analysis, equations of motion, and theoretical foundation |
| `NP_2025951018.m` | MATLAB Code | **Simulation script** - Numerical integration and visualization of cart-pole dynamics |
| `NP_2025951018.slx` | Simulink Model | **Block diagram simulation** - Visual representation of the dynamic system |
| `NP_2025951018.png` | Image | **System visualization** - Diagram or simulation results |

## 🧮 MATLAB Simulation Details

The MATLAB script (`NP_2025951018.m`) includes:

### 📊 Simulation Parameters
- **Simulation Time**: 0 to 5 seconds (dt = 0.01s)
- **Cart Mass**: M = 1.0 kg  
- **Pendulum Mass**: m = 0.1 kg
- **Pendulum Length**: l = 0.5 m
- **Initial Perturbation**: θ₀ = 0.2 rad (≈11.5°)

### 🎯 Features
- ✅ **Numerical Integration**: Uses MATLAB's `ode45` solver
- ✅ **Multi-plot Visualization**: 4-panel analysis including:
  - Cart position vs time
  - Pendulum angle vs time  
  - Cart phase portrait (position vs velocity)
  - Pendulum phase portrait (angle vs angular velocity)
- ✅ **Animation Function**: Optional real-time visualization
- ✅ **Free Response**: Simulates system without external force (F_ext = 0)

### 🔬 Mathematical Implementation
The code implements the derived equations of motion:

**Cart Acceleration:**
```matlab
x_ddot = (F_ext + m * l * theta_dot^2 * sin_theta - m * g * sin_theta * cos_theta) / denominator;
```

**Pendulum Angular Acceleration:**
```matlab
theta_ddot = (-F_ext * cos_theta - m * l * theta_dot^2 * sin_theta * cos_theta + (M + m) * g * sin_theta) / (l * denominator);
```

Where: `denominator = M + m - m * cos_theta^2`

## 🎮 Usage Instructions

### Running the MATLAB Simulation
1. Open `NP_2025951018.m` in MATLAB
2. Run the script to generate plots and analysis
3. Uncomment line 79 to enable animation: `animate_cartpole(t_sol, x, theta, l);`

### Opening the Simulink Model
1. Open `NP_2025951018.slx` in MATLAB/Simulink
2. Run the simulation to observe block-diagram based results

## 📈 Expected Results

The simulation demonstrates:
- **Unstable Equilibrium**: Small perturbations grow over time
- **Nonlinear Dynamics**: Complex coupling between cart and pendulum motion
- **Energy Conservation**: In the absence of damping and external forces
- **Phase Space Behavior**: Characteristic trajectories in state space

## 🎓 Academic Context

**Course**: MECH 483 - Robotics  
**Student**: Niccolás Parra (ID: 2025951018)  
**Topic**: Cart-Pole System Mathematical Modeling using Lagrangian Mechanics

---

*This documentation provides a complete overview of the cart-pole system analysis and simulation components.*
