# PLC-Vehicles  
Adaptive Steering Control Wrapper for Lateral Stability  
*(for `PLC_vehicles.py`)*

---

## 📌 Table of Contents
- [1. Overview](#1-overview)
- [2. Algorithm Architecture](#2-algorithm-architecture)
  - [2.1 Base Controller](#21-base-controller)
  - [2.2 PLC Wrapper (Slow Loop Only)](#22-plc-wrapper-slow-loop-only)
- [3. Simulation Scenarios](#3-simulation-scenarios)
- [4. Results Summary](#4-results-summary)
- [5. Interpretation](#5-interpretation)
- [6. File Structure](#6-file-structure)
- [7. How to Run](#7-how-to-run)
- [8. Key Parameters](#8-key-parameters)
- [9. Limitations](#9-limitations)
- [10. Future Improvements](#10-future-improvements)
- [11. References](#11-references)
- [12. License](#12-license)

---

## 1. Overview

**PLC-Vehicles** is a lightweight adaptive control wrapper designed to improve vehicle lateral stability under challenging road conditions such as:

- Curved roads  
- S-curves  
- Continuous turns  
- Sudden obstacle avoidance  

The controller adapts steering proportional gain \(K_p\) based on real-time *driving stress* using a **log-domain bounded update**, ensuring smooth and stable gain adjustment.

✔ Model-free  
✔ O(1) computational cost  
✔ Works on top of existing PID steering controllers  
✔ Optional conditional activation  

---

## 2. Algorithm Architecture

### 2.1 Base Controller
A standard **PID lateral controller** computes the steering command from:

- Lateral error  
- Heading error  
- Cross-track dynamics  

This acts as the baseline controller.

---

### 2.2 PLC Wrapper (Slow Loop Only)

The adaptive gain update follows:

\[
\log K(t+1)
= \log K(t) + \text{clip}(\Delta\log,\,-0.01,\,0.01)
\]

- Gain increases when curvature or lateral stress rises  
- Gain decreases when oscillation risk increases  
- Conditional PLC activates only above a curvature threshold  

⚠ **Note:** This implementation includes **Slow Loop only**.  
Predictive Fast Loop (feedforward) is **not implemented**.

---

## 3. Simulation Scenarios

The script evaluates four typical driving challenges:

1. **single_curve** – smooth, single-direction curve  
2. **s_curve** – direction-reversing curvature  
3. **continuous_curves** – repeated turns  
4. **sudden_obstacle** – fast avoidance maneuver  

Controllers tested:
- **PID**
- **PLC-Always**
- **PLC-Cond** (conditional activation)

Metric:  
**Mean lateral error**

---

## 4. Results Summary

### Mean Lateral Error
```
Scenario             |    PID    | PLC-Always | PLC-Cond
---------------------------------------------------------
single_curve         |   0.5599  |   0.5108   |  0.5599
s_curve              |   0.4968  |   0.6645   |  0.4840
continuous_curves    |   0.5204  |   0.5080   |  0.5204
sudden_obstacle      |   1.1481  |   0.8271   |  0.8702
```

### Improvement Over PID
```
Scenario             | PLC-Always | PLC-Cond
------------------------------------------------
single_curve         |   +8.8%    |    0.0%
s_curve              |  -33.8%    |   +2.6%
continuous_curves    |   +2.4%    |    0.0%
sudden_obstacle      |  +28.0%    |  +24.2%
```

---

## 5. Interpretation

### ✔ Emergency Steering Improvement
Under sudden obstacle:
- PLC-Always: **+28% improvement**
- PLC-Cond: **+24% improvement**

→ Adaptive gain boosts controller authority effectively.

### ✔ S-curve Behavior
- PLC-Always: large overshoot (**−33.8% worse**)  
- PLC-Cond: activates only during high curvature → **+2.6% improvement**

### ✔ Mild Curves
- Small improvements or neutral effect  
- PLC-Conditional often remains inactive

---

## 6. File Structure

```
PLC_vehicles.py
├── VehicleModel
├── PIDController
├── PLCWrapper
├── ScenarioGenerator
├── SimulationRunner
└── PlotUtils
```

---

## 7. How to Run

### Requirements
```
Python >= 3.8
numpy
matplotlib
```

### Execute
```
python PLC_vehicles.py
```

---

## 8. Key Parameters

### PLC Parameters
| Parameter | Description | Default |
|----------|-------------|---------|
| curvature_threshold | PLC-Cond activation threshold | 0.022 |
| log_update_step | Max log gain update step | 0.01 |
| base_Kp | Base steering gain | scenario-dependent |
| alpha_var | Variance smoothing | 0.1 |

---

## 9. Limitations
- No Fast Loop (predictive layer missing)  
- Simplified vehicle kinematics  
- No noise/real sensor model  
- No real-world validation  

---

## 10. Future Improvements
- Add Fast Loop feedforward  
- Adaptive curvature threshold  
- Multi-speed dynamics  
- Integration with MPC/path planning  

---

## 11. References
- Proactive Latency Control (TechRxiv, 2025)
- PID lateral control literature  
- Vehicle bicycle model dynamics  

---

## 12. License
MIT License
