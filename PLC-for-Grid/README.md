# Proactive Frequency Control (PFC) for Power Grid

A Lyapunov-certified adaptive frequency control algorithm for Energy Storage Systems (ESS) in power grids.

## Overview

This project implements the **Proactive Frequency Control (PFC)** algorithm, adapted from the "Proactive Latency Control" framework (Lee, 2025) for power grid frequency regulation. The PFC algorithm provides mathematically guaranteed stability through Lyapunov analysis while achieving superior frequency control performance compared to conventional droop control.

## Key Features

- **Lyapunov Stability Guarantee**: Mathematically proven BIBO stability and Lipschitz continuity
- **Adaptive Gain Control**: Automatically adjusts controller gain based on system conditions
- **Log-bounded Updates**: Prevents gain divergence with ±2%/step rate limiting
- **Hybrid Mode Support**: Emergency mode for large disturbances with guaranteed stability

## Algorithm Comparison

| Algorithm | Large Disturbance | Normal Disturbance | Lyapunov Proof |
|-----------|-------------------|-------------------|----------------|
| Droop | Baseline | Baseline | Yes (fixed K) |
| Adaptive | Best | Medium | No (Heuristic) |
| PFC | Good (+31.2%) | Best (+29.1%) | **Yes - Verified** |
| PFC-Hybrid | Good (+35.1%) | Good (+18.8%) | Yes (switched) |

## Mathematical Foundation

### Core Equations

**1. Normalized Error Ratio (Eq. 8-9)**
```
r_instant(t) = |f_nom - f(t)| / Δf_tolerance × k_r
r_ema(t) = α_r × r_instant(t) + (1-α_r) × r_ema(t-1)
```

**2. Damping Function (Eq. 10)**
```
f(r) = 1.0,                        if r_min ≤ r_ema ≤ r_max
     = 1.0 + θ × (r - r_max),      if r > r_max (increase K)
     = 1.0 - θ × (r_min - r) × 0.3, if r < r_min (decrease K)
```

**3. Log-bounded K Update (Eq. 11-12)**
```
log K(t+1) = log K(t) + clip(log f(r_ema), -ε, +ε)
K(t+1) = clip(exp(log K(t+1)), K_min, K_max)
```

**4. Lyapunov Function (Eq. 16)**
```
V(t) = 0.5 × e²(t) + 0.5 × (K(t) - K*)²
```

### Stability Guarantees

| Property | Guarantee |
|----------|-----------|
| BIBO Stability | K ∈ [K_min, K_max] always maintained |
| Lipschitz Continuity | \|K(t+1)/K(t)\| ≤ exp(ε) ≈ 1.02 |
| Asymptotic Convergence | V → minimum as e → 0 |

## Installation

### Requirements
- Python 3.8+
- NumPy
- Matplotlib

### Install Dependencies
```bash
pip install numpy matplotlib
```

## Usage

### Basic Execution
```bash
python Proactive_Frequency_Control_V_2.py
```

### Using Individual Controllers

```python
from Proactive_Frequency_Control_V_2 import PFC, Droop, GridParams

# Initialize controller
controller = PFC(
    K_init=0.6,      # Target gain
    K_min=0.3,       # Minimum gain
    K_max=1.5,       # Maximum gain
    P_rated=1.0      # Rated power [p.u.]
)

# Compute control output
grid = GridParams()
P_output, K_current = controller.compute(freq=59.95, f_nom=grid.f_nom)
```

### Controller Classes

| Class | Description |
|-------|-------------|
| `Droop` | Fixed droop controller (baseline) |
| `Droop_Adaptive` | Heuristic adaptive droop controller |
| `PFC` | Lyapunov-certified PFC controller |
| `PFC_Hybrid` | PFC with emergency mode switching |

## Output Files

Running the script generates the following output files:

| File | Description |
|------|-------------|
| `pfc_test1_gen_trip.png` | Generator trip response comparison |
| `pfc_test2_renewable.png` | Renewable fluctuation response comparison |
| `pfc_lyapunov_analysis_renewable.png` | Lyapunov stability analysis (renewable) |
| `pfc_lyapunov_analysis_gen_trip.png` | Lyapunov stability analysis (gen trip) |

## Test Scenarios

### Test 1: Generator Trip (Large Disturbance)
- Simulates sudden loss of 0.5 p.u. generation at t=10s
- Duration: 60 seconds
- Evaluates transient response capability

### Test 2: Renewable Fluctuation (Normal Disturbance)
- Simulates continuous renewable energy variability
- Duration: 300 seconds
- Evaluates steady-state regulation performance

### Test 3: Statistical Verification
- Runs 20 different random seeds
- Validates consistent performance across scenarios

### Test 4: Lyapunov Function Tracking
- Tracks V(t) evolution over time
- Verifies stability conditions numerically

## Performance Results

### Generator Trip (Test 1)
```
Config          RMS Dev(mHz)  Max Dev(mHz)  Improvement
Droop              278.51       313.68         -
Adaptive           151.23       167.79       +45.7%
PFC                191.73       291.76       +31.2%
PFC-Hybrid         180.77       201.21       +35.1%
```

### Renewable Fluctuation (Test 2)
```
Config          RMS Dev(mHz)  Max Dev(mHz)  Improvement
Droop               32.27        70.40         -
Adaptive            26.92        57.33       +16.6%
PFC                 22.87        49.64       +29.1%
PFC-Hybrid          26.19        51.38       +18.8%
```

## Configuration Parameters

### PFC Controller Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `K_init` | 0.6 | Initial/target gain K* |
| `K_min` | 0.3 | Minimum gain bound |
| `K_max` | 1.5 | Maximum gain bound |
| `freq_tolerance` | 0.03 | Frequency tolerance Δf_tol [Hz] |
| `k_r` | 2.0 | Sensitivity boost factor |
| `alpha_r` | 0.30 | EMA smoothing coefficient |
| `r_min` | 0.80 | Stable region lower bound |
| `r_max` | 1.20 | Stable region upper bound |
| `theta` | 0.5 | Gain adjustment strength |
| `epsilon` | 0.02 | Log-domain update limit (±2%/step) |

### Grid Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `f_nom` | 60.0 | Nominal frequency [Hz] |
| `M` | 3.0 | Inertia constant [s] |
| `D` | 1.0 | Damping coefficient |

## Project Structure

```
├── Proactive_Frequency_Control_V_2.py   # Main implementation
├── README.md                             # This file
├── pfc_test1_gen_trip.png               # Test 1 results
├── pfc_test2_renewable.png              # Test 2 results
├── pfc_lyapunov_analysis_renewable.png  # Lyapunov analysis
└── pfc_lyapunov_analysis_gen_trip.png   # Lyapunov analysis
```

## References

1. Lee, J. (2025). "Proactive Latency Control" - Original PLC framework for network systems
2. Kundur, P. (1994). "Power System Stability and Control" - Power system dynamics fundamentals

## License

This project is provided for research and educational purposes.

## Author

Adapted for power grid frequency control applications.
