# Proactive Latency Control for LEO Satellite Handovers

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Paper](https://img.shields.io/badge/paper-preprint-green.svg)](https://github.com/JHLee7534/medical-decision-inspired-control)

**Official implementation of the paper:**  
*"Proactive Latency Control: Robust Dual-Loop Adaptation for Predictably Uncertain LEO Networks"*  
Jin-Hyeong Lee, MD (2025)

---

## 🎯 Overview

This repository provides a complete simulation framework for **Proactive Latency Control (PLC)**, a dual-loop supervisory layer(Layer 4.5-like) designed to stabilize end-to-end latency in Low Earth Orbit (LEO) satellite networks. PLC addresses the fundamental challenge of predictable yet disruptive handovers that occur every 5-16 seconds in LEO constellations like Starlink, OneWeb, and Kuiper.

### Key Features

✅ **Dual-Loop Architecture**  
- **Fast Loop**: Pre-intervention band with RTT-based handover detection (≤1 RTT response)  
- **Slow Loop**: Adaptive self-damping gain (200-400ms adjustment period)

✅ **Lightweight Implementation**  
- **O(1) complexity**: ~36 μs per 10ms control cycle  
- **Minimal memory**: ~100 bytes per controller instance  
- **Terminal-feasible**: Suitable for resource-constrained devices

✅ **Comprehensive Evaluation**  
- **3 stress scenarios**: Normal, Degraded, Heavy Load (up to 75 handovers/500s)  
- **2 baseline controllers**: GCC-like (delay-based), BBR v1-like (bandwidth-based)  
- **Rigorous statistics**: 50K samples, Bootstrap CI (B=1,000)

✅ **Fully Reproducible**  
- **Complete source code**: 2,190 lines, extensively documented  
- **Deterministic results**: Fixed random seeds  
- **One-command execution**: All figures/tables regenerated

---

## 📊 Performance Highlights

Under Heavy Load conditions (75 handovers per 500s), PLC demonstrates:

| Metric | BBR Baseline | BBR + PLC | Improvement |
|--------|--------------|-----------|-------------|
| **Mean Latency** | 848.10 ms | 543.70 ms | **-304.40 ms** |
| **P99 Latency** | 1153.85 ms | 1109.33 ms | **-44.52 ms** |
| **≤100ms Compliance** | 0.01% | 7.99% | **+7.98 pp** |
| **goodput** | 746.69 kbps | 746.69 kbps | **-0 kbps** |

*Statistical significance: p < 0.001 (Bootstrap iterations = 1,000)*

**Key Insight**: PLC transforms BBR's severe bufferbloat oscillations into stable low-latency operation through anticipatory handover mitigation and adaptive damping.

---

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/JHLee7534/medical-decision-inspired-control.git
cd medical-decision-inspired-control

# Install dependencies
pip install -r requirements.txt
```

**Requirements:**
- Python 3.8+
- numpy >= 1.20
- pandas >= 1.3
- matplotlib >= 3.3
- seaborn >= 0.11

### Basic Usage

**Run Normal conditions (typical LEO operation):**
```bash
python LEO_PLC_Final_rule_preho_v1_0.py \
    --N 50000 \
    --rtt-rule \
    --preho-rule \
    --handover-min 10 \
    --handover-max 16 \
    --burst-loss-prob 0.08 \
    --reserve-pct 0.12 \
    --cap-drop 0.75 \
    --bootstrap 1000 \
    --seed 1 \
    --outdir results_normal
```

**Expected runtime:** ~6-8 minutes (N=50,000) + 10-15 minutes (Bootstrap)

**Output files:**
```
results_normal/
├── leo_ho_10-16s_prehoRTT_summary.csv          # Performance metrics
├── leo_ho_10-16s_prehoRTT_improvements.csv     # Δ vs baseline
├── leo_ho_10-16s_prehoRTT_bootstrap_results.csv # Confidence intervals
├── leo_ho_10-16s_prehoRTT_comparison.png       # Latency distributions
├── leo_ho_10-16s_prehoRTT_timeseries.png       # Time-series plots
└── leo_ho_10-16s_prehoRTT_bootstrap_ci.png     # Bootstrap CI plots
```

---

## 📚 Experimental Scenarios

The simulation provides three stress-test scenarios representing different LEO operating conditions:

### Scenario 1: Normal Conditions
**Represents:** Typical Starlink operation in stable weather, rural/suburban coverage
```bash
python LEO_PLC_Final_rule_preho_v2_0.py \
    --handover-min 10 --handover-max 16 \
    --cap-drop 0.75 --burst-loss-prob 0.08 --reserve-pct 0.12 \
    --seed 1 --outdir results_normal
```
- **Handover frequency**: ~25 events per 500s
- **Capacity drop**: 25% during handover
- **Burst loss**: 8%

### Scenario 2: Degraded Network
**Represents:** Adverse conditions (rain fade, beam contention, peak hours)
```bash
python LEO_PLC_Final_rule_preho_v2_0.py \
    --handover-min 8 --handover-max 14 \
    --cap-drop 0.70 --burst-loss-prob 0.12 --reserve-pct 0.15 \
    --seed 1 --outdir results_degraded
```
- **Handover frequency**: ~38 events per 500s
- **Capacity drop**: 30% during handover
- **Burst loss**: 12%

### Scenario 3: Heavy Load (Extreme Stress)
**Represents:** Polar regions, severe weather, constellation gaps
```bash
python LEO_PLC_Final_rule_preho_v2_0.py \
    --handover-min 5 --handover-max 9 \
    --cap-drop 0.55 --burst-loss-prob 0.20 --reserve-pct 0.22 \
    --seed 1 --outdir results_heavy
```
- **Handover frequency**: ~75 events per 500s
- **Capacity drop**: 45% during handover
- **Burst loss**: 20%

---

## 🛠️ Command-Line Arguments

### Simulation Control
```
--N INT              Number of timesteps (default: 50000 = 500s)
--seed INT           Random seed for reproducibility (default: 42)
--outdir PATH        Output directory (default: ./results)
--debug              Enable debug output
--no-plot            Disable plotting
--no-handover        Disable handover events (baseline test)
```

### PLC Configuration
```
--preho-rule         Enable pre-handover prediction
--rtt-rule           Use RTT-based detection (vs oracle)
--rtt-spike-threshold FLOAT   RTT ratio threshold (default: 1.20)
--no-congestion-filter        Disable false positive filtering
--g-boost FLOAT      Pre-intervention gain boost (default: 2.0)
--reserve-pct FLOAT  Capacity reserve percentage (default: 0.12)
```

### LEO Network Parameters
```
--handover-min FLOAT    Minimum handover interval in seconds (default: 10.0)
--handover-max FLOAT    Maximum handover interval in seconds (default: 20.0)
--burst-loss-prob FLOAT Burst loss probability (default: 0.15)
--cap-drop FLOAT        Capacity drop multiplier (default: 0.75)
--cap-ramp FLOAT        Capacity ramp-up rate (default: 0.08)
```

### Statistical Analysis
```
--bootstrap INT      Bootstrap iterations (0=disable, default: 0)
--abs-threshold FLOAT  Compliance threshold in ms (default: 100.0)
```

---

## 📈 Understanding the Results

### Output Files Explained

**1. `*_summary.csv`** - Comprehensive performance metrics
```csv
config,mean_latency_ms,variance,p99_latency_ms,compliance_100ms_%,...
BASE,371.94,152819.37,112.43,79.34,...
PRE,68.15,1345.23,67.46,87.12,...
SELF,67.89,1298.45,67.38,87.25,...
BOTH,67.49,1276.58,67.47,87.33,...
```

**2. `*_improvements.csv`** - Relative improvement vs baseline
```csv
config,mean_reduction_%,variance_reduction_%,p99_reduction_%,...
BOTH,-81.87,-99.16,-39.96,...
```

**3. `*_bootstrap_results.csv`** - Statistical confidence intervals
```csv
metric,config,mean,ci_lower,ci_upper,p_value
mean_latency,BOTH,67.49,67.23,67.75,< 0.001
```

### Key Metrics

| Metric | Description | Target |
|--------|-------------|--------|
| **Mean Latency** | Average end-to-end latency | < 80 ms |
| **P99 Latency** | 99th percentile latency | < 100 ms |
| **Variance** | Latency stability | Low variance |
| **Compliance (≤100ms)** | % samples below 100ms | > 90% |
| **Goodput** | Effective throughput | Minimal loss |

---

## 🔬 Architecture Details

### Dual-Loop Control Architecture

```
┌─────────────────────────────────────────────────┐
│            PLC Supervisory Layer                │
│                                                 │
│  ┌──────────────┐         ┌─────────────────┐  │
│  │  Fast Loop   │         │   Slow Loop     │  │
│  │ (≤1 RTT)     │         │ (200-400ms)     │  │
│  │              │         │                 │  │
│  │ • RTT-based  │         │ • Self-damping  │  │
│  │   prediction │         │   gain K_p      │  │
│  │ • g(ΔT)      │         │ • f(r_ema)      │  │
│  │   gating     │         │   homeostatic   │  │
│  └──────┬───────┘         └────────┬────────┘  │
│         │                          │           │
│         └────────┬─────────────────┘           │
│                  ▼                              │
│           rate_multiplier                       │
│                  │                              │
└──────────────────┼──────────────────────────────┘
                   ▼
        ┌──────────────────────┐
        │  Baseline Controller  │
        │  (BBR v1 / GCC)       │
        └──────────────────────┘
```

### Core Algorithm (10ms control cycle)

```python
# 1. Measurement & Smoothing
latency_avg = mean(last_5_measurements)
e_EMA = α * (latency_avg - ΔT_th) + (1-α) * e_EMA_prev

# 2. Fast Loop: Pre-Intervention Band
g = g_pre_intervention(latency_avg)  # Equation (1)
if handover_imminent:
    g *= g_boost  # 2.0× amplification

# 3. Slow Loop: Self-Damping Gain
r_ema = 0.20 * r_instant + 0.80 * r_ema_prev
f_r = f_self_damping(r_ema)          # Equation (10)
K_p = update_gain(K_p, f_r)          # Equations (11-12)

# 4. Control Signal Generation
u = clip(g * K_p * (e_EMA / ΔT_th), 0, 0.60)
rate_multiplier = clip(1 - u, 0.60, 1.20)

# 5. Actuation
rate_target = rate_baseline * rate_multiplier
```

**Computational Cost:** O(1) per cycle, ~36 μs on 3.6 GHz CPU

---

## 🧪 Baseline Controllers

### 1. GCC-like (Delay-Based)
Simplified WebRTC Google Congestion Control (RFC 8888)
- **Trend estimation**: 1D Kalman filter (q=3.0, r=20.0)
- **Over-use detection**: Gradient threshold γ=10.0 ms/s
- **AIMD**: β=0.90 decrease, 1.08× increase
- **Response lag**: ~50-80ms (count-based)

### 2. BBR v1-like (Bandwidth-Based)
Faithful approximation of Google's BBR v1
- **State machine**: STARTUP → DRAIN → PROBE_BW ↔ PROBE_RTT
- **Bandwidth estimation**: Max filter (10-sample window)
- **Min RTT tracking**: 10-second sliding window
- **LEO adaptations**: RTT bounds [20, 200]ms, congestion proxy

**Note:** Both baselines operate at 10ms granularity, synchronized with PLC.

---

## 📖 Citation

If you use this code in your research, please cite:

```bibtex
@article{lee2025plc,
  title={Proactive Latency Control for LEO Satellite Handovers: 
         Performance and Integration Requirements},
  author={Lee, Jin-Hyeong},
  year={2025},
  note={Preprint},
  url={https://github.com/JHLee7534/medical-decision-inspired-control}
}
```

---

## 📁 Repository Structure

```
medical-decision-inspired-control/
├── LEO_PLC_Final_rule_preho_v1_0.py    # Main simulation (2,190 lines)
├── test_robustness_v1_0.py             # Prediction error analysis
├── requirements.txt                     # Python dependencies
├── README.md                            # This file
├── LICENSE                              # MIT License
├── examples/
│   ├── quickstart.sh                   # Example commands
│   ├── run_all_scenarios.sh            # Reproduce paper results
│   └── parameter_sweep.py              # Sensitivity analysis
├── results/                             # Output directory (created)
└── docs/
    ├── ALGORITHM.md                    # Detailed algorithm explanation
    ├── PARAMETERS.md                   # Parameter tuning guide
    └── FAQ.md                          # Common questions
```

---

## 🔧 Advanced Usage

### Parameter Sensitivity Analysis

```python
# Sweep reserve capacity percentage
for reserve in [0.05, 0.10, 0.15, 0.20, 0.25]:
    os.system(f"""
        python LEO_PLC_Final_rule_preho_v1_0.py \
            --reserve-pct {reserve} \
            --seed 1 \
            --outdir results_reserve_{int(reserve*100)}
    """)
```

### Disable PLC Components (Ablation Study)

```bash
# Baseline only (no PLC)
python LEO_PLC_Final_rule_preho_v1_0.py --seed 1 --outdir results_baseline

# Pre-intervention only
python LEO_PLC_Final_rule_preho_v1_0.py --preho-rule --seed 1 --outdir results_pre

# Self-damping only
python LEO_PLC_Final_rule_preho_v1_0.py --seed 1 --outdir results_self

# Both mechanisms (full PLC)
python LEO_PLC_Final_rule_preho_v1_0.py --preho-rule --seed 1 --outdir results_both
```

### Oracle vs RTT-based Detection

```bash
# Oracle mode (perfect prediction, indicator)
python LEO_PLC_Final_rule_preho_v1_0.py --preho-rule --seed 1 --outdir results_oracle

# RTT-based mode (realistic deployment)
python LEO_PLC_Final_rule_preho_v1_0.py --preho-rule --rtt-rule --seed 1 --outdir results_rtt
```

---

## ⚠️ Known Limitations

### Simulation-Based Evaluation
- **Synthetic LEO model**: Simplified handover characteristics
- **No real traces**: Starlink/OneWeb data not included
- **Fixed topology**: No routing changes or gateway coordination
- **→ Priority**: Field trials on actual LEO terminals

### Multi-Flow Behavior
- **Phase-dependent performance**: Excellent in PROBE_BW, catastrophic in STARTUP
- **Asymmetric init issue**: BBR+PLC conflict during capacity discovery
- **→ Solution**: Phase-aware integration (BBR state API required)

### Deployment Requirements
- **CoDel AQM mandatory**: Prevents catastrophic failures (31s → 119ms)
- **BBR v1-specific**: Wrapper designed for BBR's state machine
- **Homogeneous deployment**: Best performance when all flows use PLC

**See paper Section VI for detailed discussion and mitigation strategies.**

---

## 🛣️ Roadmap

### Immediate (0-6 months)
- [ ] CUBIC-PLC simulation test
- [ ] Field trials on Starlink terminals
- [ ] Linux BBR v1 kernel integration
- [ ] Phase-aware BBR-PLC coordination
- [ ] Production A/B testing (telemedicine platforms)

### Medium-term (6-12 months)
- [ ] BBR v2 compatibility
- [ ] Multi-flow (n > 10) validation
- [ ] Weighted schedulers (DRR, priority queues)
- [ ] ns-3 simulator integration

### Long-term (12-24+ months)
- [ ] IETF standardization (CCWG/TSVWG)
- [ ] 6G satellite-terrestrial integration
- [ ] Cross-layer optimization
- [ ] Commercial deployment (SpaceX, OneWeb)

---

## 🤝 Contributing

We welcome contributions! Areas of particular interest:

1. **Real LEO measurements**: Starlink/OneWeb trace collection
2. **BBR v2 integration**: Compatibility with ECN-aware BBR
3. **Multi-flow scenarios**: Scalability validation (n > 10)
4. **Alternative baselines**: TCP Cubic, Copa, PCC Vivace
5. **Deployment guides**: Docker, Kubernetes, OpenWRT

**Please open an issue before submitting major PRs.**

---

## 📧 Contact

**Author:** Jin-Hyeong Lee, MD  
**Email:** ljh7534@gmail.com  
**ORCID:** [0009-0008-8242-8444](https://orcid.org/0009-0008-8242-8444)

**Bug reports:** [GitHub Issues](https://github.com/JHLee7534/medical-decision-inspired-control/issues)  
**Research collaboration:** ljh7534@gmail.com

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

```
Copyright (c) 2025 Jin-Hyeong Lee

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction...
```

---

## 🙏 Acknowledgments

- **OpenAI ChatGPT (GPT-4/GPT-o1)**: Code verification and parameter sweep automation
- **Starlink measurement studies**: Michel et al. (2022), Mohan et al. (2024)
- **BBR development team**: Google (Cardwell et al., 2016)
- **WebRTC GCC**: IETF RMCAT WG (RFC 8888)

---

**Last updated:** 2025.11.18.  
**Version:** 2.0.0
