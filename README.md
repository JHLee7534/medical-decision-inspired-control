# Proactive Latency Control (PLC) for LEO Satellite Networks

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Paper](https://img.shields.io/badge/paper-preprint-brightgreen.svg)](https://github.com/JinHyeong-Lee7534/medical-decision-inspired-control)

> **Robust Dual-Loop Adaptation for Predictably Uncertain LEO Networks**

Implementation of the dual-loop supervisory control framework for latency stabilization in Low Earth Orbit (LEO) satellite networks, as described in:

**Lee, J-H. (2025).** *"Proactive Latency Control: Robust Dual-Loop Adaptation for Predictably Uncertain LEO Networks."* [Preprint]

---

## 📋 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Experimental Scenarios](#experimental-scenarios)
- [Reproducing Paper Results](#reproducing-paper-results)
- [Output Files](#output-files)
- [Repository Structure](#repository-structure)
- [Performance Validation](#performance-validation)
- [Citation](#citation)
- [License](#license)
- [Contact](#contact)

---

## 🎯 Overview

Low Earth Orbit (LEO) satellite networks (Starlink, OneWeb, Kuiper) enable telemedicine and real-time interactive applications in underserved regions. However, **predictable periodic handovers** cause latency spikes that conventional reactive congestion control cannot handle effectively.

### The Problem

- ❌ **Reactive algorithms** (CoDel, PIE, GCC) respond *after* violations occur
- ❌ **BBR misinterprets** handover-induced RTT spikes as persistent congestion
- ❌ **Fixed gains** either overreact to transients or underreact to congestion

### Our Solution: PLC (Proactive Latency Control)

PLC is a **dual-loop supervisory controller** that exploits LEO's predictability:

- ✅ **Fast Loop (Pre-Intervention)**: Anticipatory rate modulation 1-2 seconds before handover
- ✅ **Slow Loop (Self-Damping)**: Adaptive gain adjustment to suppress oscillations
- ✅ **Terminal-Side Detection**: RTT-based handover prediction without satellite telemetry
- ✅ **Prediction-Failure Tolerant**: Maintains ≥11% improvement even at 35% FP + 30% FN error rates

### Key Results (Heavy Load: 75 handovers/500s)

| Metric | Improvement | Statistical Significance |
|--------|-------------|-------------------------|
| **p99 Latency** | **-8.83 ms** | p < 0.001 (95% CI: [-12.85, -5.18]) |
| **Variance** | **-893.27 ms²** | p < 0.001 (95% CI: [-931.09, -856.63]) |
| **Compliance (≤100ms)** | **+6.92 pp** | p < 0.001 (95% CI: [6.72, 7.15]) |
| **Goodput Cost** | **-1.3%** | 20.02 kbps (vs 1500 kbps baseline) |

**Real-World Impact**: For 1 million daily telemedicine sessions, this prevents **~69,200 perceptible freezes per day** or **25.3 million violations annually**.

---

## ✨ Key Features

### 1. **Dual-Loop Architecture**
- **Fast Loop**: Pre-intervention band with graduated control response (35-60ms)
  - Response time: ≤1 RTT (~50ms)
  - Mechanism: Linear ramp function `g(ΔT)`
- **Slow Loop**: Self-damping gain adaptation
  - Response time: 200-400ms (10-20 control cycles)
  - Mechanism: Log-domain multiplicative gain update with BIBO stability (Lipschitz constant L ≈ 1.02)

### 2. **Terminal-Side Handover Prediction**
- **Dual-EMA RTT tracking** (α_short=0.30, α_long=0.05)
- **O(1) computational complexity** (<0.02 ms per cycle)
- **85-90% detection accuracy** without satellite telemetry
- **<5% false positive rate** with congestion filtering

### 3. **Baseline Controllers**
- **GCC-like**: Delay-based congestion control (WebRTC standard)
- **BBR v1-like**: Bandwidth-based congestion control (Google BBR)
- Both implemented with realistic parameters from production systems

### 4. **Realistic LEO Environment**
- **Starlink-like parameters**: ~550km altitude, 50ms base RTT
- **Periodic handovers**: 10-16s intervals (configurable)
- **Beam switching**: 2-5s intervals, +30ms delay spikes
- **Doppler effect**: ±8ms sinusoidal jitter (120s period)
- **Burst packet loss**: Research-based loss models (0.5% baseline, 2% during handover)

### 5. **Three Experimental Scenarios**
| Scenario | Handovers/500s | Capacity Drop | Burst Loss | Use Case |
|----------|----------------|---------------|------------|----------|
| **Normal** | ~25 | 25% | 8% | Typical operation, stable weather |
| **Degraded** | ~38 | 30% | 12% | Adverse weather, beam contention |
| **Heavy Load** | ~75 | 45% | 20% | Extreme stress, polar regions |

### 6. **Robustness Analysis**
- **36 FP/FN combinations** tested (Appendix B)
- **Adaptive mode selection** framework provided
- **Regional dominance patterns** identified:
  - **PRE optimal**: High accuracy (FP≤15%, FN≤10%)
  - **SELF optimal**: High false negatives (FN≥20%)
  - **BOTH optimal**: Mixed error profiles

---

## 🚀 Installation

### Requirements
- Python 3.8+
- NumPy ≥ 1.20
- pandas ≥ 1.3
- matplotlib ≥ 3.3
- seaborn ≥ 0.11

### Install Dependencies

```bash
# Clone the repository
git clone https://github.com/JinHyeong-Lee7534/medical-decision-inspired-control.git
cd medical-decision-inspired-control

# Install required packages
pip install -r requirements.txt
```

### Verify Installation

```bash
python LEO_PLC_Final_rule_preho_v1_0.py --help
python test_robustness_v1_0.py --help
```

---

## ⚡ Quick Start

### Run Main Simulation (Normal Scenario)

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

**Expected Runtime**: ~6-8 minutes (without bootstrap), ~20 minutes (with bootstrap)

### Run Robustness Analysis (Appendix B)

```bash
python test_robustness_v1_0.py \
    --N 50000 \
    --scenario normal \
    --seed 42 \
    --outdir results_robustness
```

**Expected Runtime**: ~20-30 minutes (36 FP/FN combinations × 4 configs)

---

## 🧪 Experimental Scenarios

### Scenario 1: Normal Conditions (Paper Section 3.1)

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

**Expected Results**:
- p99 latency reduction: ~7.55 ms
- Variance reduction: ~114 ms²
- Compliance improvement: ~1.05 pp

---

### Scenario 2: Degraded Network (Adverse Weather)

```bash
python LEO_PLC_Final_rule_preho_v1_0.py \
    --N 50000 \
    --rtt-rule \
    --preho-rule \
    --handover-min 8 \
    --handover-max 14 \
    --burst-loss-prob 0.12 \
    --reserve-pct 0.15 \
    --cap-drop 0.70 \
    --bootstrap 1000 \
    --seed 1 \
    --outdir results_degraded
```

**Expected Results**:
- p99 latency reduction: ~4.76 ms
- Variance reduction: ~144 ms²
- Compliance improvement: ~1.49 pp

---

### Scenario 3: Heavy Load (Extreme Stress)

```bash
python LEO_PLC_Final_rule_preho_v1_0.py \
    --N 50000 \
    --rtt-rule \
    --preho-rule \
    --handover-min 5 \
    --handover-max 9 \
    --burst-loss-prob 0.20 \
    --reserve-pct 0.22 \
    --cap-drop 0.55 \
    --bootstrap 1000 \
    --seed 1 \
    --outdir results_heavy
```

**Expected Results** (matches Paper Table 4):
- p99 latency reduction: **8.83 ms** ✓
- Variance reduction: **893.27 ms²** ✓
- Compliance improvement: **6.92 pp** ✓
- Goodput cost: **20.02 kbps** ✓

---

## 📊 Reproducing Paper Results

### Main Results (Section 3.2, Table 3-4)

Run all three scenarios sequentially:

```bash
# Normal scenario
python LEO_PLC_Final_rule_preho_v1_0.py --N 50000 --rtt-rule --preho-rule \
    --handover-min 10 --handover-max 16 --burst-loss-prob 0.08 \
    --reserve-pct 0.12 --cap-drop 0.75 --bootstrap 1000 --seed 1 \
    --outdir results_normal

# Degraded scenario
python LEO_PLC_Final_rule_preho_v1_0.py --N 50000 --rtt-rule --preho-rule \
    --handover-min 8 --handover-max 14 --burst-loss-prob 0.12 \
    --reserve-pct 0.15 --cap-drop 0.70 --bootstrap 1000 --seed 1 \
    --outdir results_degraded

# Heavy load scenario
python LEO_PLC_Final_rule_preho_v1_0.py --N 50000 --rtt-rule --preho-rule \
    --handover-min 5 --handover-max 9 --burst-loss-prob 0.20 \
    --reserve-pct 0.22 --cap-drop 0.55 --bootstrap 1000 --seed 1 \
    --outdir results_heavy
```

**Compare with Paper Table 4** (BBR Baseline):
- Open `results_{scenario}/leo_ho_{X}-{Y}s_prehoRTT_summary.csv`
- Check rows with `config='BOTH'`
- Verify p99, variance, compliance match paper values

---

### Ablation Study (Section 3.3, Table 5)

Results automatically generated in the same run:
- **BASE**: No PLC (reference)
- **PRE**: Pre-intervention only
- **SELF**: Self-damping only
- **BOTH**: Combined mechanism

Check `leo_*_improvements.csv` for delta values vs BASE.

---

### Robustness Analysis (Appendix B)

```bash
# Normal scenario with FP/FN sweep
python test_robustness_v1_0.py \
    --N 50000 \
    --scenario normal \
    --seed 42 \
    --outdir results_robustness_normal

# Generate optimal configuration matrix
python analyze_robustness.py \
    --input results_robustness_normal/robustness_variance_focused.csv \
    --output optimal_matrix.txt
```

**Compare with Paper**:
- Table B.1: Optimal configuration matrix
- Table B.2: Variance reduction by region
- Critical finding: ≥11% reduction even at FP=35%, FN=30%

---

## 📁 Output Files

### Main Simulation Output

```
{outdir}/
├── leo_ho_{min}-{max}s_preho{mode}_summary.csv
│   ├── Columns: baseline, config, mean_latency_ms, p99_latency_ms,
│   │            variance, goodput_kbps, compliance_100ms_%, 
│   │            loss_rate_%, handover_events
│   └── Rows: 8 combinations (2 baselines × 4 configs)
│
├── leo_ho_{min}-{max}s_preho{mode}_improvements.csv
│   ├── Delta values: Δ_mean_ms, Δ_p99_ms, Δ_variance,
│   │                 mean_reduction_%, p99_reduction_%,
│   │                 variance_reduction_%, goodput_gain_%
│   └── Rows: 6 (2 baselines × 3 PLC configs vs BASE)
│
├── leo_ho_{min}-{max}s_preho{mode}_bootstrap_results.csv
│   ├── Bootstrap CIs for all metrics (B=1000)
│   └── Block sizes: [200, 400]
│
├── leo_ho_{min}-{max}s_preho{mode}_comparison.png
│   └── Latency distribution violin plots
│
├── leo_ho_{min}-{max}s_preho{mode}_timeseries.png
│   └── Time-series visualization (4 subplots)
│
└── leo_ho_{min}-{max}s_preho{mode}_bootstrap_ci.png
    └── Bootstrap confidence interval plot
```

### Robustness Analysis Output

```
{outdir}/
├── robustness_variance_focused.csv
│   ├── 144 rows: 36 (FP,FN) × 4 configs
│   ├── Columns: fp_rate, fn_rate, variance, variance_degradation_%,
│   │            goodput_kbps, goodput_degradation_%, p99_ms,
│   │            prediction_accuracy, prediction_precision,
│   │            prediction_recall
│   └── Primary metric: variance_degradation_%
│
├── robustness_variance_bbr.png
│   ├── 2×3 heatmap grid (Variance + Goodput degradation)
│   ├── Rows: Variance (top), Goodput (bottom)
│   ├── Columns: PRE, SELF, BOTH
│   └── Color scale: Green (improvement) ↔ Red (degradation)
│
└── optimal_config_matrix.txt
    └── Table B.1 from paper (6×6 matrix)
```

---

## 📂 Repository Structure

```
medical-decision-inspired-control/
├── LEO_PLC_Final_rule_preho_v1_0.py  # Main simulator (2134 lines)
│   ├── Dual-loop PLC implementation
│   ├── GCC and BBR baseline controllers
│   ├── LEO handover profile generation
│   ├── Bootstrap statistical analysis
│   └── Visualization functions
│
├── test_robustness_v1_0.py           # Robustness analysis (517 lines)
│   ├── FP/FN error injection
│   ├── 36 combination sweep
│   ├── Optimal config selection
│   └── Heatmap generation
│
├── requirements.txt                  # Python dependencies
├── LICENSE                           # MIT License
├── README.md                         # This file
├── CITATION.cff                      # Citation metadata
│
├── docs/                             # Documentation
│   ├── USAGE.md                      # Extended usage guide
│   ├── PARAMETERS.md                 # Parameter descriptions
│   └── OUTPUTS.md                    # Output file formats
│
├── examples/                         # Example scripts
│   ├── run_all_scenarios.sh          # Reproduce all paper results
│   ├── quick_demo.sh                 # 5-minute demo (N=5000)
│   └── analyze_robustness.py         # Post-processing scripts
│
└── results/                          # Example outputs (not tracked)
    ├── results_normal/
    ├── results_degraded/
    └── results_heavy/
```

---

## ✅ Performance Validation

### Computational Overhead (Paper Section 3.4)

| Configuration | Runtime (50k steps) | Per-cycle Cost | Overhead |
|---------------|---------------------|----------------|----------|
| **BBR (Baseline)** | 4.8 s | ~96 μs | - |
| **BBR-PLC (BOTH)** | 6.6 s | ~132 μs | **+0.18%** |

**Key Points**:
- Absolute overhead: 36 μs per 20 ms cycle
- Memory footprint: <100 bytes
- O(1) complexity per control cycle
- Negligible on modern hardware (3.6 GHz CPU)

---

### Statistical Rigor

- **Sample size**: N=50,000 (>80% power, α=0.05)
- **Bootstrap**: B=1,000 iterations
- **Block bootstrap**: Sizes [200, 400] (4-8× decorrelation length)
- **Paired comparison**: Fixed seed (seed=1) for reproducibility
- **Confidence intervals**: 95% CI reported for all metrics
- **Multiple testing**: Results consistent across 3 scenarios

---

## 📖 Citation

If you use this code in your research, please cite:

### BibTeX

```bibtex
@article{lee2025plc,
  title={Proactive Latency Control: Robust Dual-Loop Adaptation for Predictably Uncertain LEO Networks},
  author={Lee, Jin-Hyeong},
  year={2025},
  note={Preprint},
  url={https://github.com/JinHyeong-Lee7534/medical-decision-inspired-control}
}
```

### APA

```
Lee, J.-H. (2025). Proactive Latency Control: Robust Dual-Loop Adaptation 
for Predictably Uncertain LEO Networks. Preprint. 
https://github.com/JinHyeong-Lee7534/medical-decision-inspired-control
```

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

```
Copyright (c) 2025 Jin-Hyeong Lee

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
...
```

---

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Areas for Contribution

1. **Field Validation**: Integration with real Starlink terminals
2. **Multi-Flow Fairness**: Extension to concurrent users
3. **Adaptive Parameter Tuning**: RL-based optimization
4. **ns-3 Integration**: Full-stack network simulation
5. **Codec Co-Optimization**: Joint codec + PLC optimization

---

## 📞 Contact

**Author**: Jin-Hyeong Lee, MD  
**ORCID**: [0009-0008-8242-8444](https://orcid.org/0009-0008-8242-8444)  
**Email**: ljh7534@gmail.com

**Issues & Questions**: [GitHub Issues](https://github.com/JinHyeong-Lee7534/medical-decision-inspired-control/issues)  
**Research Collaboration**: ljh7534@gmail.com

---

## 🙏 Acknowledgments

- Portions of the implementation were assisted by **OpenAI's ChatGPT (GPT-4/GPT-5)** under the author's direct supervision for code verification and parameter sweep automation.
- Experimental design inspired by **medical diagnostic test performance evaluation** and **clinical decision-making principles**.
- LEO network parameters calibrated from published **Starlink measurement studies** [6-12].

---

## 📚 References

Key papers cited in the implementation:

1. **BBR Congestion Control**: Cardwell et al. (2016). "BBR: Congestion-based congestion control." *ACM Queue*.
2. **GCC Algorithm**: Holmer et al. (2016). "A Google Congestion Control Algorithm for Real-Time Communication." *IETF Draft*.
3. **Starlink Measurements**: Michel et al. (2022). "A First Look at Starlink Performance." *ACM IMC*.
4. **LEO Handover Management**: Akyildiz et al. (1999). "Handover Management in Low Earth Orbit Satellite Networks." *Mobile Networks and Applications*.

Full reference list available in the paper.

---

## 🔖 Keywords

`LEO satellites` · `latency control` · `handover detection` · `telemedicine` · `BBR` · `congestion control` · `real-time systems` · `Starlink` · `dual-loop control` · `prediction robustness`

---

## 📊 Project Status

- ✅ **Simulation**: Complete and validated
- ✅ **Paper**: Preprint available
- ⏳ **Field Trials**: Planned (Q2 2025)
- ⏳ **Multi-Flow**: In development
- ⏳ **ns-3 Integration**: Planned

---

## ⭐ Star History

If you find this work useful, please consider starring the repository!

[![Star History Chart](https://api.star-history.com/svg?repos=JinHyeong-Lee7534/medical-decision-inspired-control&type=Date)](https://star-history.com/#JinHyeong-Lee7534/medical-decision-inspired-control&Date)

---

**Last Updated**: 2025-01-28  
**Version**: 1.0.0
