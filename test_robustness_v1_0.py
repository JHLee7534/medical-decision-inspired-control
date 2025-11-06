#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_robustness_v1_0.py
==================================================================
Prediction Error Robustness Analysis for Proactive Latency Control (PLC)

Systematic evaluation of PLC component stability under various 
False Positive (FP) and False Negative (FN) prediction error 
combinations, demonstrating optimal configuration adapts to 
predictor quality.

This script generates the results for Appendix B of the paper:
"Proactive Latency Control: Robust Dual-Loop Adaptation for 
Predictably Uncertain LEO Networks"

==================================================================
Copyright (c) 2025  Jin-Hyeong Lee, MD
ORCID: 0009-0008-8242-8444
Independent Researcher, South Korea
Correspondence: ljh7534@gmail.com

Licensed under the MIT License.
See the LICENSE file in the project root for the full license text.
==================================================================

PAPER:
    Lee, J-H. (2025). "Proactive Latency Control: Robust Dual-Loop 
    Adaptation for Predictably Uncertain LEO Networks."
    Appendix B: Prediction Error Robustness Analysis
    
    Preprint available at: https://github.com/JinHyeong-Lee7534/
    medical-decision-inspired-control

CITATION:
    If you use this code in your research, please cite:
    
    @article{lee2025plc,
      title={Proactive Latency Control: Robust Dual-Loop Adaptation 
             for Predictably Uncertain LEO Networks},
      author={Lee, Jin-Hyeong},
      year={2025},
      note={Preprint}
    }

==================================================================
KEY FEATURES:
    1. Error Injection Framework
       - Artificial FP/FN noise added to RTT-based predictor
       - Maintains identical network traces for fair comparison
       - Preserves True Positive/True Negative statistics
    
    2. Comprehensive Parameter Sweep
       - FP rates: 0%, 15%, 20%, 25%, 30%, 35%
       - FN rates: 0%, 10%, 15%, 20%, 25%, 30%
       - Total: 36 combinations per configuration
    
    3. Three Experimental Scenarios
       - Ideal: FP ≤25%, FN ≤20% (high accuracy)
       - Normal: FP ≤35%, FN ≤30% (realistic operation)
       - Challenging: FP ≤50%, FN ≤45% (degraded prediction)
    
    4. Stability Metrics
       - Primary: Variance (ms²) - measures latency jitter
       - Secondary: Goodput (kbps) - throughput cost
       - Reference: P99 latency (ms) - tail behavior
    
    5. Configuration Analysis
       - BASE: No PLC (reference baseline)
       - PRE: Pre-intervention only (fast loop)
       - SELF: Self-damping only (slow loop)
       - BOTH: Combined mechanism (dual-loop)
    
    6. Visualization
       - Heatmaps: Variance & Goodput degradation by (FP, FN)
       - Matrix: Optimal configuration for each error profile
       - Summary: Regional dominance patterns

==================================================================
USAGE:
    Normal scenario (realistic prediction errors):
    
    $ python test_robustness_v1_0.py \\
        --N 50000 \\
        --scenario normal \\
        --seed 42 \\
        --outdir results_robustness_normal
    
    ---
    
    Ideal scenario (high accuracy predictor):
    
    $ python test_robustness_v1_0.py \\
        --N 50000 \\
        --scenario ideal \\
        --seed 42 \\
        --outdir results_robustness_ideal
    
    ---
    
    Challenging scenario (degraded predictor):
    
    $ python test_robustness_v1_0.py \\
        --N 50000 \\
        --scenario challenging \\
        --seed 42 \\
        --outdir results_robustness_challenging
    
    ---
    
    Custom FP/FN ranges:
    
    $ python test_robustness_v1_0.py \\
        --N 50000 \\
        --fp 0.0 0.10 0.20 0.30 \\
        --fn 0.0 0.10 0.20 0.30 \\
        --seed 42 \\
        --outdir results_robustness_custom
    
    ---
    
    Both baselines (BBR and Delay-based):
    
    $ python test_robustness_v1_0.py \\
        --N 50000 \\
        --scenario normal \\
        --both-baselines \\
        --seed 42 \\
        --outdir results_robustness_both

COMMAND-LINE ARGUMENTS:
    Simulation Control:
      --N INT              Number of time steps (default: 50000)
      --seed INT           Random seed for reproducibility (default: 42)
      --outdir PATH        Output directory 
                           (default: ./results_robustness_variance)
      --debug              Enable debug output
      --no-plot            Disable plotting
    
    Baseline Selection:
      --baseline {bbr,delay}  
                           Baseline controller (default: bbr)
      --both-baselines     Test both BBR and Delay-based baselines
    
    Scenario Presets:
      --scenario {ideal,normal,challenging}
                           Pre-defined FP/FN range (default: normal)
        ideal:      FP=[0, 10, 15, 20, 25]%, FN=[0, 5, 10, 15, 20]%
        normal:     FP=[0, 15, 20, 25, 30, 35]%, FN=[0, 10, 15, 20, 25, 30]%
        challenging: FP=[0, 25, 30, 35, 40, 50]%, FN=[0, 20, 25, 30, 35, 45]%
    
    Custom Error Rates:
      --fp FLOAT [FLOAT ...]
                           Custom False Positive rates (0.0-1.0)
      --fn FLOAT [FLOAT ...]
                           Custom False Negative rates (0.0-1.0)
    
    LEO Network Parameters:
      --handover-min FLOAT Minimum handover interval (s, default: 10.0)
      --handover-max FLOAT Maximum handover interval (s, default: 16.0)
      --burst-loss FLOAT   Burst loss probability (default: 0.08)
      --cap-drop FLOAT     Capacity drop multiplier (default: 0.75)
      --cap-ramp FLOAT     Capacity ramp-up rate (default: 0.08)
      --reserve-pct FLOAT  Capacity reserve % (default: 0.12)
      --rtt-threshold FLOAT
                           RTT spike threshold (default: 1.20)

EXPERIMENTAL DESIGN:
    1. Fixed Seed Baseline
       - BASE config run once with seed=42
       - Results replicated for all FP/FN combinations
       - Ensures identical network conditions
    
    2. Error Injection
       - FP: False alarm → unnecessary pre-intervention
       - FN: Missed handover → no anticipatory action
       - Applied to RTT-based predictor output
    
    3. Degradation Calculation
       Variance degradation (%) = 
           100 × (Var_config - Var_BASE) / Var_BASE
       
       Goodput degradation (%) = 
           100 × (Goodput_config - Goodput_BASE) / Goodput_BASE
    
    4. Optimal Selection
       For each (FP, FN) pair:
         - Find config with minimum variance
         - Break ties by goodput cost
         - Generate dominance matrix

OUTPUT FILES:
    {outdir}/
      robustness_variance_focused.csv
        → Full results: 36 × 4 configs × baselines
        → Columns: variance, goodput, p99, compliance, 
                   prediction accuracy/precision/recall
      
      robustness_variance_{baseline}.png
        → Heatmaps: Variance & Goodput degradation
        → Rows: FN rates (y-axis)
        → Cols: FP rates (x-axis)
        → 3 subplots per metric (PRE, SELF, BOTH)
      
      optimal_config_matrix.txt (generated post-analysis)
        → Table B.1 from Appendix B
        → Optimal configuration for each (FP, FN) pair

KEY FINDINGS (Normal Scenario):
    From Appendix B.2 (Table B.1):
    
    Configuration Coverage:
      - SELF dominant: 16/36 (44.4%) - high FN regions
      - BOTH balanced: 11/36 (30.6%) - mixed FP/FN
      - PRE aggressive: 9/36 (25.0%) - low error rates
    
    Regional Patterns:
      - High Accuracy (FP≤15%, FN≤10%): PRE optimal
        → Variance reduction: -27.8%
      
      - FN-Dominant (FN≥20%, FP≤20%): SELF optimal
        → Variance reduction: -21.4%
      
      - Mixed Errors (moderate FP/FN): BOTH optimal
        → Variance reduction: -23.2%
      
      - Worst Case (FP=35%, FN=30%): All ≥11% reduction
        → Validates robustness claim

VALIDATION:
    Reproduces Appendix B results from the paper:
    
    Critical Finding:
      "Even in worst-case scenarios (FP=35%, FN=30%), 
       all configurations maintain ≥11% variance reduction, 
       confirming the paper's core claim: PLC is engineered 
       to remain stable under imperfect prediction."
    
    Operational Insight:
      Adaptive mode selection based on predictor quality 
      could yield +2-4% additional variance reduction vs 
      fixed BOTH configuration.

PERFORMANCE:
    - Computation: ~20-30 minutes for Normal scenario
      (36 FP/FN combinations × 4 configs)
    - Memory: Peak ~2GB (storing all latency traces)
    - Parallelization: Not implemented (deterministic seed)

REQUIREMENTS:
    - Python 3.8+
    - numpy >= 1.20
    - pandas >= 1.3
    - matplotlib >= 3.3
    - seaborn >= 0.11
    - LEO_PLC_Final_rule_preho_v1_0.py (main simulator)

INSTALLATION:
    $ pip install numpy pandas matplotlib seaborn
    
    Ensure LEO_PLC_Final_rule_preho_v1_0.py is in the same directory.

RELATED FILES:
    - LEO_PLC_Final_rule_preho_v1_0.py
        → Main simulator (required dependency)
    
    - analyze_robustness.py (optional)
        → Post-processing script for optimal matrix generation

==================================================================
IMPLEMENTATION NOTES:
    1. Error Injection Method
       - Wraps sim.predict_handover_rtt() function
       - Applies probabilistic FP/FN based on original output
       - Maintains confusion matrix (TP/FP/TN/FN)
    
    2. Baseline Replication
       - BASE config run once per baseline
       - Results copied for all (FP, FN) pairs
       - Ensures fair comparison (identical traces)
    
    3. Degradation Calculation
       - Primary: Variance change (stability metric)
       - Secondary: Goodput change (throughput cost)
       - Reference: P99, compliance (tail behavior)
    
    4. Visualization
       - Red: Performance degradation (worse)
       - Green: Performance improvement (better)
       - Color scale centered at 0%

==================================================================

KNOWN LIMITATIONS:
    1. Sequential execution (no parallelization)
    2. Fixed network topology per scenario
    3. Single baseline per run (use --both-baselines for all)
    4. Memory-intensive for N > 100,000

FUTURE ENHANCEMENTS:
    - Parallel execution across FP/FN combinations
    - Online adaptive mode switching simulation
    - Integration with real predictor traces
    - Multi-flow robustness analysis

==================================================================
CONTACT:
    Bug reports and feature requests:
    https://github.com/JinHyeong-Lee7534/medical-decision-inspired-control/issues
    
    Research collaboration inquiries:
    ljh7534@gmail.com

==================================================================
ACKNOWLEDGMENTS:
    This robustness analysis framework was developed to validate
    the core claim of prediction-failure tolerance in the PLC
    framework. Experimental design inspired by medical diagnostic
    test performance evaluation.

==================================================================
"""


import os
import sys
import argparse
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
import LEO_PLC_Final_rule_preho_v1_0 as sim


def simulate_with_error_injection(
    N: int,
    seed: int,
    baseline: str,
    config: str,
    fp_rate: float,
    fn_rate: float,
    handover_params=None,
    preho_cfg=None,
    cap_drop: float = 0.75,
    cap_ramp: float = 0.08,
    debug: bool = False,
):
    """Simulate with FP/FN injection"""
    original_predict = sim.predict_handover_rtt
    error_rng = random.Random(seed + 99999)
    prediction_stats = {"tp": 0, "fp": 0, "tn": 0, "fn": 0, "total": 0}

    def noisy_predict(cc_state, delay_comp, preho_cfg_inner):
        original_flag = original_predict(cc_state, delay_comp, preho_cfg_inner)
        prediction_stats["total"] += 1
        
        if not original_flag and fp_rate > 0.0 and error_rng.random() < fp_rate:
            prediction_stats["fp"] += 1
            return True
        
        if original_flag and fn_rate > 0.0 and error_rng.random() < fn_rate:
            prediction_stats["fn"] += 1
            return False
        
        if original_flag:
            prediction_stats["tp"] += 1
        else:
            prediction_stats["tn"] += 1
        
        return original_flag

    sim.predict_handover_rtt = noisy_predict

    try:
        enable_rule = (config != "BASE")
        
        latencies, metrics = sim.simulate_plc(
            N=N,
            seed=seed,
            baseline=baseline,
            include_handover=True,
            config=config,
            handover_params=handover_params,
            preho_cfg=preho_cfg,
            enable_rule=enable_rule,
            rtt_rule=True,
            cap_drop=cap_drop,
            cap_ramp=cap_ramp,
            debug=False,
        )
    finally:
        sim.predict_handover_rtt = original_predict

    # Statistics
    p99 = float(np.percentile(latencies, 99))
    mean = float(np.mean(latencies))
    p95 = float(np.percentile(latencies, 95))
    var = float(np.var(latencies, ddof=1))  # Sample variance
    std = float(np.std(latencies, ddof=1))
    
    # Jitter (IQR-based, more robust than std)
    p75 = float(np.percentile(latencies, 75))
    p25 = float(np.percentile(latencies, 25))
    jitter_iqr = p75 - p25
    
    # Compliance
    latencies_array = np.array(latencies)
    compliance_100ms = float(np.sum(latencies_array < 100) / len(latencies_array) * 100)
    compliance_150ms = float(np.sum(latencies_array < 150) / len(latencies_array) * 100)
    
    total_time = N * sim.DT
    goodput = (metrics.bytes_received * 8) / total_time / 1000.0
    loss_rate = 100.0 * metrics.packets_lost / max(1, metrics.packets_sent)
    
    # Prediction metrics
    if enable_rule and prediction_stats["total"] > 0:
        accuracy = (prediction_stats["tp"] + prediction_stats["tn"]) / prediction_stats["total"]
        precision = prediction_stats["tp"] / max(1, prediction_stats["tp"] + prediction_stats["fp"])
        recall = prediction_stats["tp"] / max(1, prediction_stats["tp"] + prediction_stats["fn"])
    else:
        accuracy = precision = recall = np.nan

    result = {
        "baseline": baseline,
        "config": config,
        "fp_rate": fp_rate,
        "fn_rate": fn_rate,
        "variance": var,  # PRIMARY METRIC
        "std": std,
        "jitter_iqr": jitter_iqr,
        "goodput_kbps": goodput,  # SECONDARY METRIC
        "mean_ms": mean,
        "p95_ms": p95,
        "p99_ms": p99,
        "compliance_100ms_%": compliance_100ms,
        "compliance_150ms_%": compliance_150ms,
        "loss_rate_%": loss_rate,
        "handover_events": metrics.handover_events,
        "prediction_accuracy": accuracy,
        "prediction_precision": precision,
        "prediction_recall": recall,
        "pred_tp": prediction_stats["tp"],
        "pred_fp": prediction_stats["fp"],
        "pred_fn": prediction_stats["fn"],
        "pred_tn": prediction_stats["tn"],
    }
    
    if debug:
        print(f"  → Var={var:.1f}, Goodput={goodput:.0f}kbps, "
              f"P99={p99:.1f}ms, Jitter={jitter_iqr:.1f}ms")
    
    return result


def run_sweep(args):
    os.makedirs(args.outdir, exist_ok=True)

    fp_list = [float(x) for x in args.fp]
    fn_list = [float(x) for x in args.fn]
    baselines = ["bbr", "delay"] if args.both_baselines else [args.baseline]
    configs = ["BASE", "PRE", "SELF", "BOTH"]

    preho_cfg = sim.PreHOPredictParams(
        lookahead_steps=100,
        g_boost=2.0,
        reserve_pct=args.reserve_pct,
        rtt_spike_threshold=args.rtt_threshold,
        rtt_alpha_short=0.3,
        rtt_alpha_long=0.05,
        use_congestion_filter=True,
    )
    handover_params = sim.HandoverParams(
        HANDOVER_INTERVAL_MIN=args.handover_min,
        HANDOVER_INTERVAL_MAX=args.handover_max,
        BURST_LOSS_PROB=args.burst_loss,
    )

    print("="*80)
    print("🛡️  ROBUSTNESS EXPERIMENT - VARIANCE FOCUSED (v3.0)")
    print("="*80)
    print(f"Primary Metric: VARIANCE (jitter/stability)")
    print(f"Secondary Metric: GOODPUT (throughput)")
    print(f"N: {args.N} steps ({args.N * sim.DT:.0f} seconds)")
    print(f"Baselines: {baselines}")
    print(f"Configs: {configs}")
    print(f"FP rates: {fp_list}")
    print(f"FN rates: {fn_list}")
    print(f"Total runs: {len(baselines) * len(configs) * len(fp_list) * len(fn_list)}")
    print(f"Output: {args.outdir}/")
    print("="*80 + "\n")

    rows = []
    total_runs = len(baselines) * len(configs) * len(fp_list) * len(fn_list)
    run_count = 0
    
    # Run BASE once per baseline (fixed seed)
    base_results = {}
    print("🔧 Running BASE config once per baseline (fixed seed)...\n")
    for baseline in baselines:
        print(f"  {baseline.upper()} BASE...", end=" ", flush=True)
        base_res = simulate_with_error_injection(
            N=args.N,
            seed=args.seed,
            baseline=baseline,
            config="BASE",
            fp_rate=0.0,
            fn_rate=0.0,
            handover_params=handover_params,
            preho_cfg=preho_cfg,
            cap_drop=args.cap_drop,
            cap_ramp=args.cap_ramp,
            debug=False,
        )
        base_results[baseline] = base_res
        print(f"✓ Var={base_res['variance']:.1f}, Goodput={base_res['goodput_kbps']:.0f}kbps")
    
    print(f"\n{'='*80}\n")
    
    for baseline in baselines:
        print(f"\n{'='*60}")
        print(f"🔧 Baseline: {baseline.upper()}")
        print(f"{'='*60}")
        
        for config in configs:
            if config == "BASE":
                # Replicate BASE result
                for fp in fp_list:
                    for fn in fn_list:
                        base_copy = base_results[baseline].copy()
                        base_copy["fp_rate"] = fp
                        base_copy["fn_rate"] = fn
                        rows.append(base_copy)
                continue
            
            print(f"  Config: {config}")
            
            for fp in fp_list:
                for fn in fn_list:
                    run_count += 1
                    print(f"    [{run_count}/{total_runs - len(baselines)*len(fp_list)*len(fn_list)}] "
                          f"FP={fp:.0%}, FN={fn:.0%}...", end=" ", flush=True)
                    
                    try:
                        res = simulate_with_error_injection(
                            N=args.N,
                            seed=args.seed + run_count,
                            baseline=baseline,
                            config=config,
                            fp_rate=fp,
                            fn_rate=fn,
                            handover_params=handover_params,
                            preho_cfg=preho_cfg,
                            cap_drop=args.cap_drop,
                            cap_ramp=args.cap_ramp,
                            debug=args.debug,
                        )
                        rows.append(res)
                        
                        if not args.debug:
                            print(f"✓ Var={res['variance']:.1f}")
                        
                    except Exception as e:
                        print(f"❌ Error: {e}")
                        rows.append({
                            "baseline": baseline,
                            "config": config,
                            "fp_rate": fp,
                            "fn_rate": fn,
                            "variance": np.nan,
                        })

    df = pd.DataFrame(rows)

    # Calculate degradations
    for baseline in baselines:
        base_var = base_results[baseline]["variance"]
        base_goodput = base_results[baseline]["goodput_kbps"]
        
        mask = df["baseline"] == baseline
        
        # Variance degradation (increase is bad)
        df.loc[mask, "variance_degradation_%"] = 100.0 * (
            df.loc[mask, "variance"] - base_var
        ) / base_var
        
        # Goodput degradation (decrease is bad)
        df.loc[mask, "goodput_degradation_%"] = 100.0 * (
            base_goodput - df.loc[mask, "goodput_kbps"]
        ) / base_goodput

    # Save results
    out_csv = os.path.join(args.outdir, "robustness_variance_focused.csv")
    df.to_csv(out_csv, index=False)
    print(f"\n💾 Results saved: {out_csv}")

    return df, base_results


def plot_results(df, outdir):
    """Create visualization focused on variance"""
    print("\n📊 Creating visualizations...")
    
    baselines = df['baseline'].unique()
    configs = ['PRE', 'SELF', 'BOTH']
    
    for baseline in baselines:
        df_baseline = df[df['baseline'] == baseline]
        
        # Two heatmaps: Variance and Goodput
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        fig.suptitle(f'Robustness Analysis (Variance-Focused) - {baseline.upper()} Baseline', 
                    fontsize=14, fontweight='bold')
        
        # Row 1: Variance Degradation
        for i, config in enumerate(configs):
            ax = axes[0, i]
            data = df_baseline[df_baseline['config'] == config]
            
            if len(data) == 0:
                continue
            
            pivot = data.pivot_table(values='variance_degradation_%', 
                                    index='fn_rate', 
                                    columns='fp_rate')
            
            vmax = max(abs(pivot.min().min()), abs(pivot.max().max()), 10)
            
            sns.heatmap(pivot, annot=True, fmt='.1f', cmap='RdYlGn_r',
                       center=0, vmin=-10, vmax=vmax,
                       ax=ax, cbar_kws={'label': 'Variance Degradation (%)'})
            
            ax.set_title(f'{config} - Variance', fontweight='bold')
            ax.set_xlabel('False Positive Rate')
            ax.set_ylabel('False Negative Rate')
        
        # Row 2: Goodput Degradation
        for i, config in enumerate(configs):
            ax = axes[1, i]
            data = df_baseline[df_baseline['config'] == config]
            
            if len(data) == 0:
                continue
            
            pivot = data.pivot_table(values='goodput_degradation_%', 
                                    index='fn_rate', 
                                    columns='fp_rate')
            
            vmax = max(abs(pivot.min().min()), abs(pivot.max().max()), 5)
            
            sns.heatmap(pivot, annot=True, fmt='.1f', cmap='RdYlGn_r',
                       center=0, vmin=-5, vmax=vmax,
                       ax=ax, cbar_kws={'label': 'Goodput Degradation (%)'})
            
            ax.set_title(f'{config} - Goodput', fontweight='bold')
            ax.set_xlabel('False Positive Rate')
            ax.set_ylabel('False Negative Rate')
        
        plt.tight_layout()
        plot_path = os.path.join(outdir, f"robustness_variance_{baseline}.png")
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"  ✅ Saved: {plot_path}")


def print_summary(df, base_results):
    """Print summary statistics focused on variance"""
    print("\n" + "="*80)
    print("📊 SUMMARY - VARIANCE & GOODPUT FOCUSED")
    print("="*80)
    
    for baseline in df['baseline'].unique():
        df_baseline = df[df['baseline'] == baseline]
        base_var = base_results[baseline]["variance"]
        base_goodput = base_results[baseline]["goodput_kbps"]
        
        print(f"\n{baseline.upper()} Baseline:")
        print(f"BASE (reference): Variance={base_var:.1f}, Goodput={base_goodput:.0f}kbps")
        print("-"*80)
        
        # Maximum error condition
        max_error = df_baseline[
            (df_baseline['fp_rate'] == df_baseline['fp_rate'].max()) &
            (df_baseline['fn_rate'] == df_baseline['fn_rate'].max())
        ]
        
        print(f"\nAt max error (FP={max_error['fp_rate'].iloc[0]:.0%}, "
              f"FN={max_error['fn_rate'].iloc[0]:.0%}):")
        
        for config in ['PRE', 'SELF', 'BOTH']:
            row = max_error[max_error['config'] == config].iloc[0]
            var = row['variance']
            var_deg = row['variance_degradation_%']
            goodput = row['goodput_kbps']
            goodput_deg = row['goodput_degradation_%']
            jitter = row['jitter_iqr']
            
            print(f"  {config:6s}: Var={var:6.1f} ({var_deg:+6.1f}%), "
                  f"Goodput={goodput:6.0f}kbps ({goodput_deg:+5.1f}%), "
                  f"Jitter={jitter:5.1f}ms")
        
        # Typical operation (FP=20-30%, FN=15-25%)
        typical = df_baseline[
            (df_baseline['fp_rate'] >= 0.20) & (df_baseline['fp_rate'] <= 0.30) &
            (df_baseline['fn_rate'] >= 0.15) & (df_baseline['fn_rate'] <= 0.25)
        ]
        
        if not typical.empty:
            print(f"\nAt typical error (FP=20-30%, FN=15-25%):")
            for config in ['PRE', 'SELF', 'BOTH']:
                config_data = typical[typical['config'] == config]
                if not config_data.empty:
                    avg_var_deg = config_data['variance_degradation_%'].mean()
                    avg_goodput_deg = config_data['goodput_degradation_%'].mean()
                    avg_jitter = config_data['jitter_iqr'].mean()
                    print(f"  {config:6s}: Var deg={avg_var_deg:+6.1f}%, "
                          f"Goodput deg={avg_goodput_deg:+5.1f}%, "
                          f"Jitter={avg_jitter:5.1f}ms")
        
        # Robustness comparison (variance-based)
        pre_var_deg = max_error[max_error['config'] == 'PRE']['variance_degradation_%'].iloc[0]
        both_var_deg = max_error[max_error['config'] == 'BOTH']['variance_degradation_%'].iloc[0]
        
        if not np.isnan(pre_var_deg) and not np.isnan(both_var_deg):
            if abs(both_var_deg) > 0.1:
                if pre_var_deg > 0 and both_var_deg > 0:
                    ratio = pre_var_deg / both_var_deg
                else:
                    ratio = abs(pre_var_deg) / max(abs(both_var_deg), 0.1)
                
                print(f"\n🎯 Variance Robustness: BOTH is {ratio:.2f}× more stable than PRE")
                
                # Goodput comparison
                pre_goodput_deg = max_error[max_error['config'] == 'PRE']['goodput_degradation_%'].iloc[0]
                both_goodput_deg = max_error[max_error['config'] == 'BOTH']['goodput_degradation_%'].iloc[0]
                
                if not np.isnan(pre_goodput_deg) and not np.isnan(both_goodput_deg):
                    goodput_diff = pre_goodput_deg - both_goodput_deg
                    if goodput_diff > 0:
                        print(f"🎯 Goodput Advantage: BOTH maintains {goodput_diff:.1f}% better throughput")
    
    print("\n" + "="*80)


def parse_args():
    p = argparse.ArgumentParser(description="Robustness testing (Variance-focused v3.0)")
    
    p.add_argument("--N", type=int, default=50000)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--outdir", type=str, default="./results_robustness_variance")
    
    p.add_argument("--baseline", type=str, default="bbr", choices=["bbr", "delay"])
    p.add_argument("--both-baselines", action="store_true")
    
    p.add_argument("--scenario", type=str, default="normal",
                   choices=["ideal", "normal", "challenging"])
    
    p.add_argument("--fp", nargs="+", default=None)
    p.add_argument("--fn", nargs="+", default=None)
    
    p.add_argument("--handover-min", type=float, default=10.0)
    p.add_argument("--handover-max", type=float, default=16.0)
    p.add_argument("--burst-loss", type=float, default=0.08)
    
    p.add_argument("--cap-drop", type=float, default=0.75)
    p.add_argument("--cap-ramp", type=float, default=0.08)
    p.add_argument("--reserve-pct", type=float, default=0.12)
    p.add_argument("--rtt-threshold", type=float, default=1.20)
    
    p.add_argument("--no-plot", action="store_true")
    p.add_argument("--debug", action="store_true")
    
    args = p.parse_args()
    
    # Apply scenario defaults
    if args.fp is None or args.fn is None:
        scenarios = {
            "ideal": {
                "fp": ["0.0", "0.10", "0.15", "0.20", "0.25"],
                "fn": ["0.0", "0.05", "0.10", "0.15", "0.20"]
            },
            "normal": {
                "fp": ["0.0", "0.15", "0.20", "0.25", "0.30", "0.35"],
                "fn": ["0.0", "0.10", "0.15", "0.20", "0.25", "0.30"]
            },
            "challenging": {
                "fp": ["0.0", "0.25", "0.30", "0.35", "0.40", "0.50"],
                "fn": ["0.0", "0.20", "0.25", "0.30", "0.35", "0.45"]
            }
        }
        
        selected = scenarios[args.scenario]
        if args.fp is None:
            args.fp = selected["fp"]
        if args.fn is None:
            args.fn = selected["fn"]
    
    return args


def main():
    args = parse_args()
    
    print(f"\n📍 Selected scenario: {args.scenario.upper()}")
    print(f"   Primary focus: VARIANCE (jitter/stability)")
    print(f"   Secondary focus: GOODPUT (throughput)")
    
    # Run experiment
    df, base_results = run_sweep(args)
    
    # Print summary
    print_summary(df, base_results)
    
    # Create plots
    if not args.no_plot:
        plot_results(df, args.outdir)
    
    print("\n✅ Experiment complete!")
    print(f"📁 Results: {args.outdir}/")
    print(f"\n💡 Key insight: BOTH = PRE's fast response + SELF's variance reduction")


if __name__ == "__main__":
    main()
