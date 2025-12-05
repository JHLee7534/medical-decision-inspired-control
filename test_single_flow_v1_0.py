#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_single_flow_v1_0.py
==================================================================
Comprehensive Test Suite: BBR + CoDel vs BBR + PLC + CoDel
with Realistic Network Load Conditions for LEO Satellite Networks

@inproceedings{nichols2012controlling,
  title={Controlling queue delay},
  author={Nichols, Kathleen and Jacobson, Van},
  booktitle={ACM Queue},
  volume={10},
  number={5},
  pages={20--34},
  year={2012}
}

@misc{rfc8289,
  author={K. Nichols and V. Jacobson and A. McGregor and J. Iyengar},
  title={{Controlled Delay Active Queue Management}},
  howpublished={RFC 8289},
  year={2018}
}

==================================================================
Copyright (c) 2025  Jin-Hyeong Lee, MD
ORCID: 0009-0008-8242-8444
Independent Researcher, South Korea
Correspondence: ljh7534@gmail.com

Licensed under the MIT License.
See the LICENSE file in the project root for the full license text.
==================================================================

PAPER:
    Lee, J-H. (2025). "Proactive Latency Control: Robust Dual-Loop Adaptation for Predictably Uncertain LEO Networks"
    

CITATION:
    If you use this code in your research, please cite:
    
    @article{lee2025plc,
      title={Proactive Latency Control: Robust Dual-Loop Adaptation for Predictably Uncertain LEO Networks},
      author={Lee, Jin-Hyeong},
      year={2025},
      note={Preprint}
    }
==================================================================

This test suite provides:
1. Realistic network capacity and load scenarios
2. Multiple test conditions (Normal, Degraded, Heavy Load)
3. Comprehensive metrics and visualization
4. Statistical analysis and confidence intervals
5. Automated report generation

Author: Based on Jin-Hyeong Lee's LEO PLC framework
Modified for realistic network testing scenarios
"""

import math
import argparse
import os
import json
from collections import deque
from datetime import datetime
from typing import Dict, List, Tuple, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle
import seaborn as sns

# Set plotting style
sns.set_style("whitegrid")
plt.rcParams['figure.dpi'] = 100
plt.rcParams['savefig.dpi'] = 300

import sys
sys.path.append('/home/claude')
import LEO_PLC_Final_rule_preho_v2_1 as leo


# =============================================================================
# Enhanced CoDel Queue Implementation
# =============================================================================

class Packet:
    """Packet representation for queue management"""
    __slots__ = ("size_bytes", "enqueue_time_s", "flow_id")

    def __init__(self, size_bytes: float, enqueue_time_s: float, flow_id: int = 0):
        self.size_bytes = float(size_bytes)
        self.enqueue_time_s = float(enqueue_time_s)
        self.flow_id = flow_id


class EnhancedCoDelQueue:
    """
    Enhanced CoDel Queue with realistic buffer management
    - Supports byte-based buffer limits
    - Implements RFC 8289 CoDel algorithm
    - Provides detailed queue statistics
    """
    
    def __init__(
        self,
        capacity_kbps: float,
        max_buffer_bytes: int = 100000,  # 100KB buffer
        use_codel: bool = True,
        codel_target_ms: float = 20.0,
        codel_interval_ms: float = 100.0,
    ):
        self.capacity_kbps = capacity_kbps
        self.max_buffer_bytes = max_buffer_bytes
        self.queue_bytes = 0.0
        self._queue = deque()
        
        # CoDel parameters
        self.use_codel = use_codel
        self.codel_target_s = codel_target_ms / 1000.0
        self.codel_interval_s = codel_interval_ms / 1000.0
        
        # CoDel state
        self._first_above_time_s = 0.0
        self._dropping = False
        self._drop_next_s = 0.0
        self._drop_count = 0
        
        # Statistics
        self.stats = {
            'enqueued_packets': 0,
            'enqueued_bytes': 0,
            'dropped_packets': 0,
            'dropped_bytes': 0,
            'codel_drops': 0,
            'buffer_drops': 0,
            'max_queue_bytes': 0,
            'max_sojourn_ms': 0
        }
    
    def _service_bytes_per_second(self) -> float:
        """Calculate service rate in bytes per second"""
        if self.capacity_kbps <= 0:
            return 0.0
        return self.capacity_kbps * 1000.0 / 8.0
    
    def get_queue_delay_ms(self) -> float:
        """Current queueing delay estimate"""
        srv_bps = self._service_bytes_per_second()
        if srv_bps <= 0 or self.queue_bytes <= 0:
            return 0.0
        return (self.queue_bytes / srv_bps) * 1000.0
    
    def set_capacity(self, capacity_kbps: float) -> None:
        """Update link capacity dynamically"""
        self.capacity_kbps = max(0.0, float(capacity_kbps))
    
    def enqueue(self, bytes_in: float, now_s: float) -> bool:
        """
        Enqueue bytes with buffer overflow protection
        Returns True if enqueued, False if dropped
        """
        if bytes_in <= 0:
            return True
        
        # Check buffer limit
        if self.queue_bytes + bytes_in > self.max_buffer_bytes:
            self.stats['buffer_drops'] += 1
            self.stats['dropped_bytes'] += bytes_in
            return False
        
        self._queue.append(Packet(size_bytes=bytes_in, enqueue_time_s=now_s))
        self.queue_bytes += float(bytes_in)
        
        # Update statistics
        self.stats['enqueued_packets'] += 1
        self.stats['enqueued_bytes'] += bytes_in
        self.stats['max_queue_bytes'] = max(self.stats['max_queue_bytes'], self.queue_bytes)
        
        return True
    
    def _codel_should_drop(self, now_s: float) -> bool:
        """CoDel drop decision algorithm"""
        if not self._queue:
            self._first_above_time_s = 0.0
            return False
        
        head = self._queue[0]
        sojourn_s = now_s - head.enqueue_time_s
        sojourn_ms = sojourn_s * 1000.0
        
        # Update max sojourn time statistic
        self.stats['max_sojourn_ms'] = max(self.stats['max_sojourn_ms'], sojourn_ms)
        
        if not self._dropping:
            # Not in dropping state
            if sojourn_s < self.codel_target_s:
                self._first_above_time_s = 0.0
                return False
            
            if self._first_above_time_s == 0.0:
                self._first_above_time_s = now_s + self.codel_interval_s
                return False
            
            if now_s < self._first_above_time_s:
                return False
            
            # Enter dropping state
            self._dropping = True
            self._drop_count = 1
            self._drop_next_s = now_s + self.codel_interval_s / math.sqrt(self._drop_count)
            return True
        else:
            # In dropping state
            if sojourn_s < self.codel_target_s:
                # Exit dropping state
                self._dropping = False
                self._first_above_time_s = 0.0
                return False
            
            if now_s >= self._drop_next_s:
                self._drop_count += 1
                self._drop_next_s = now_s + self.codel_interval_s / math.sqrt(self._drop_count)
                return True
        
        return False
    
    def dequeue(self, dt: float, now_s: float) -> Tuple[float, float]:
        """
        Service the queue for dt seconds
        Returns: (bytes_sent, queue_delay_ms)
        """
        srv_bps = self._service_bytes_per_second()
        if srv_bps <= 0:
            return 0.0, self.get_queue_delay_ms()
        
        capacity_bytes = srv_bps * dt
        bytes_sent = 0.0
        
        # Apply CoDel dropping
        if self.use_codel:
            while self._queue and self._codel_should_drop(now_s):
                dropped_pkt = self._queue.popleft()
                self.queue_bytes = max(0.0, self.queue_bytes - dropped_pkt.size_bytes)
                self.stats['codel_drops'] += 1
                self.stats['dropped_packets'] += 1
                self.stats['dropped_bytes'] += dropped_pkt.size_bytes
        
        # FIFO service
        while self._queue and capacity_bytes > 0.0:
            head = self._queue[0]
            if head.size_bytes <= capacity_bytes:
                # Send complete packet
                send_bytes = head.size_bytes
                capacity_bytes -= send_bytes
                bytes_sent += send_bytes
                self.queue_bytes = max(0.0, self.queue_bytes - send_bytes)
                self._queue.popleft()
            else:
                # Partial packet transmission
                send_bytes = capacity_bytes
                head.size_bytes -= send_bytes
                capacity_bytes = 0.0
                bytes_sent += send_bytes
                self.queue_bytes = max(0.0, self.queue_bytes - send_bytes)
        
        return bytes_sent, self.get_queue_delay_ms()
    
    def get_statistics(self) -> dict:
        """Return queue statistics"""
        return self.stats.copy()


# =============================================================================
# Network Scenario Definitions
# =============================================================================

class NetworkScenario:
    """Define different network conditions for testing"""
    
    SCENARIOS = {
        'normal': {
            'name': 'Normal Conditions',
            'base_capacity_kbps': 3000,
            'capacity_variation': 500,
            'initial_rate_kbps': 3500,
            'handover_min_s': 10,
            'handover_max_s': 16,
            'burst_loss_prob': 0.08,
            'base_loss_rate': 0.005,
            'prop_delay_ms': 35,
            'buffer_size_bytes': 150000,
            'cap_drop_factor': 0.65,
            'description': 'Typical LEO satellite network conditions'
        },
        'congested': {
            'name': 'Congested Network',
            'base_capacity_kbps': 2000,
            'capacity_variation': 400,
            'initial_rate_kbps': 2800,
            'handover_min_s': 10,
            'handover_max_s': 16,
            'burst_loss_prob': 0.10,
            'base_loss_rate': 0.008,
            'prop_delay_ms': 35,
            'buffer_size_bytes': 100000,
            'cap_drop_factor': 0.55,
            'description': 'Network under heavy load with limited capacity'
        },
        'degraded': {
            'name': 'Degraded Conditions',
            'base_capacity_kbps': 2500,
            'capacity_variation': 600,
            'initial_rate_kbps': 3000,
            'handover_min_s': 8,
            'handover_max_s': 14,
            'burst_loss_prob': 0.12,
            'base_loss_rate': 0.01,
            'prop_delay_ms': 40,
            'buffer_size_bytes': 120000,
            'cap_drop_factor': 0.50,
            'description': 'Adverse weather or interference conditions'
        },
        'heavy_load': {
            'name': 'Heavy Load / Stress Test',
            'base_capacity_kbps': 1500,
            'capacity_variation': 300,
            'initial_rate_kbps': 2500,
            'handover_min_s': 6,
            'handover_max_s': 10,
            'burst_loss_prob': 0.15,
            'base_loss_rate': 0.015,
            'prop_delay_ms': 45,
            'buffer_size_bytes': 80000,
            'cap_drop_factor': 0.45,
            'description': 'Extreme stress test with frequent handovers'
        },
        'variable': {
            'name': 'Highly Variable',
            'base_capacity_kbps': 2500,
            'capacity_variation': 1000,
            'initial_rate_kbps': 3000,
            'handover_min_s': 5,
            'handover_max_s': 20,
            'burst_loss_prob': 0.10,
            'base_loss_rate': 0.007,
            'prop_delay_ms': 35,
            'buffer_size_bytes': 125000,
            'cap_drop_factor': 0.60,
            'description': 'High variability in network conditions'
        }
    }
    
    @classmethod
    def get_scenario(cls, name: str) -> dict:
        """Get scenario configuration by name"""
        return cls.SCENARIOS.get(name, cls.SCENARIOS['normal'])


# =============================================================================
# Enhanced Simulation Engine
# =============================================================================

def run_realistic_simulation(
    N: int = 50000,
    scenario_name: str = 'normal',
    use_plc: bool = True,
    use_codel: bool = True,
    seed: int = 42,
    verbose: bool = True,
    debug_interval: int = 5000
) -> Tuple[Dict, Dict]:
    """
    Run realistic LEO network simulation
    
    Args:
        N: Number of simulation steps
        scenario_name: Network scenario to simulate
        use_plc: Enable PLC mechanism
        use_codel: Enable CoDel AQM
        seed: Random seed for reproducibility
        verbose: Print progress information
        debug_interval: Steps between debug prints
    
    Returns:
        Tuple of (metrics_dict, history_dict)
    """
    
    # Get scenario configuration
    scenario = NetworkScenario.get_scenario(scenario_name)
    
    if verbose:
        print(f"\n{'='*80}")
        print(f"Running: {'BBR+PLC+CoDel' if use_plc else 'BBR+CoDel'}")
        print(f"Scenario: {scenario['name']}")
        print(f"Description: {scenario['description']}")
        print(f"Steps: {N} ({N * leo.DT:.1f} seconds)")
        print(f"{'='*80}")
    
    # Initialize random number generator
    rng = np.random.default_rng(seed)
    
    # Create LEO handover profile with scenario parameters
    ho_params = leo.HandoverParams()
    ho_params.HANDOVER_INTERVAL_MIN = scenario['handover_min_s']
    ho_params.HANDOVER_INTERVAL_MAX = scenario['handover_max_s']
    ho_params.BURST_LOSS_PROB = scenario['burst_loss_prob']
    
    delay_profile, loss_profile, ho_indicator = leo.create_handover_profile(
        N, dt=leo.DT, params=ho_params, seed=seed
    )
    
    # Add base loss rate
    loss_profile = np.maximum(loss_profile, scenario['base_loss_rate'])
    
    # Generate capacity profile with realistic variations
    base_cap = scenario['base_capacity_kbps']
    cap_var = scenario['capacity_variation']
    
    # Multiple frequency components for realistic variation
    t = np.arange(N)
    capacity_profile = (
        base_cap +
        cap_var * 0.5 * np.sin(2 * np.pi * t / 2000) +  # Slow variation
        cap_var * 0.3 * np.sin(2 * np.pi * t / 500) +   # Medium variation
        cap_var * 0.2 * np.sin(2 * np.pi * t / 100)     # Fast variation
    )
    capacity_profile = np.maximum(capacity_profile, base_cap * 0.3)  # Minimum 30% of base
    
    # Initialize queue with realistic buffer size
    queue = EnhancedCoDelQueue(
        capacity_kbps=capacity_profile[0],
        max_buffer_bytes=scenario['buffer_size_bytes'],
        use_codel=use_codel,
        codel_target_ms=20.0,
        codel_interval_ms=100.0
    )
    
    # Initialize congestion control state
    cc_state = leo.CCState()
    cc_state.rate_kbps = scenario['initial_rate_kbps']
    cc_state.bw_est = scenario['initial_rate_kbps']
    cc_state.min_rtt = scenario['prop_delay_ms']
    cc_state.phase = "STARTUP"
    
    # Initialize PLC components
    plc_state = leo.PLCState()
    plc_params = leo.PLCParams()
    preho_cfg = leo.PreHOPredictParams()
    
    # History tracking
    history = {
        'time': [],
        'latency': [],
        'queue_delay': [],
        'prop_delay': [],
        'ho_delay': [],
        'sending_rate': [],
        'capacity': [],
        'handover': [],
        'goodput': [],
        'loss_rate': [],
        'queue_bytes': [],
        'queue_drops': [],
        'plc_multiplier': [],
        'cwnd': [],
        'rtt': [],
        'throughput': []
    }
    
    # Simulation state variables
    bytes_delivered_total = 0.0
    bytes_sent_total = 0.0
    cap_factor = 1.0
    cap_drop = scenario['cap_drop_factor']
    cap_ramp = 0.08
    
    # Main simulation loop
    for step in range(N):
        now_s = step * leo.DT
        
        # Current network conditions
        cur_ho_delay_ms = delay_profile[step]
        cur_loss_rate = loss_profile[step]
        is_handover = ho_indicator[step] == 1
        
        # Get current queue state
        queue_delay_ms = queue.get_queue_delay_ms()
        
        # Calculate delay components
        prop_delay_ms = scenario['prop_delay_ms']
        delay_comp = leo.DelayComponents(
            propagation_delay=prop_delay_ms,
            handover_delay=cur_ho_delay_ms,
            queue_delay=queue_delay_ms
        )
        
        # Handover prediction
        imminent_ho = leo.predict_handover_rtt(cc_state, delay_comp, preho_cfg)
        g_boost = preho_cfg.g_boost if imminent_ho else 1.0
        reserve_factor = max(0.0, 1.0 - preho_cfg.reserve_pct) if imminent_ho else 1.0
        
        # Capacity adjustment during handover
        if is_handover or imminent_ho:
            target_cap_factor = cap_drop
        else:
            target_cap_factor = 1.0
        
        cap_factor += cap_ramp * (target_cap_factor - cap_factor)
        cap_factor = np.clip(cap_factor, cap_drop, 1.0)
        
        current_capacity_kbps = capacity_profile[step] * cap_factor
        queue.set_capacity(current_capacity_kbps)
        
        # Total RTT calculation
        total_rtt_ms = prop_delay_ms + cur_ho_delay_ms + queue_delay_ms
        
        # Get base rate from congestion control
        base_rate_kbps = cc_state.rate_kbps
        
        # Apply PLC adjustment if enabled
        if use_plc:
            plc_mult, plc_state, _, _ = leo.plc_rate_adjustment(
                latency_ms=total_rtt_ms,
                p=plc_params,
                state=plc_state,
                use_predictive=True,
                use_self=True,
                g_external_boost=g_boost,
                is_handover=is_handover,
                bbr_phase=cc_state.phase
            )
            sending_rate_kbps = max(100.0, base_rate_kbps * plc_mult)
        else:
            plc_mult = 1.0
            sending_rate_kbps = base_rate_kbps
        
        # Apply reserve factor for handover preparation
        sending_rate_kbps *= reserve_factor
        
        # Calculate bytes to send this timestep
        send_bytes = sending_rate_kbps * 1000.0 * leo.DT / 8.0
        bytes_sent_total += send_bytes
        
        # Simulate packet loss
        delivered_bytes = 0.0
        if send_bytes > 0:
            packets = max(1, int(send_bytes / leo.PACKET_SIZE))
            lost_packets, _ = leo.apply_packet_loss_with_burst(
                packets_count=packets,
                loss_rate=cur_loss_rate,
                is_handover=is_handover,
                params=ho_params,
                rng=rng
            )
            delivered_packets = packets - lost_packets
            delivered_bytes = delivered_packets * leo.PACKET_SIZE
            delivered_bytes = min(send_bytes, delivered_bytes)
        
        bytes_delivered_total += delivered_bytes
        
        # Calculate instantaneous metrics
        instant_goodput_kbps = (delivered_bytes * 8.0) / (leo.DT * 1000.0)
        instant_throughput_kbps = (send_bytes * 8.0) / (leo.DT * 1000.0)
        
        # Queue operations
        queue_accepted = queue.enqueue(send_bytes, now_s)
        bytes_serviced, queue_delay_after_ms = queue.dequeue(leo.DT, now_s)
        
        # Update congestion control state
        cc_state = leo.update_congestion_control(
            rtt_ms=total_rtt_ms,
            bytes_acked=int(delivered_bytes),
            state=cc_state,
            baseline="bbr",
            step=step,
            queue_size_bytes=queue.queue_bytes,
            delay_comp=delay_comp,
            is_handover=is_handover,
            preho_cfg=preho_cfg,
            debug=(verbose and step % debug_interval == 0),
            debug_interval=debug_interval
        )
        
        # Record history
        history['time'].append(now_s)
        history['latency'].append(total_rtt_ms)
        history['queue_delay'].append(queue_delay_ms)
        history['prop_delay'].append(prop_delay_ms)
        history['ho_delay'].append(cur_ho_delay_ms)
        history['sending_rate'].append(sending_rate_kbps)
        history['capacity'].append(current_capacity_kbps)
        history['handover'].append(1 if is_handover else 0)
        history['goodput'].append(instant_goodput_kbps)
        history['loss_rate'].append(cur_loss_rate)
        history['queue_bytes'].append(queue.queue_bytes)
        history['queue_drops'].append(1 if not queue_accepted else 0)
        history['plc_multiplier'].append(plc_mult)
        history['cwnd'].append(cc_state.cwnd_gain if hasattr(cc_state, 'cwnd_gain') else 0)
        history['rtt'].append(total_rtt_ms)
        history['throughput'].append(instant_throughput_kbps)
        
        # Debug output
        if verbose and step % debug_interval == 0 and step > 0:
            recent_latency = np.mean(history['latency'][-1000:])
            recent_goodput = np.mean(history['goodput'][-1000:])
            print(f"  Step {step:6d}: RTT={recent_latency:6.1f}ms, "
                  f"Goodput={recent_goodput:6.1f}kbps, "
                  f"Queue={queue.queue_bytes:8.0f}B")
    
    # Calculate summary metrics
    sim_duration_s = N * leo.DT
    latency_array = np.array(history['latency'])
    queue_delay_array = np.array(history['queue_delay'])
    rate_array = np.array(history['sending_rate'])
    goodput_array = np.array(history['goodput'])
    
    # Get queue statistics
    queue_stats = queue.get_statistics()
    
    metrics = {
        # Identification
        'scenario': scenario_name,
        'use_plc': use_plc,
        'use_codel': use_codel,
        
        # Latency metrics (ms)
        'latency_mean': np.mean(latency_array),
        'latency_std': np.std(latency_array),
        'latency_min': np.min(latency_array),
        'latency_p25': np.percentile(latency_array, 25),
        'latency_p50': np.percentile(latency_array, 50),
        'latency_p75': np.percentile(latency_array, 75),
        'latency_p90': np.percentile(latency_array, 90),
        'latency_p95': np.percentile(latency_array, 95),
        'latency_p99': np.percentile(latency_array, 99),
        'latency_max': np.max(latency_array),
        
        # Queue metrics
        'queue_delay_mean': np.mean(queue_delay_array),
        'queue_delay_max': np.max(queue_delay_array),
        'queue_active_pct': (queue_delay_array > 0).mean() * 100,
        'queue_drops_total': queue_stats['dropped_packets'],
        'queue_codel_drops': queue_stats['codel_drops'],
        'queue_buffer_drops': queue_stats['buffer_drops'],
        
        # Throughput metrics (kbps)
        'sending_rate_mean': np.mean(rate_array),
        'sending_rate_std': np.std(rate_array),
        'goodput_mean': np.mean(goodput_array),
        'goodput_total': bytes_delivered_total * 8.0 / (sim_duration_s * 1000.0),
        'throughput_efficiency': (bytes_delivered_total / bytes_sent_total * 100) if bytes_sent_total > 0 else 0,
        
        # Compliance metrics (%)
        'compliance_50ms': (latency_array <= 50).mean() * 100,
        'compliance_100ms': (latency_array <= 100).mean() * 100,
        'compliance_150ms': (latency_array <= 150).mean() * 100,
        'compliance_200ms': (latency_array <= 200).mean() * 100,
        
        # Network events
        'handover_count': sum(history['handover']),
        'handover_rate': sum(history['handover']) / N * 100,
        
        # Simulation parameters
        'sim_duration_s': sim_duration_s,
        'sim_steps': N,
        'seed': seed
    }
    
    if verbose:
        print(f"\nSimulation Complete!")
        print(f"  Mean Latency: {metrics['latency_mean']:.1f}ms")
        print(f"  P99 Latency: {metrics['latency_p99']:.1f}ms")
        print(f"  Goodput: {metrics['goodput_mean']:.1f}kbps")
        print(f"  100ms Compliance: {metrics['compliance_100ms']:.1f}%")
    
    return metrics, history


# =============================================================================
# Visualization Functions
# =============================================================================

def plot_comprehensive_comparison(
    base_hist: Dict,
    plc_hist: Dict,
    scenario_name: str,
    save_path: Optional[str] = None
) -> plt.Figure:
    """Create comprehensive comparison plots"""
    
    scenario = NetworkScenario.get_scenario(scenario_name)
    
    # Create figure with custom layout
    fig = plt.figure(figsize=(16, 12))
    gs = gridspec.GridSpec(4, 3, hspace=0.3, wspace=0.3)
    
    # Color scheme
    colors = {
        'base': '#E74C3C',     # Red
        'plc': '#3498DB',       # Blue
        'capacity': '#95A5A6',  # Gray
        'handover': '#F39C12',  # Orange
        'target': '#27AE60'     # Green
    }
    
    time_s = np.array(base_hist['time'])
    
    # 1. Latency over time (main plot)
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(time_s, base_hist['latency'], label='BBR+CoDel', 
             color=colors['base'], alpha=0.7, linewidth=0.8)
    ax1.plot(time_s, plc_hist['latency'], label='BBR+PLC+CoDel', 
             color=colors['plc'], alpha=0.8, linewidth=0.8)
    
    # Mark handovers
    ho_indices = np.where(np.array(base_hist['handover']) == 1)[0]
    for idx in ho_indices[::5]:  # Show every 5th handover to avoid clutter
        ax1.axvspan(time_s[idx], time_s[min(idx+5, len(time_s)-1)], 
                   alpha=0.1, color=colors['handover'], zorder=0)
    
    ax1.axhline(y=100, color=colors['target'], linestyle='--', 
                alpha=0.5, label='100ms target')
    ax1.axhline(y=150, color=colors['handover'], linestyle='--', 
                alpha=0.5, label='150ms limit')
    ax1.set_ylabel('RTT (ms)', fontsize=10)
    ax1.set_title(f'End-to-End Latency - {scenario["name"]}', 
                  fontsize=12, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, time_s[-1]])
    
    # 2. Queue Delay
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(time_s, base_hist['queue_delay'], label='BASE', 
             color=colors['base'], alpha=0.6, linewidth=0.5)
    ax2.plot(time_s, plc_hist['queue_delay'], label='PLC', 
             color=colors['plc'], alpha=0.7, linewidth=0.5)
    ax2.set_ylabel('Queue Delay (ms)', fontsize=9)
    ax2.set_title('Queueing Delay', fontsize=10)
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # 3. Sending Rate
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(time_s, base_hist['capacity'], color=colors['capacity'], 
             alpha=0.3, linewidth=1, label='Capacity')
    ax3.plot(time_s, base_hist['sending_rate'], color=colors['base'], 
             alpha=0.6, linewidth=0.5, label='BASE Rate')
    ax3.plot(time_s, plc_hist['sending_rate'], color=colors['plc'], 
             alpha=0.7, linewidth=0.5, label='PLC Rate')
    ax3.set_ylabel('Rate (kbps)', fontsize=9)
    ax3.set_title('Sending Rate vs Capacity', fontsize=10)
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)
    
    # 4. PLC Multiplier
    ax4 = fig.add_subplot(gs[1, 2])
    ax4.plot(time_s, plc_hist['plc_multiplier'], color=colors['plc'], 
             alpha=0.7, linewidth=0.5)
    ax4.axhline(y=1.0, color='black', linestyle='--', alpha=0.3)
    ax4.set_ylabel('PLC Multiplier', fontsize=9)
    ax4.set_title('PLC Rate Adjustment', fontsize=10)
    ax4.grid(True, alpha=0.3)
    
    # 5. Latency Distribution
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.hist(base_hist['latency'], bins=50, alpha=0.5, color=colors['base'], 
             label='BASE', density=True)
    ax5.hist(plc_hist['latency'], bins=50, alpha=0.5, color=colors['plc'], 
             label='PLC', density=True)
    ax5.axvline(100, color=colors['target'], linestyle='--', alpha=0.5)
    ax5.set_xlabel('Latency (ms)', fontsize=9)
    ax5.set_ylabel('Density', fontsize=9)
    ax5.set_title('Latency Distribution', fontsize=10)
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)
    
    # 6. CDF Comparison
    ax6 = fig.add_subplot(gs[2, 1])
    base_sorted = np.sort(base_hist['latency'])
    plc_sorted = np.sort(plc_hist['latency'])
    cdf_y = np.arange(1, len(base_sorted) + 1) / len(base_sorted)
    ax6.plot(base_sorted, cdf_y, color=colors['base'], label='BASE', linewidth=2)
    ax6.plot(plc_sorted, cdf_y, color=colors['plc'], label='PLC', linewidth=2)
    ax6.axvline(100, color=colors['target'], linestyle='--', alpha=0.5)
    ax6.axvline(150, color=colors['handover'], linestyle='--', alpha=0.5)
    ax6.set_xlabel('Latency (ms)', fontsize=9)
    ax6.set_ylabel('CDF', fontsize=9)
    ax6.set_title('Latency CDF', fontsize=10)
    ax6.legend(fontsize=8)
    ax6.grid(True, alpha=0.3)
    ax6.set_xlim([0, 300])
    
    # 7. Goodput over time
    ax7 = fig.add_subplot(gs[2, 2])
    window_size = 100
    base_goodput_smooth = pd.Series(base_hist['goodput']).rolling(window_size).mean()
    plc_goodput_smooth = pd.Series(plc_hist['goodput']).rolling(window_size).mean()
    ax7.plot(time_s, base_goodput_smooth, color=colors['base'], 
             alpha=0.7, linewidth=1, label='BASE')
    ax7.plot(time_s, plc_goodput_smooth, color=colors['plc'], 
             alpha=0.8, linewidth=1, label='PLC')
    ax7.set_ylabel('Goodput (kbps)', fontsize=9)
    ax7.set_title('Smoothed Goodput (100-step MA)', fontsize=10)
    ax7.legend(fontsize=8)
    ax7.grid(True, alpha=0.3)
    
    # 8. Queue Bytes
    ax8 = fig.add_subplot(gs[3, 0])
    ax8.plot(time_s, np.array(base_hist['queue_bytes'])/1000, 
             color=colors['base'], alpha=0.6, linewidth=0.5, label='BASE')
    ax8.plot(time_s, np.array(plc_hist['queue_bytes'])/1000, 
             color=colors['plc'], alpha=0.7, linewidth=0.5, label='PLC')
    ax8.set_ylabel('Queue (KB)', fontsize=9)
    ax8.set_xlabel('Time (s)', fontsize=9)
    ax8.set_title('Queue Occupancy', fontsize=10)
    ax8.legend(fontsize=8)
    ax8.grid(True, alpha=0.3)
    
    # 9. Loss Rate
    ax9 = fig.add_subplot(gs[3, 1])
    ax9.plot(time_s, np.array(base_hist['loss_rate'])*100, 
             color='darkred', alpha=0.6, linewidth=0.8)
    ax9.fill_between(time_s, 0, np.array(base_hist['loss_rate'])*100, 
                     color='darkred', alpha=0.2)
    ax9.set_ylabel('Loss Rate (%)', fontsize=9)
    ax9.set_xlabel('Time (s)', fontsize=9)
    ax9.set_title('Packet Loss Rate', fontsize=10)
    ax9.grid(True, alpha=0.3)
    
    # 10. Performance Summary Box
    ax10 = fig.add_subplot(gs[3, 2])
    ax10.axis('off')
    
    # Calculate improvements
    base_p99 = np.percentile(base_hist['latency'], 99)
    plc_p99 = np.percentile(plc_hist['latency'], 99)
    base_mean = np.mean(base_hist['latency'])
    plc_mean = np.mean(plc_hist['latency'])
    
    improvement_text = f"""
Performance Improvement:
━━━━━━━━━━━━━━━━━━━━
Mean Latency:
  BASE: {base_mean:.1f}ms
  PLC:  {plc_mean:.1f}ms
  Improvement: {((base_mean-plc_mean)/base_mean*100):.1f}%

P99 Latency:
  BASE: {base_p99:.1f}ms
  PLC:  {plc_p99:.1f}ms
  Improvement: {((base_p99-plc_p99)/base_p99*100):.1f}%

100ms Compliance:
  BASE: {(np.array(base_hist['latency'])<=100).mean()*100:.1f}%
  PLC:  {(np.array(plc_hist['latency'])<=100).mean()*100:.1f}%
"""
    ax10.text(0.1, 0.5, improvement_text, fontsize=9, 
             family='monospace', verticalalignment='center')
    
    # Main title
    fig.suptitle(f'BBR+CoDel vs BBR+PLC+CoDel - {scenario["name"]}', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Plot saved to {save_path}")
    
    return fig


def generate_report(
    results: Dict[str, Dict],
    output_dir: str = "results"
) -> None:
    """Generate comprehensive test report"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Create summary DataFrame
    summary_data = []
    for key, result in results.items():
        scenario, method = key.rsplit('_', 1)
        metrics = result['metrics']
        row = {
            'Scenario': scenario,
            'Method': method,
            'Mean Latency (ms)': metrics['latency_mean'],
            'P99 Latency (ms)': metrics['latency_p99'],
            'Queue Delay (ms)': metrics['queue_delay_mean'],
            'Goodput (kbps)': metrics['goodput_mean'],
            '100ms Compliance (%)': metrics['compliance_100ms'],
            '150ms Compliance (%)': metrics['compliance_150ms']
        }
        summary_data.append(row)
    
    summary_df = pd.DataFrame(summary_data)
    
    # Save to CSV
    csv_path = os.path.join(output_dir, 'test_results.csv')
    summary_df.to_csv(csv_path, index=False)
    print(f"\nResults saved to {csv_path}")
    
    # Generate HTML report
    html_content = f"""
    <html>
    <head>
        <title>LEO Network Test Results</title>
        <style>
            body {{ font-family: Arial, sans-serif; margin: 20px; }}
            h1 {{ color: #2c3e50; }}
            table {{ border-collapse: collapse; width: 100%; }}
            th, td {{ border: 1px solid #ddd; padding: 8px; text-align: right; }}
            th {{ background-color: #3498db; color: white; }}
            tr:nth-child(even) {{ background-color: #f2f2f2; }}
            .improvement {{ color: green; font-weight: bold; }}
            .degradation {{ color: red; }}
        </style>
    </head>
    <body>
        <h1>LEO Network Simulation Test Results</h1>
        <p>Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
        
        <h2>Summary Results</h2>
        {summary_df.to_html(index=False, float_format=lambda x: f'{x:.2f}')}
        
        <h2>Improvement Analysis</h2>
        <table>
            <tr>
                <th>Scenario</th>
                <th>Mean Latency Reduction (%)</th>
                <th>P99 Latency Reduction (%)</th>
                <th>Queue Delay Reduction (%)</th>
                <th>100ms Compliance Gain (pp)</th>
            </tr>
    """
    
    # Calculate improvements for each scenario
    for scenario in ['normal', 'congested', 'degraded', 'heavy_load', 'variable']:
        if f'{scenario}_base' in results and f'{scenario}_plc' in results:
            base = results[f'{scenario}_base']['metrics']
            plc = results[f'{scenario}_plc']['metrics']
            
            lat_imp = ((base['latency_mean'] - plc['latency_mean']) / base['latency_mean']) * 100
            p99_imp = ((base['latency_p99'] - plc['latency_p99']) / base['latency_p99']) * 100
            queue_imp = ((base['queue_delay_mean'] - plc['queue_delay_mean']) / 
                        base['queue_delay_mean']) * 100 if base['queue_delay_mean'] > 0 else 0
            comp_gain = plc['compliance_100ms'] - base['compliance_100ms']
            
            html_content += f"""
            <tr>
                <td>{scenario.title()}</td>
                <td class="{'improvement' if lat_imp > 0 else 'degradation'}">{lat_imp:.1f}</td>
                <td class="{'improvement' if p99_imp > 0 else 'degradation'}">{p99_imp:.1f}</td>
                <td class="{'improvement' if queue_imp > 0 else 'degradation'}">{queue_imp:.1f}</td>
                <td class="{'improvement' if comp_gain > 0 else 'degradation'}">{comp_gain:.1f}</td>
            </tr>
            """
    
    html_content += """
        </table>
    </body>
    </html>
    """
    
    html_path = os.path.join(output_dir, 'test_report.html')
    with open(html_path, 'w') as f:
        f.write(html_content)
    print(f"HTML report saved to {html_path}")


# =============================================================================
# Main Test Suite
# =============================================================================

def run_test_suite(
    scenarios: List[str] = None,
    N: int = 10000,
    seeds: List[int] = None,
    output_dir: str = "results",
    generate_plots: bool = True,
    verbose: bool = True
) -> Dict:
    """
    Run complete test suite comparing BBR+CoDel vs BBR+PLC+CoDel
    
    Args:
        scenarios: List of scenario names to test
        N: Number of simulation steps
        seeds: List of random seeds for multiple runs
        output_dir: Directory for output files
        generate_plots: Whether to generate visualization plots
        verbose: Print detailed progress
    
    Returns:
        Dictionary with all test results
    """
    
    if scenarios is None:
        scenarios = ['normal', 'congested', 'degraded']
    
    if seeds is None:
        seeds = [42]  # Single seed by default
    
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"{'='*80}")
    print("LEO NETWORK SIMULATION TEST SUITE")
    print(f"{'='*80}")
    print(f"Scenarios: {', '.join(scenarios)}")
    print(f"Steps per test: {N} ({N*leo.DT:.1f} seconds)")
    print(f"Random seeds: {seeds}")
    print(f"Output directory: {output_dir}")
    print(f"{'='*80}\n")
    
    all_results = {}
    
    for scenario_name in scenarios:
        print(f"\n{'─'*60}")
        print(f"TESTING SCENARIO: {scenario_name.upper()}")
        print(f"{'─'*60}")
        
        # Run multiple seeds and average results
        base_metrics_list = []
        plc_metrics_list = []
        
        for seed in seeds:
            print(f"\n  Seed {seed}:")
            
            # Run BASE (BBR+CoDel)
            base_metrics, base_hist = run_realistic_simulation(
                N=N,
                scenario_name=scenario_name,
                use_plc=False,
                use_codel=True,
                seed=seed,
                verbose=verbose
            )
            base_metrics_list.append(base_metrics)
            
            # Run PLC (BBR+PLC+CoDel)
            plc_metrics, plc_hist = run_realistic_simulation(
                N=N,
                scenario_name=scenario_name,
                use_plc=True,
                use_codel=True,
                seed=seed,
                verbose=verbose
            )
            plc_metrics_list.append(plc_metrics)
        
        # Store results (using last seed's history for plotting)
        all_results[f'{scenario_name}_base'] = {
            'metrics': base_metrics_list[-1] if len(seeds) == 1 else 
                      {k: np.mean([m[k] for m in base_metrics_list]) 
                       for k in base_metrics_list[0].keys()},
            'history': base_hist
        }
        
        all_results[f'{scenario_name}_plc'] = {
            'metrics': plc_metrics_list[-1] if len(seeds) == 1 else
                      {k: np.mean([m[k] for m in plc_metrics_list])
                       for k in plc_metrics_list[0].keys()},
            'history': plc_hist
        }
        
        # Generate comparison plot for this scenario
        if generate_plots:
            plot_path = os.path.join(output_dir, f'comparison_{scenario_name}.png')
            fig = plot_comprehensive_comparison(
                base_hist, plc_hist, scenario_name, save_path=plot_path
            )
            plt.close(fig)
        
        # Print summary for this scenario
        base_final = all_results[f'{scenario_name}_base']['metrics']
        plc_final = all_results[f'{scenario_name}_plc']['metrics']
        
        print(f"\n  Summary for {scenario_name}:")
        print(f"    BASE - Mean RTT: {base_final['latency_mean']:.1f}ms, "
              f"P99: {base_final['latency_p99']:.1f}ms, "
              f"100ms: {base_final['compliance_100ms']:.1f}%")
        print(f"    PLC  - Mean RTT: {plc_final['latency_mean']:.1f}ms, "
              f"P99: {plc_final['latency_p99']:.1f}ms, "
              f"100ms: {plc_final['compliance_100ms']:.1f}%")
        
        improvement = ((base_final['latency_mean'] - plc_final['latency_mean']) / 
                      base_final['latency_mean'] * 100)
        print(f"    Improvement: {improvement:.1f}%")
    
    # Generate final report
    generate_report(all_results, output_dir)
    
    # Save raw results as JSON
    json_path = os.path.join(output_dir, 'raw_results.json')
    json_data = {k: v['metrics'] for k, v in all_results.items()}
    with open(json_path, 'w') as f:
        json.dump(json_data, f, indent=2)
    print(f"\nRaw results saved to {json_path}")
    
    print(f"\n{'='*80}")
    print("TEST SUITE COMPLETED")
    print(f"{'='*80}")
    
    return all_results


# =============================================================================
# Command Line Interface
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Test BBR+CoDel vs BBR+PLC+CoDel in realistic LEO network conditions"
    )
    
    parser.add_argument(
        '--scenarios', 
        nargs='+',
        choices=list(NetworkScenario.SCENARIOS.keys()),
        default=['normal', 'congested', 'degraded'],
        help='Network scenarios to test'
    )
    
    parser.add_argument(
        '--steps', '-N',
        type=int,
        default=10000,
        help='Number of simulation steps (default: 10000)'
    )
    
    parser.add_argument(
        '--seeds',
        nargs='+',
        type=int,
        default=[1],
        help='Random seeds for reproducibility'
    )
    
    parser.add_argument(
        '--output-dir',
        type=str,
        default='results',
        help='Directory for output files'
    )
    
    parser.add_argument(
        '--no-plots',
        action='store_true',
        help='Disable plot generation'
    )
    
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Minimal output'
    )
    
    parser.add_argument(
        '--quick',
        action='store_true',
        help='Quick test with N=5000 and single scenario'
    )
    
    args = parser.parse_args()
    
    # Quick test mode
    if args.quick:
        args.steps = 5000
        args.scenarios = ['normal']
        print("Running quick test (N=5000, normal scenario only)")
    
    # Run test suite
    results = run_test_suite(
        scenarios=args.scenarios,
        N=args.steps,
        seeds=args.seeds,
        output_dir=args.output_dir,
        generate_plots=not args.no_plots,
        verbose=not args.quiet
    )
    
    # Print final summary
    if not args.quiet:
        print("\nFinal Summary:")
        print("─" * 60)
        for scenario in args.scenarios:
            if f'{scenario}_base' in results and f'{scenario}_plc' in results:
                base = results[f'{scenario}_base']['metrics']
                plc = results[f'{scenario}_plc']['metrics']
                improvement = ((base['latency_mean'] - plc['latency_mean']) / 
                             base['latency_mean'] * 100)
                print(f"{scenario:12s}: {improvement:+6.1f}% latency reduction with PLC")


if __name__ == "__main__":
    main()
