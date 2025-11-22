#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_multi_flow_v1_0.py
==================================================================
Asymmetric 2-Flow Test to Verify Per-Flow Queue Independence
==================================================================
LEO Satellite Network Simulator with Proactive Latency Control (PLC)

Implements dual-loop supervisory control for latency stabilization
in Low Earth Orbit (LEO) satellite networks under predictable
handover disruptions.

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

ASYMMETRY METHODS:
1. Staggered start (Flow2 starts 10s later)
2. Different PLC configurations
3. Different initial rates
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import LEO_PLC_Final_rule_preho_v2_0 as leo
from collections import deque


def jain_fairness(x):
    x = np.array(x, dtype=float)
    if len(x) == 0:
        return 1.0
    num = (x.sum()) ** 2
    den = len(x) * (x ** 2).sum()
    if den == 0:
        return 1.0
    return num / den


def plot_multiflow_latency_comparison(
    flow1,
    flow2,
    N,
    scenario_name="Multi-Flow Comparison",
    save_path=None,
    baseline="BBR"
):
    """
    Generate Figure 4.1-style multi-flow latency visualization
    
    Args:
        flow1: First Flow object with latency_history
        flow2: Second Flow object with latency_history
        N: Total simulation steps
        scenario_name: Title for the plot
        save_path: Path to save the figure (optional)
        baseline: Name of baseline controller
    """
    # ✅ FIX: Create separate time arrays for each flow based on their actual history length
    time_s1 = np.arange(len(flow1.latency_history)) * leo.DT
    time_s2 = np.arange(len(flow2.latency_history)) * leo.DT
    
    # ✅ FIX: Calculate offset for flow2 if it started later (staggered start)
    time_offset2 = (N - len(flow2.latency_history)) * leo.DT
    time_s2 = time_s2 + time_offset2  # Shift flow2 time axis
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle(f'{scenario_name}', fontsize=16, fontweight='bold')
    
    # Top panel: Flow 1
    ax1.plot(time_s1, flow1.latency_history, 
             color='#D55E00', linewidth=0.8, alpha=0.7, label=f'{flow1.name} ({baseline})')
    ax1.axhline(y=100, color='red', linestyle='--', linewidth=1.5, 
                alpha=0.6, label='100ms threshold')
    ax1.set_ylabel('E2E Latency (ms)', fontsize=12, fontweight='bold')
    ax1.set_title(f'{flow1.name}: {"PLC Enabled" if flow1.use_plc else "Baseline Only"}',
                  fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3, linestyle=':', linewidth=0.5)
    ax1.set_ylim(bottom=0)
    
    # Calculate and display statistics
    mean_lat1 = np.mean(flow1.latency_history)
    p99_lat1 = np.percentile(flow1.latency_history, 99)
    max_lat1 = np.max(flow1.latency_history)
    
    stats_text1 = f'Mean: {mean_lat1:.1f}ms | P99: {p99_lat1:.1f}ms | Max: {max_lat1:.1f}ms'
    ax1.text(0.02, 0.95, stats_text1, transform=ax1.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Bottom panel: Flow 2
    ax2.plot(time_s2, flow2.latency_history,  # ✅ FIX: Use time_s2 (offset)
             color='#009E73', linewidth=0.8, alpha=0.7, label=f'{flow2.name} ({baseline})')
    ax2.axhline(y=100, color='red', linestyle='--', linewidth=1.5,
                alpha=0.6, label='100ms threshold')
    ax2.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('E2E Latency (ms)', fontsize=12, fontweight='bold')
    ax2.set_title(f'{flow2.name}: {"PLC Enabled" if flow2.use_plc else "Baseline Only"}',
                  fontsize=13, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3, linestyle=':', linewidth=0.5)
    ax2.set_ylim(bottom=0)
    
    # Calculate and display statistics
    mean_lat2 = np.mean(flow2.latency_history) if len(flow2.latency_history) > 0 else 0
    p99_lat2 = np.percentile(flow2.latency_history, 99) if len(flow2.latency_history) > 0 else 0
    max_lat2 = np.max(flow2.latency_history) if len(flow2.latency_history) > 0 else 0
    
    stats_text2 = f'Mean: {mean_lat2:.1f}ms | P99: {p99_lat2:.1f}ms | Max: {max_lat2:.1f}ms'
    ax2.text(0.02, 0.95, stats_text2, transform=ax2.transAxes,
             fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    # ✅ FIX: Add visual indicator for staggered start
    if time_offset2 > 0:
        ax2.axvline(x=time_offset2, color='gray', linestyle=':', linewidth=2, 
                    alpha=0.6, label=f'Flow2 Start (t={time_offset2:.1f}s)')
        ax2.legend(loc='upper right', fontsize=10)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"  → Figure saved to: {save_path}")
    
    plt.show()
    plt.close()



class Flow:
    def __init__(self, name, baseline="bbr", use_plc=True, seed_offset=0, initial_rate_kbps=500.0):
        self.name = name
        self.baseline = baseline
        self.use_plc = use_plc
        self.rng = np.random.default_rng(42 + seed_offset)
        
        self.cc_state = leo.CCState()
        self.cc_state.rate_kbps = initial_rate_kbps  # ✅ NEW: Different initial rate
        
        self.plc_params = leo.PLCParams()
        self.plc_state = leo.PLCState() if use_plc else None

        self.bytes_delivered = 0.0
        self.last_want_kbps = 0.0
        
        self.latency_history = []
        self.rate_history = []
        self.queue_delay_history = []
        self.active = False  # ✅ NEW: Staggered start support

    def decide_rate(self, latency_ms, is_handover, g_boost=1.0):
        if not self.active:
            return 0.0
            
        base_rate = self.cc_state.rate_kbps

        if not self.use_plc:
            want = base_rate
        else:
            rate_mult, self.plc_state, _, _ = leo.plc_rate_adjustment(
                latency_ms=latency_ms,
                p=self.plc_params,
                state=self.plc_state,
                use_predictive=True,
                use_self=True,
                g_external_boost=g_boost,
                is_handover=is_handover,
                bbr_phase=self.cc_state.phase if self.baseline == "bbr" else "N/A",
            )
            want = base_rate * rate_mult

        self.last_want_kbps = max(100.0, want)
        self.rate_history.append(self.last_want_kbps)
        return self.last_want_kbps

    def apply_feedback(
        self,
        delivered_bytes,
        latency_ms,
        step,
        queue_bytes,
        delay_comp,
        is_handover,
        preho_cfg,
    ):
        if not self.active:
            return
            
        bytes_acked = int(delivered_bytes)

        self.cc_state = leo.update_congestion_control(
            rtt_ms=latency_ms,
            bytes_acked=bytes_acked,
            state=self.cc_state,
            baseline=self.baseline,
            step=step,
            queue_size_bytes=queue_bytes,
            delay_comp=delay_comp,
            is_handover=is_handover,
            preho_cfg=preho_cfg,
            debug=False,
            debug_interval=5000,
        )

        self.bytes_delivered += delivered_bytes
        self.latency_history.append(latency_ms)
        self.queue_delay_history.append(delay_comp.queue_delay)

    def apply_loss(self, bytes_to_send, loss_rate, is_handover, params):
        if bytes_to_send <= 0:
            return 0.0
        
        packets = max(1, int(bytes_to_send / leo.PACKET_SIZE))
        lost, _ = leo.apply_packet_loss_with_burst(
            packets_count=packets,
            loss_rate=loss_rate,
            is_handover=is_handover,
            params=params,
            rng=self.rng,
        )
        delivered_packets = packets - lost
        delivered_bytes = delivered_packets * leo.PACKET_SIZE
        return min(bytes_to_send, delivered_bytes)


class Packet:
    __slots__ = ("size_bytes", "enqueue_time_s")

    def __init__(self, size_bytes: float, enqueue_time_s: float):
        self.size_bytes = float(size_bytes)
        self.enqueue_time_s = float(enqueue_time_s)


class PerFlowQueue:
    """
    Per-flow FIFO queue with optional CoDel AQM (RFC 8289-style, head-packet based).

    - Queue model: FIFO in bytes, but we track arrival time per "packet batch".
    - CoDel state is per-queue and uses sojourn time of the head packet.
    """
    def __init__(
        self,
        capacity_share_kbps: float,
        use_codel: bool = False,
        codel_target_ms: float = 20.0,
        codel_interval_ms: float = 100.0,
    ):
        self.capacity_share_kbps = capacity_share_kbps
        self.queue_bytes = 0.0

        # Internal FIFO of Packet objects
        self._queue = deque()

        # CoDel configuration (converted to seconds)
        self.use_codel = use_codel
        self.codel_target_s = codel_target_ms / 1000.0
        self.codel_interval_s = codel_interval_ms / 1000.0

        # CoDel state (RFC 8289 style)
        self._first_above_time_s = 0.0
        self._dropping = False
        self._drop_next_s = 0.0
        self._drop_count = 0

    # ------------------------------------------------------------------
    # Basic helpers
    # ------------------------------------------------------------------
    def _service_bytes_per_second(self) -> float:
        if self.capacity_share_kbps <= 0:
            return 0.0
        return self.capacity_share_kbps * 1000.0 / 8.0

    def _current_queue_delay_ms(self) -> float:
        srv_bps = self._service_bytes_per_second()
        if srv_bps <= 0 or self.queue_bytes <= 0:
            return 0.0
        return (self.queue_bytes / srv_bps) * 1000.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def enqueue(self, bytes_in: float, now_s: float) -> None:
        """
        Enqueue 'bytes_in' bytes that arrive at simulation time 'now_s'.

        In this simulator, all bytes produced in one timestep for a flow
        are treated as a single "packet batch" with one timestamp.
        """
        if bytes_in <= 0:
            return
        self._queue.append(Packet(size_bytes=bytes_in, enqueue_time_s=now_s))
        self.queue_bytes += float(bytes_in)

    def _codel_should_drop_head(self, now_s: float) -> bool:
        """
        Decide whether the *head* packet should be dropped (CoDel).

        This is a head-packet implementation approximating RFC 8289:

        - If sojourn time of head packet has stayed above 'target' for at least
          'interval', we enter dropping state.
        - In dropping state, we drop packets according to the sqrt control law.
        """
        if not self._queue:
            # Empty queue resets CoDel "first_above_time" (RFC 8289 §4.1).
            self._first_above_time_s = 0.0
            return False

        head = self._queue[0]
        sojourn_s = now_s - head.enqueue_time_s

        if not self._dropping:
            # Not currently dropping.
            if sojourn_s < self.codel_target_s:
                # Below target: queue is deemed fine.
                self._first_above_time_s = 0.0
                return False

            # Above target.
            if self._first_above_time_s == 0.0:
                # First time we see above-target sojourn.
                self._first_above_time_s = now_s + self.codel_interval_s
                return False

            if now_s < self._first_above_time_s:
                # Still within the "observation interval".
                return False

            # Sojourn has been above target for at least one interval:
            # enter dropping state.
            self._dropping = True
            self._drop_count = 1
            self._drop_next_s = now_s + self.codel_interval_s / math.sqrt(self._drop_count)
            return True  # Drop this head packet
        else:
            # Already in dropping state.
            if sojourn_s < self.codel_target_s:
                # Queue has recovered → leave dropping state.
                self._dropping = False
                self._first_above_time_s = 0.0
                return False

            # Still above target and in dropping state.
            if now_s >= self._drop_next_s:
                # Time to drop next packet according to CoDel control law.
                self._drop_count += 1
                self._drop_next_s = now_s + self.codel_interval_s / math.sqrt(self._drop_count)
                return True

        return False

    def dequeue(self, dt: float, now_s: float):
        """
        Dequeue up to capacity_share_kbps * dt bytes.

        Returns:
            bytes_sent: total bytes successfully served this timestep
            queue_delay_ms_after: queueing delay after service (ms)
        """
        srv_bps = self._service_bytes_per_second()
        if srv_bps <= 0:
            # No service this step (e.g., capacity_share_kbps == 0)
            return 0.0, self._current_queue_delay_ms()

        capacity_bytes = srv_bps * dt
        bytes_sent = 0.0

        # 1) Apply CoDel dropping to head packets (if enabled)
        if self.use_codel:
            # Drop head packets as long as CoDel says "drop" and queue is non-empty.
            # This corresponds to RFC 8289's "while in dropping state" handling,
            # but adapted to discrete timesteps.
            while self._queue and self._codel_should_drop_head(now_s):
                dropped_bytes = self._queue.popleft().size_bytes
                self.queue_bytes = max(0.0, self.queue_bytes - dropped_bytes)

        # 2) Serve bytes up to capacity (FIFO)
        while self._queue and capacity_bytes > 0.0:
            head = self._queue[0]
            if head.size_bytes <= capacity_bytes:
                # Send whole packet batch
                send_bytes = head.size_bytes
                capacity_bytes -= send_bytes
                bytes_sent += send_bytes
                self.queue_bytes = max(0.0, self.queue_bytes - send_bytes)
                self._queue.popleft()
            else:
                # Partial service of this batch
                send_bytes = capacity_bytes
                head.size_bytes -= send_bytes
                capacity_bytes = 0.0
                bytes_sent += send_bytes
                self.queue_bytes = max(0.0, self.queue_bytes - send_bytes)

        return bytes_sent, self._current_queue_delay_ms()

    def get_queue_delay_ms(self) -> float:
        """
        Return the current queueing delay estimate in milliseconds,
        based on bytes in queue and the assigned per-flow capacity.
        """
        return self._current_queue_delay_ms()


def run_asymmetric_two_flow_sim(
    N=50000,
    seed=1,
    baseline="bbr",
    include_handover=True,
    cap_drop=0.55,
    cap_ramp=0.08,
    use_rtt_rule=True,
    stagger_delay_steps=1000,  # Flow2 starts 10s later
    flow1_initial_rate=300.0,   # Different initial rates
    flow2_initial_rate=700.0,
    flow1_plc=True,             # Asymmetric PLC
    flow2_plc=True,
    out_print=True,
    # ✅ CoDel AQM option
    use_codel=False,
    codel_target_ms=20.0,
    codel_interval_ms=100.0,
):
    """
    Asymmetric 2-flow simulation to break synchronization
    """
    rng = np.random.default_rng(seed)

    if include_handover:
        delay_profile, loss_profile, ho_indicator = leo.create_handover_profile(
            N,
            dt=leo.DT,
            params=leo.HandoverParams(),
            seed=seed,
        )
    else:
        delay_profile = np.zeros(N)
        loss_profile = np.ones(N) * leo.BASE_LOSS_RATE
        ho_indicator = np.zeros(N, dtype=np.int8)

    time_arr = np.arange(N) * leo.DT
    base_capacity = 1500.0 + 300.0 * np.sin(2 * math.pi * time_arr / 60.0)

    preho_cfg = leo.PreHOPredictParams(
        lookahead_steps=5,
        g_boost=1.5,
        reserve_pct=0.20,
        rtt_spike_threshold=1.20,
        rtt_alpha_short=0.3,
        rtt_alpha_long=0.05,
        use_congestion_filter=True,
    )

    # ✅ NEW: Different initial rates
    f1 = Flow("flow1", baseline=baseline, use_plc=flow1_plc, seed_offset=0, 
              initial_rate_kbps=flow1_initial_rate)
    f2 = Flow("flow2", baseline=baseline, use_plc=flow2_plc, seed_offset=1000,
              initial_rate_kbps=flow2_initial_rate)

    # Flow1 starts immediately, Flow2 starts later
    f1.active = True
    f2.active = False

    queue1 = PerFlowQueue(
        capacity_share_kbps=750.0,
        use_codel=use_codel,
        codel_target_ms=codel_target_ms,
        codel_interval_ms=codel_interval_ms,
    )
    queue2 = PerFlowQueue(
        capacity_share_kbps=750.0,
        use_codel=use_codel,
        codel_target_ms=codel_target_ms,
        codel_interval_ms=codel_interval_ms,
    )
    cap_factor = 1.0

    latency_diffs = []
    queue_delay_diffs = []

    for step in range(N):
        now_s = step * leo.DT
        # ✅ NEW: Activate Flow2 after stagger delay
        if step == stagger_delay_steps:
            f2.active = True
            print(f"[Step {step}] Flow2 ACTIVATED (staggered start)")
        
        cur_delay_ms = delay_profile[step]
        cur_loss_rate = loss_profile[step]
        is_ho = ho_indicator[step] == 1

        base_prop_ms = (
            leo.NETWORK_DELAY_BASE + leo.CODEC_DELAY + leo.RENDER_DELAY
        )

        cur_capacity_kbps = base_capacity[step] * cap_factor

        queue_delay1_ms = queue1.get_queue_delay_ms()
        queue_delay2_ms = queue2.get_queue_delay_ms()

        delay_comp1 = leo.DelayComponents(
            propagation_delay=base_prop_ms,
            handover_delay=cur_delay_ms,
            queue_delay=queue_delay1_ms,
        )
        delay_comp2 = leo.DelayComponents(
            propagation_delay=base_prop_ms,
            handover_delay=cur_delay_ms,
            queue_delay=queue_delay2_ms,
        )

        if use_rtt_rule:
            imminent1 = leo.predict_handover_rtt(
                f1.cc_state, delay_comp1, preho_cfg
            ) if f1.active else False
            imminent2 = leo.predict_handover_rtt(
                f2.cc_state, delay_comp2, preho_cfg
            ) if f2.active else False
        else:
            imminent1 = leo.predict_handover_rule(
                step, ho_indicator, preho_cfg.lookahead_steps
            ) if f1.active else False
            imminent2 = imminent1 if f2.active else False

        g_boost1 = preho_cfg.g_boost if imminent1 else 1.0
        g_boost2 = preho_cfg.g_boost if imminent2 else 1.0
        
        reserve_factor1 = max(0.0, 1.0 - preho_cfg.reserve_pct) if imminent1 else 1.0
        reserve_factor2 = max(0.0, 1.0 - preho_cfg.reserve_pct) if imminent2 else 1.0

        imminent_any = imminent1 or imminent2
        target_cap = cap_drop if (is_ho or imminent_any) else 1.0
        cap_factor = cap_factor + cap_ramp * (target_cap - cap_factor)
        cur_capacity_kbps = base_capacity[step] * cap_factor

        total_latency1_ms = base_prop_ms + cur_delay_ms + queue_delay1_ms
        total_latency2_ms = base_prop_ms + cur_delay_ms + queue_delay2_ms

        want1_kbps = f1.decide_rate(
            latency_ms=total_latency1_ms,
            is_handover=is_ho,
            g_boost=g_boost1,
        )
        want2_kbps = f2.decide_rate(
            latency_ms=total_latency2_ms,
            is_handover=is_ho,
            g_boost=g_boost2,
        )

        total_want = want1_kbps + want2_kbps

        if total_want <= cur_capacity_kbps:
            send1_kbps = want1_kbps
            send2_kbps = want2_kbps
        else:
            if total_want > 0:
                send1_kbps = cur_capacity_kbps * (want1_kbps / total_want)
                send2_kbps = cur_capacity_kbps * (want2_kbps / total_want)
            else:
                send1_kbps = 0.0
                send2_kbps = 0.0

        send1_kbps *= reserve_factor1
        send2_kbps *= reserve_factor2

        send1_bytes = send1_kbps * 1000.0 * leo.DT / 8.0
        send2_bytes = send2_kbps * 1000.0 * leo.DT / 8.0

        ho_params = leo.HandoverParams()
        delivered1_bytes = f1.apply_loss(send1_bytes, cur_loss_rate, is_ho, ho_params)
        delivered2_bytes = f2.apply_loss(send2_bytes, cur_loss_rate, is_ho, ho_params)

        queue1.enqueue(send1_bytes, now_s)
        queue2.enqueue(send2_bytes, now_s)

        # Active flows share capacity equally
        active_flows = sum([f1.active, f2.active])
        if active_flows > 0:
            queue1.capacity_share_kbps = cur_capacity_kbps / active_flows if f1.active else 0
            queue2.capacity_share_kbps = cur_capacity_kbps / active_flows if f2.active else 0
        else:
            queue1.capacity_share_kbps = 0
            queue2.capacity_share_kbps = 0

        _, queue_delay1_ms = queue1.dequeue(leo.DT, now_s)
        _, queue_delay2_ms = queue2.dequeue(leo.DT, now_s)

        f1.apply_feedback(
            delivered_bytes=delivered1_bytes,
            latency_ms=total_latency1_ms,
            step=step,
            queue_bytes=queue1.queue_bytes,
            delay_comp=delay_comp1,
            is_handover=is_ho,
            preho_cfg=preho_cfg,
        )
        f2.apply_feedback(
            delivered_bytes=delivered2_bytes,
            latency_ms=total_latency2_ms,
            step=step,
            queue_bytes=queue2.queue_bytes,
            delay_comp=delay_comp2,
            is_handover=is_ho,
            preho_cfg=preho_cfg,
        )

        latency_diffs.append(abs(total_latency1_ms - total_latency2_ms))
        queue_delay_diffs.append(abs(queue_delay1_ms - queue_delay2_ms))

    sim_dur_sec = N * leo.DT
    
    # Account for stagger delay in Flow2's throughput calculation
    f1_active_time = sim_dur_sec
    f2_active_time = sim_dur_sec - (stagger_delay_steps * leo.DT)
    
    thr1_bps = (f1.bytes_delivered * 8.0) / f1_active_time
    thr2_bps = (f2.bytes_delivered * 8.0) / f2_active_time if f2_active_time > 0 else 0
    jfi = jain_fairness([thr1_bps, thr2_bps])

    p99_lat1 = np.percentile(f1.latency_history, 99) if len(f1.latency_history) > 0 else 0
    p99_lat2 = np.percentile(f2.latency_history, 99) if len(f2.latency_history) > 0 else 0
    mean_lat1 = np.mean(f1.latency_history) if len(f1.latency_history) > 0 else 0
    mean_lat2 = np.mean(f2.latency_history) if len(f2.latency_history) > 0 else 0

    mean_queue_delay1 = np.mean(f1.queue_delay_history) if len(f1.queue_delay_history) > 0 else 0
    mean_queue_delay2 = np.mean(f2.queue_delay_history) if len(f2.queue_delay_history) > 0 else 0

    if out_print:
        print("\n" + "=" * 80)
        print("ASYMMETRIC 2-FLOW TEST RESULTS")
        print("=" * 80)
        print(f"Baseline CC        : {baseline.upper()}")
        print(f"Stagger Delay      : {stagger_delay_steps * leo.DT:.1f} seconds")
        print(f"Initial Rates      : Flow1={flow1_initial_rate:.0f} kbps, Flow2={flow2_initial_rate:.0f} kbps")
        print(f"PLC Enabled        : Flow1={flow1_plc}, Flow2={flow2_plc}")
        print()
        print(f"{'Metric':<30} {'Flow1':<15} {'Flow2':<15} {'Difference':<15}")
        print("-" * 80)
        print(f"{'Throughput (kbps)':<30} {thr1_bps/1e3:>14.2f} {thr2_bps/1e3:>14.2f} {abs(thr1_bps-thr2_bps)/1e3:>14.2f}")
        print(f"{'Mean Latency (ms)':<30} {mean_lat1:>14.2f} {mean_lat2:>14.2f} {abs(mean_lat1-mean_lat2):>14.2f}")
        print(f"{'P99 Latency (ms)':<30} {p99_lat1:>14.2f} {p99_lat2:>14.2f} {abs(p99_lat1-p99_lat2):>14.2f}")
        print(f"{'Mean Queue Delay (ms)':<30} {mean_queue_delay1:>14.2f} {mean_queue_delay2:>14.2f} {abs(mean_queue_delay1-mean_queue_delay2):>14.2f}")
        print()
        print("INDEPENDENCE VERIFICATION:")
        print("-" * 80)
        print(f"  Mean Δ Total Latency     : {np.mean(latency_diffs):.4f} ms")
        print(f"  Max Δ Total Latency      : {np.max(latency_diffs):.4f} ms")
        print(f"  Mean Δ Queue Delay       : {np.mean(queue_delay_diffs):.4f} ms")
        print(f"  Max Δ Queue Delay        : {np.max(queue_delay_diffs):.4f} ms")
        print(f"  % Steps with Δ Latency>0 : {100*np.sum(np.array(latency_diffs)>0.001)/len(latency_diffs):.2f}%")
        print()
        print(f"Jain's Fairness Index : {jfi:.4f}")
        print("=" * 80)

    return {
        "thr1_bps": thr1_bps,
        "thr2_bps": thr2_bps,
        "jfi": jfi,
        "mean_latency_diff": np.mean(latency_diffs),
        "max_latency_diff": np.max(latency_diffs),
        "flow1": f1,  # ✅ NEW: Return flow objects for visualization
        "flow2": f2,  # ✅ NEW
        "mean_lat1": mean_lat1,
        "mean_lat2": mean_lat2,
        "p99_lat1": p99_lat1,
        "p99_lat2": p99_lat2,
    }

def run_multi_flow_sim(
    N=50000,
    seed=1,
    baseline="bbr",
    include_handover=False,
    num_flows=5,
    stagger_delay_steps=None,   # 각 flow의 시작 시점 (step 단위 리스트)
    initial_rates=None,         # 각 flow의 초기 rate (kbps 리스트)
    plc_flags=None,             # 각 flow PLC 사용 여부 (bool 리스트)
    out_print=True,
    use_codel=False,
    codel_target_ms=20.0,
    codel_interval_ms=100.0,
):
    """
    General multi-flow simulation (default: 5 flows)

    - Per-flow queue + optional CoDel
    - Arbitrary number of flows (num_flows)
    - 각 flow별 staggered start / initial rate / PLC on/off 설정 가능
    """
    rng = np.random.default_rng(seed)

    # -----------------------------
    # Handover / Loss profile
    # -----------------------------
    if include_handover:
        delay_profile, loss_profile, ho_indicator = leo.create_handover_profile(
            N,
            dt=leo.DT,
            params=leo.HandoverParams(),
            seed=seed,
        )
    else:
        delay_profile = np.zeros(N)
        loss_profile = np.ones(N) * leo.BASE_LOSS_RATE
        ho_indicator = np.zeros(N, dtype=np.int8)

    time_arr = np.arange(N) * leo.DT
    base_capacity = 1500.0 + 300.0 * np.sin(2 * math.pi * time_arr / 60.0)

    preho_cfg = leo.PreHOPredictParams(
        lookahead_steps=5,
        g_boost=1.5,
        reserve_pct=0.20,
        rtt_spike_threshold=1.20,
        rtt_alpha_short=0.3,
        rtt_alpha_long=0.05,
        use_congestion_filter=True,
    )

    # -----------------------------
    # 기본 파라미터 세팅
    # -----------------------------
    if stagger_delay_steps is None:
        stagger_delay_steps = [0] * num_flows
    if initial_rates is None:
        initial_rates = [500.0] * num_flows
    if plc_flags is None:
        plc_flags = [True] * num_flows

    assert len(stagger_delay_steps) == num_flows
    assert len(initial_rates) == num_flows
    assert len(plc_flags) == num_flows

    # -----------------------------
    # Flow / Queue 초기화
    # -----------------------------
    flows = []
    queues = []

    for i in range(num_flows):
        f = Flow(
            name=f"flow{i+1}",
            baseline=baseline,
            use_plc=plc_flags[i],
            seed_offset=1000 * i,
            initial_rate_kbps=initial_rates[i],
        )
        f.active = False   # stagger 처리
        flows.append(f)

        q = PerFlowQueue(
            capacity_share_kbps=0.0,   # 매 step에서 업데이트
            use_codel=use_codel,
            codel_target_ms=codel_target_ms,
            codel_interval_ms=codel_interval_ms,
        )
        queues.append(q)

    cap_factor = 1.0

    # independence / fairness 관측용
    mean_pair_lat_diffs = []
    mean_pair_q_diffs = []

    # -----------------------------
    # 메인 시뮬레이션 루프
    # -----------------------------
    for step in range(N):
        now_s = step * leo.DT

        # 각 flow 활성화 (staggered start)
        for i, f in enumerate(flows):
            if step == stagger_delay_steps[i]:
                f.active = True
                if out_print:
                    print(f"[Step {step}] {f.name} ACTIVATED (stagger={stagger_delay_steps[i]*leo.DT:.1f}s)")

        cur_delay_ms = delay_profile[step]
        cur_loss_rate = loss_profile[step]
        is_ho = ho_indicator[step] == 1

        base_prop_ms = leo.NETWORK_DELAY_BASE + leo.CODEC_DELAY + leo.RENDER_DELAY

        cur_capacity_kbps = base_capacity[step] * cap_factor

        # 현재 queue delay 측정
        queue_delay_ms_list = [q.get_queue_delay_ms() for q in queues]

        # DelayComponents per-flow
        delay_comps = [
            leo.DelayComponents(
                propagation_delay=base_prop_ms,
                handover_delay=cur_delay_ms,
                queue_delay=queue_delay_ms_list[i],
            )
            for i in range(num_flows)
        ]

        # handover prediction
        imminent_flags = []
        for i, f in enumerate(flows):
            if f.active:
                imminent = leo.predict_handover_rtt(
                    f.cc_state, delay_comps[i], preho_cfg
                ) if baseline == "bbr" or baseline == "gcc" else False
            else:
                imminent = False
            imminent_flags.append(imminent)

        g_boosts = [preho_cfg.g_boost if flag else 1.0 for flag in imminent_flags]
        reserve_factors = [
            max(0.0, 1.0 - preho_cfg.reserve_pct) if flag else 1.0
            for flag in imminent_flags
        ]

        imminent_any = any(imminent_flags)
        target_cap = 0.55 if (is_ho or imminent_any) else 1.0  # cap_drop=0.55와 동일
        cap_ramp = 0.08
        cap_factor = cap_factor + cap_ramp * (target_cap - cap_factor)
        cur_capacity_kbps = base_capacity[step] * cap_factor

        # E2E latency per-flow
        total_latencies = [
            base_prop_ms + cur_delay_ms + queue_delay_ms_list[i]
            for i in range(num_flows)
        ]

        # 각 flow의 원하는 rate (PLC 포함/미포함)
        wants = []
        for i, f in enumerate(flows):
            wants.append(
                f.decide_rate(
                    latency_ms=total_latencies[i],
                    is_handover=is_ho,
                    g_boost=g_boosts[i],
                )
            )

        total_want = sum(wants)
        sends_kbps = [0.0] * num_flows

        if total_want <= cur_capacity_kbps:
            sends_kbps = wants[:]
        else:
            if total_want > 0:
                sends_kbps = [
                    cur_capacity_kbps * w / total_want for w in wants
                ]
            else:
                sends_kbps = [0.0] * num_flows

        # reserve factor 적용
        sends_kbps = [
            sends_kbps[i] * reserve_factors[i] for i in range(num_flows)
        ]

        send_bytes = [
            s * 1000.0 * leo.DT / 8.0 for s in sends_kbps
        ]

        # 패킷 손실 적용
        ho_params = leo.HandoverParams()
        delivered_bytes = [
            flows[i].apply_loss(
                send_bytes[i],
                cur_loss_rate,
                is_ho,
                ho_params,
            )
            for i in range(num_flows)
        ]

        # 큐에 enqueue
        for i in range(num_flows):
            queues[i].enqueue(send_bytes[i], now_s)

        # active flow들에 capacity fair share
        active_count = sum(1 for f in flows if f.active)
        for i in range(num_flows):
            if active_count > 0 and flows[i].active:
                queues[i].capacity_share_kbps = cur_capacity_kbps / active_count
            else:
                queues[i].capacity_share_kbps = 0.0

        # dequeue + queue delay after
        queue_delay_after = []
        for i in range(num_flows):
            _, q_delay_ms = queues[i].dequeue(leo.DT, now_s)
            queue_delay_after.append(q_delay_ms)

        # congestion control feedback
        for i, f in enumerate(flows):
            f.apply_feedback(
                delivered_bytes=delivered_bytes[i],
                latency_ms=total_latencies[i],
                step=step,
                queue_bytes=queues[i].queue_bytes,
                delay_comp=delay_comps[i],
                is_handover=is_ho,
                preho_cfg=preho_cfg,
            )

        # pairwise latency / queue delay 차이 (독립성 관찰용)
        if num_flows >= 2:
            pair_lat_diffs = []
            pair_q_diffs = []
            for i in range(num_flows):
                for j in range(i + 1, num_flows):
                    pair_lat_diffs.append(abs(total_latencies[i] - total_latencies[j]))
                    pair_q_diffs.append(abs(queue_delay_ms_list[i] - queue_delay_ms_list[j]))
            mean_pair_lat_diffs.append(np.mean(pair_lat_diffs))
            mean_pair_q_diffs.append(np.mean(pair_q_diffs))

    # -----------------------------
    # 시뮬레이션 결과 요약
    # -----------------------------
    sim_dur_sec = N * leo.DT
    thr_bps = []
    for i, f in enumerate(flows):
        active_time = sim_dur_sec - stagger_delay_steps[i] * leo.DT
        if active_time <= 0:
            thr_bps.append(0.0)
        else:
            thr_bps.append(f.bytes_delivered * 8.0 / active_time)

    jfi = jain_fairness(thr_bps)
    mean_lat = [np.mean(f.latency_history) for f in flows]
    p99_lat = [np.percentile(f.latency_history, 99) for f in flows]
    mean_q_delay = [np.mean(f.queue_delay_history) for f in flows]

    if out_print:
        print("\n" + "=" * 80)
        print(f"MULTI-FLOW TEST RESULTS (num_flows={num_flows})")
        print("=" * 80)
        print(f"Baseline CC        : {baseline.upper()}")
        print(f"Include handover   : {include_handover}")
        print(f"Use CoDel          : {use_codel}")
        print()
        print(f"{'Flow':<8} {'Thr (kbps)':>12} {'MeanLat(ms)':>14} {'P99Lat(ms)':>14} {'MeanQ(ms)':>12}")
        print("-" * 80)
        for i in range(num_flows):
            print(f"{f'flow{i+1}':<8} {thr_bps[i]/1e3:>12.2f} {mean_lat[i]:>14.2f} {p99_lat[i]:>14.2f} {mean_q_delay[i]:>12.2f}")
        print("-" * 80)
        print(f"Jain's Fairness Index : {jfi:.4f}")
        if len(mean_pair_lat_diffs) > 0:
            print(f"Mean pairwise ΔLatency : {np.mean(mean_pair_lat_diffs):.4f} ms")
            print(f"Mean pairwise ΔQDelay  : {np.mean(mean_pair_q_diffs):.4f} ms")
        print("=" * 80)

    return {
        "flows": flows,
        "throughputs_bps": thr_bps,
        "jfi": jfi,
        "mean_latency": mean_lat,
        "p99_latency": p99_lat,
        "mean_queue_delay": mean_q_delay,
        "mean_pair_lat_diff": np.mean(mean_pair_lat_diffs) if mean_pair_lat_diffs else 0.0,
        "mean_pair_q_diff": np.mean(mean_pair_q_diffs) if mean_pair_q_diffs else 0.0,
    }


if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("TEST 1: Staggered Start (Flow2 starts 10s later)")
    print("=" * 80)
    
    result1 = run_asymmetric_two_flow_sim(
        N=50000,
        seed=1,
        baseline="bbr",
        include_handover=False,
        stagger_delay_steps=1000,  # 10 seconds delay
        flow1_initial_rate=500.0,
        flow2_initial_rate=500.0,
        flow1_plc=False,         
        flow2_plc=False,
        use_codel=False,        # ✅ CoDel activation
        codel_target_ms=20.0,  # Ex: queue delay target 20ms
        codel_interval_ms=100.0,
        out_print=True,
    )
    
    # ✅ NEW: Generate visualization for TEST 1
    print("\n📊 Generating visualization for TEST 1...")
    plot_multiflow_latency_comparison(
        flow1=result1["flow1"],
        flow2=result1["flow2"],
        N=50000,
        scenario_name="TEST 1: Staggered Start - BBR Baseline Only (No PLC)",
        save_path="test1_staggered_start.png",
        baseline="BBR"
    )
    
    print("\n" + "=" * 80)
    print("TEST 2: Different Initial Rates")
    print("=" * 80)
    
    result2 = run_asymmetric_two_flow_sim(
        N=50000,
        seed=1,
        baseline="bbr",
        include_handover=False,
        stagger_delay_steps=0,
        flow1_initial_rate=300.0,   # Low start
        flow2_initial_rate=900.0,   # High start
        flow1_plc=False,
        flow2_plc=False,
        use_codel=False,        # ✅ CoDel activation
        codel_target_ms=20.0,  # Ex: queue delay target 20ms
        codel_interval_ms=100.0,
        out_print=True,
    )
    
    # ✅ NEW: Generate visualization for TEST 2
    print("\n📊 Generating visualization for TEST 2...")
    plot_multiflow_latency_comparison(
        flow1=result2["flow1"],
        flow2=result2["flow2"],
        N=50000,
        scenario_name="TEST 2: Asymmetric Initial Rates - BBR Baseline Only (No PLC)",
        save_path="test2_asymmetric_rates.png",
        baseline="BBR"
    )
    
    print("\n" + "=" * 80)
    print("TEST 3: Asymmetric PLC (Flow1=PLC, Flow2=No PLC)")
    print("=" * 80)
    
    result3 = run_asymmetric_two_flow_sim(
        N=50000,
        seed=1,
        baseline="bbr",
        include_handover=True,
        stagger_delay_steps=0,
        flow1_initial_rate=500.0,
        flow2_initial_rate=500.0,
        flow1_plc=True,   # With PLC
        flow2_plc=False,  # Without PLC
        use_codel=False,        # ✅ CoDel activation
        codel_target_ms=20.0,  # Ex: queue delay target 20ms
        codel_interval_ms=100.0,
        out_print=True,
    )
    
    # ✅ NEW: Generate visualization for TEST 3
    print("\n📊 Generating visualization for TEST 3...")
    plot_multiflow_latency_comparison(
        flow1=result3["flow1"],
        flow2=result3["flow2"],
        N=50000,
        scenario_name="TEST 3: Asymmetric PLC Configuration (Flow1=BBR+PLC vs Flow2=BBR)",
        save_path="test3_asymmetric_plc.png",
        baseline="BBR"
    )
    
    print("\n" + "=" * 80)
    print("✅ All tests completed! Visualizations saved to current directory")
    print("=" * 80)

    print("\n" + "=" * 80)
    print("TEST 4: 5-Flow Fair Queueing")
    print("=" * 80)

    result_multi = run_multi_flow_sim(
        N=50000,
        seed=111,
        baseline="bbr",
        include_handover=True,     # LEO with handover
        num_flows=5,
        stagger_delay_steps=[0, 0, 0, 0, 0],   # start together
        initial_rates=[500.0, 500.0, 500.0, 500.0, 500.0],
        plc_flags=[False, False, False, False, False],
        use_codel=False,
        codel_target_ms=20.0,
        codel_interval_ms=100.0,
        out_print=True,
    )

