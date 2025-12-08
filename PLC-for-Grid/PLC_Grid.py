import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Tuple
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # GUI 없이 저장용


# =========================
# 1. 계통 및 시뮬레이션 파라미터
# =========================

@dataclass
class GridParams:
    f_nom: float = 60.0      # 정격 주파수 [Hz]
    M: float = 4.0           # 관성 상수 (작을수록 저관성 계통)
    D: float = 1.0           # 주파수 댐핑 계수
    P_rated_ess: float = 1.0 # ESS 정격 용량 (p.u.)


@dataclass
class SimParams:
    dt: float = 0.1          # 시뮬레이션 시간 간격 [s]
    t_end: float = 300.0     # 시뮬레이션 총 시간 [s]
    seed: int = 42           # 난수 고정용 시드


# =========================
# 2. 재생에너지 변동(출력 변화) 모델 - 개선
# =========================

class RenewableDisturbance:
    """
    재생에너지(풍력/태양광 등) 출력 변동을 모사하는 모델.
    개선: 외란 시퀀스를 미리 생성하여 동일 조건 비교 가능
    """
    def __init__(self,
                 sigma_noise: float = 0.05,
                 drift_tau: float = 30.0,
                 step_prob_per_sec: float = 0.02,
                 step_mag_mean: float = 0.3):
        self.sigma_noise = sigma_noise
        self.drift_tau = drift_tau
        self.step_prob_per_sec = step_prob_per_sec
        self.step_mag_mean = step_mag_mean

    def generate_sequence(self, sim: SimParams) -> np.ndarray:
        """
        전체 시뮬레이션에 대한 외란 시퀀스를 미리 생성
        """
        np.random.seed(sim.seed)
        t_steps = int(sim.t_end / sim.dt)
        dt = sim.dt
        
        P_disturb_seq = np.zeros(t_steps)
        drift_state = 0.0
        
        for i in range(t_steps):
            # OU-like slow drift
            drift_noise = np.random.normal(0.0, 0.01)
            drift_state += (-drift_state / self.drift_tau) * dt + drift_noise * np.sqrt(dt)
            
            # High-frequency noise
            hf_noise = np.random.normal(0.0, self.sigma_noise)
            
            # Occasional step events
            if np.random.rand() < self.step_prob_per_sec * dt:
                step = np.random.exponential(self.step_mag_mean)
                step *= np.sign(np.random.randn())
            else:
                step = 0.0
            
            P_disturb_seq[i] = drift_state + hf_noise + step
        
        return P_disturb_seq


# =========================
# 3. ESS 주파수 제어기들
# =========================

class ESSDroopController:
    """
    고전적 Droop 제어: P_ess = K_droop * (f_nom - f)
    """
    def __init__(self, grid: GridParams, K_droop: float = 0.5):
        self.grid = grid
        self.K_droop = K_droop
        self.name = "Droop"

    def reset(self):
        pass

    def compute(self, freq: float) -> Tuple[float, float]:
        P_ess = self.K_droop * (self.grid.f_nom - freq) * self.grid.P_rated_ess
        P_ess = np.clip(P_ess, -self.grid.P_rated_ess, self.grid.P_rated_ess)
        return P_ess, self.K_droop


class ESSPLCController:
    """
    개선된 PLC 기반 ESS 제어기
    
    핵심 변경: 주파수 편차가 클 때 gain을 '증가'시켜 더 강하게 대응
    
    - freq_dev = |f_nom - f_meas| / f_nom  (정규화된 주파수 편차)
    - stress = freq_dev * sensitivity
    - stress_ema: 지수이동평균으로 스트레스 추적
    - 새로운 로직:
        - stress가 높으면 → gain 증가 (더 강한 대응)
        - stress가 낮으면 → gain을 기본값으로 서서히 복귀
    - Log-domain bounded update 유지 (안정성)
    """
    def __init__(self,
                 grid: GridParams,
                 K_init: float = 0.5,
                 K_min: float = 0.3,
                 K_max: float = 3.0,
                 theta: float = 2.0,           # gain 증가 민감도
                 sensitivity: float = 1000.0,  # 주파수 편차 → stress 변환 계수 (더 민감하게)
                 alpha_stress: float = 0.4,    # EMA 계수 (더 빠른 반응)
                 recovery_rate: float = 0.02,  # 안정 시 복귀 속도
                 stress_threshold: float = 0.3):  # stress 임계값 (더 낮게)
        self.grid = grid
        self.K_init = K_init
        self.K_min = K_min
        self.K_max = K_max
        self.theta = theta
        self.sensitivity = sensitivity
        self.alpha_stress = alpha_stress
        self.recovery_rate = recovery_rate
        self.stress_threshold = stress_threshold
        self.name = "PLC-ESS"
        
        self.reset()

    def reset(self):
        self.K_ess = self.K_init
        self.log_K = np.log(self.K_ess)
        self.stress_ema = 0.0

    def compute(self, freq: float) -> Tuple[float, float]:
        # (1) 정규화된 주파수 편차 계산
        freq_dev = abs(self.grid.f_nom - freq) / self.grid.f_nom
        
        # (2) Stress 계산 (편차가 클수록 stress 증가)
        stress_instant = freq_dev * self.sensitivity
        
        # (3) EMA 업데이트
        self.stress_ema = (self.alpha_stress * stress_instant + 
                          (1.0 - self.alpha_stress) * self.stress_ema)
        
        # (4) Gain 조정 로직 (핵심 변경!)
        if self.stress_ema > self.stress_threshold:
            # 스트레스가 높으면 gain 증가 → 더 강한 주파수 제어
            # f_r > 1 → log(f_r) > 0 → K 증가
            f_r = 1.0 + self.theta * (self.stress_ema - self.stress_threshold)
        else:
            # 스트레스가 낮으면 기본값으로 서서히 복귀
            # K_ess > K_init 이면 감소, K_ess < K_init 이면 증가
            target_log_K = np.log(self.K_init)
            diff = target_log_K - self.log_K
            recovery_step = np.sign(diff) * min(abs(diff), self.recovery_rate)
            self.log_K += recovery_step
            f_r = 1.0  # 추가 조정 없음
        
        # (5) Log-domain bounded update (안정성 보장)
        if self.stress_ema > self.stress_threshold:
            delta_log = np.log(max(f_r, 1e-6))
            delta_log = np.clip(delta_log, -0.05, 0.05)  # per-step ±5% 제한
            self.log_K += delta_log
        
        # (6) K_ess 재계산 및 클리핑
        self.K_ess = float(np.exp(self.log_K))
        self.K_ess = float(np.clip(self.K_ess, self.K_min, self.K_max))
        self.log_K = np.log(self.K_ess)
        
        # (7) ESS 출력 계산
        P_ess = self.K_ess * (self.grid.f_nom - freq) * self.grid.P_rated_ess
        P_ess = np.clip(P_ess, -self.grid.P_rated_ess, self.grid.P_rated_ess)
        
        return P_ess, self.K_ess


class ESSAdaptiveDroopController:
    """
    적응형 Droop 제어기 (비교용)
    - 주파수 편차에 따라 Droop 계수를 비례적으로 조정
    - PLC보다 단순하지만 적응적
    """
    def __init__(self,
                 grid: GridParams,
                 K_base: float = 0.5,
                 K_min: float = 0.2,
                 K_max: float = 2.0,
                 adapt_gain: float = 10.0):
        self.grid = grid
        self.K_base = K_base
        self.K_min = K_min
        self.K_max = K_max
        self.adapt_gain = adapt_gain
        self.name = "Adaptive Droop"
        self.K_current = K_base

    def reset(self):
        self.K_current = self.K_base

    def compute(self, freq: float) -> Tuple[float, float]:
        # 주파수 편차에 비례하여 gain 조정
        freq_dev = abs(self.grid.f_nom - freq)
        self.K_current = self.K_base + self.adapt_gain * freq_dev
        self.K_current = np.clip(self.K_current, self.K_min, self.K_max)
        
        P_ess = self.K_current * (self.grid.f_nom - freq) * self.grid.P_rated_ess
        P_ess = np.clip(P_ess, -self.grid.P_rated_ess, self.grid.P_rated_ess)
        
        return P_ess, self.K_current


# =========================
# 4. 계통 동특성 모델
# =========================

def update_frequency(freq: float,
                     grid: GridParams,
                     P_disturb: float,
                     P_ess: float,
                     dt: float) -> float:
    """
    1차 계통 주파수 동특성:
        df/dt = (P_ess - P_disturb - D*(f - f_nom)) / M
    """
    df_dt = (P_ess - P_disturb - grid.D * (freq - grid.f_nom)) / grid.M
    freq_next = freq + df_dt * dt
    
    # 비현실적인 주파수 방지 (45~75 Hz 범위)
    freq_next = np.clip(freq_next, 45.0, 75.0)
    
    return freq_next


# =========================
# 5. 시뮬레이션 루프
# =========================

def run_simulation(controller,
                   grid: GridParams,
                   sim: SimParams,
                   P_disturb_seq: np.ndarray) -> Dict[str, np.ndarray]:
    """
    개선된 시뮬레이션: 미리 생성된 외란 시퀀스 사용
    """
    freq = grid.f_nom
    t_steps = len(P_disturb_seq)
    time = np.arange(0.0, t_steps * sim.dt, sim.dt)[:t_steps]
    
    controller.reset()
    
    freq_hist = np.zeros(t_steps)
    P_ess_hist = np.zeros(t_steps)
    K_hist = np.zeros(t_steps)
    
    for i in range(t_steps):
        P_disturb = P_disturb_seq[i]
        P_ess, K_val = controller.compute(freq)
        freq = update_frequency(freq, grid, P_disturb, P_ess, sim.dt)
        
        freq_hist[i] = freq
        P_ess_hist[i] = P_ess
        K_hist[i] = K_val
    
    return {
        "time": time,
        "freq": freq_hist,
        "P_ess": P_ess_hist,
        "P_disturb": P_disturb_seq.copy(),
        "K": K_hist,
        "name": controller.name
    }


def analyze_results(results: Dict[str, np.ndarray],
                    grid: GridParams) -> Dict[str, float]:
    freq = results["freq"]
    dev = freq - grid.f_nom
    P_ess = results["P_ess"]
    
    metrics = {
        "rms_dev": float(np.sqrt(np.mean(dev**2))),
        "max_dev": float(np.max(np.abs(dev))),
        "p95_dev": float(np.percentile(np.abs(dev), 95)),
        "mean_abs_dev": float(np.mean(np.abs(dev))),
        "ess_rms": float(np.sqrt(np.mean(P_ess**2))),  # ESS 사용량 지표
        "ess_max": float(np.max(np.abs(P_ess))),
    }
    return metrics


# =========================
# 6. 시각화
# =========================

def plot_comparison(results_list: List[Dict],
                    grid: GridParams,
                    save_path: str = "comparison.png"):
    """
    여러 제어기 결과를 비교하는 그래프 생성
    """
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    
    # 공통 외란 (첫 번째 결과에서 가져옴)
    time = results_list[0]["time"]
    P_disturb = results_list[0]["P_disturb"]
    
    # (1) 주파수 비교
    ax1 = axes[0]
    ax1.axhline(y=grid.f_nom, color='gray', linestyle='--', alpha=0.5, label='Nominal (60 Hz)')
    ax1.axhspan(grid.f_nom - 0.05, grid.f_nom + 0.05, alpha=0.2, color='green', label='±0.05 Hz band')
    for i, res in enumerate(results_list):
        ax1.plot(time, res["freq"], color=colors[i], label=res["name"], linewidth=0.8)
    ax1.set_ylabel("Frequency [Hz]")
    ax1.set_title("Grid Frequency Response Comparison")
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([grid.f_nom - 0.15, grid.f_nom + 0.15])
    
    # (2) 주파수 편차 비교
    ax2 = axes[1]
    for i, res in enumerate(results_list):
        dev = res["freq"] - grid.f_nom
        ax2.plot(time, dev * 1000, color=colors[i], label=res["name"], linewidth=0.8)  # mHz로 표시
    ax2.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax2.set_ylabel("Freq Deviation [mHz]")
    ax2.set_title("Frequency Deviation from Nominal")
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # (3) ESS 출력 비교
    ax3 = axes[2]
    for i, res in enumerate(results_list):
        ax3.plot(time, res["P_ess"], color=colors[i], label=res["name"], linewidth=0.8)
    ax3.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax3.set_ylabel("ESS Power [p.u.]")
    ax3.set_title("ESS Power Output")
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    
    # (4) Gain (K) 변화 비교
    ax4 = axes[3]
    for i, res in enumerate(results_list):
        ax4.plot(time, res["K"], color=colors[i], label=res["name"], linewidth=0.8)
    ax4.set_ylabel("Controller Gain K")
    ax4.set_xlabel("Time [s]")
    ax4.set_title("Controller Gain Adaptation")
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"그래프 저장: {save_path}")


def plot_metrics_bar(metrics_dict: Dict[str, Dict], save_path: str = "metrics.png"):
    """
    성능 지표 막대 그래프
    """
    controllers = list(metrics_dict.keys())
    metrics_names = ["RMS Deviation\n[mHz]", "Max Deviation\n[mHz]", "95th Deviation\n[mHz]"]
    metrics_keys = ["rms_dev", "max_dev", "p95_dev"]
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    x = np.arange(len(metrics_names))
    width = 0.25
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']
    
    for i, ctrl in enumerate(controllers):
        values = [metrics_dict[ctrl][key] * 1000 for key in metrics_keys]  # Hz → mHz
        bars = ax.bar(x + i * width, values, width, label=ctrl, color=colors[i])
        # 값 표시
        for bar, val in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                   f'{val:.1f}', ha='center', va='bottom', fontsize=9)
    
    ax.set_ylabel('Frequency Deviation [mHz]')
    ax.set_title('Performance Comparison: Frequency Deviation Metrics')
    ax.set_xticks(x + width)
    ax.set_xticklabels(metrics_names)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"지표 그래프 저장: {save_path}")


# =========================
# 7. 메인: 비교 시뮬레이션
# =========================

def main():
    print("=" * 80)
    print("ESS Frequency Control Simulation: Improved PLC vs Droop vs Adaptive")
    print("=" * 80)
    
    # 계통 파라미터 (저관성 계통)
    grid = GridParams(
        f_nom=60.0,
        M=3.0,          # 낮은 관성 (재생에너지 비중 높음)
        D=1.0,
        P_rated_ess=1.0
    )
    
    # 시뮬레이션 파라미터
    sim = SimParams(
        dt=0.1,
        t_end=300.0,
        seed=123
    )
    
    # 재생에너지 변동 모델
    renew = RenewableDisturbance(
        sigma_noise=0.08,
        drift_tau=40.0,
        step_prob_per_sec=0.03,
        step_mag_mean=0.4
    )
    
    # 동일한 외란 시퀀스 생성 (공정한 비교!)
    print("\n외란 시퀀스 생성 중...")
    P_disturb_seq = renew.generate_sequence(sim)
    print(f"  - 외란 RMS: {np.sqrt(np.mean(P_disturb_seq**2)):.4f} p.u.")
    print(f"  - 외란 Max: {np.max(np.abs(P_disturb_seq)):.4f} p.u.")
    
    # 제어기 정의
    controllers = [
        ESSDroopController(grid, K_droop=0.6),
        ESSPLCController(grid, K_init=0.6),
        ESSAdaptiveDroopController(grid, K_base=0.6),
    ]
    
    # 시뮬레이션 실행
    results_list = []
    metrics_dict = {}
    
    print("\n시뮬레이션 실행 중...")
    for ctrl in controllers:
        res = run_simulation(ctrl, grid, sim, P_disturb_seq)
        metrics = analyze_results(res, grid)
        results_list.append(res)
        metrics_dict[ctrl.name] = metrics
        
        print(f"\n[{ctrl.name}]")
        print(f"  RMS freq deviation  : {metrics['rms_dev']*1000:+.2f} mHz")
        print(f"  Max |freq dev|      : {metrics['max_dev']*1000:+.2f} mHz")
        print(f"  95th |freq dev|     : {metrics['p95_dev']*1000:+.2f} mHz")
        print(f"  Mean |freq dev|     : {metrics['mean_abs_dev']*1000:+.2f} mHz")
        print(f"  ESS RMS power       : {metrics['ess_rms']:.4f} p.u.")
    
    # 상대 비교
    print("\n" + "=" * 80)
    print("상대 비교 (Droop 대비 개선율)")
    print("=" * 80)
    
    droop_metrics = metrics_dict["Droop"]
    for name, metrics in metrics_dict.items():
        if name == "Droop":
            continue
        imp_rms = (1.0 - metrics["rms_dev"] / droop_metrics["rms_dev"]) * 100.0
        imp_max = (1.0 - metrics["max_dev"] / droop_metrics["max_dev"]) * 100.0
        imp_p95 = (1.0 - metrics["p95_dev"] / droop_metrics["p95_dev"]) * 100.0
        
        print(f"\n[{name}]")
        print(f"  RMS 주파수 편차 개선율 : {imp_rms:+.1f}%")
        print(f"  Max 주파수 편차 개선율 : {imp_max:+.1f}%")
        print(f"  95th 주파수 편차 개선율: {imp_p95:+.1f}%")
    
    # 그래프 생성
    print("\n" + "-" * 80)
    print("그래프 생성 중...")
    plot_comparison(results_list, grid, "/home/claude/comparison.png")
    plot_metrics_bar(metrics_dict, "/home/claude/metrics.png")
    
    print("\n시뮬레이션 완료!")
    print("=" * 80)
    
    return results_list, metrics_dict


if __name__ == "__main__":
    main()
