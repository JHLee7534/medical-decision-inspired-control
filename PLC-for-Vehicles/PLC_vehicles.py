"""
PLC-PID Controller for Vehicle Steering - Visualization Enhanced Version
=========================================================================
차량 조향 제어를 위한 PLC-PID 컨트롤러 시각화 버전

원본 코드에 다음 기능 추가:
1. 시나리오별 상세 시각화
2. 제어기 비교 그래프
3. 성능 지표 막대 그래프
4. 궤적 시각화
5. PLC 메커니즘 설명 그래프
"""

import numpy as np
from collections import deque
from dataclasses import dataclass
from typing import Tuple, Dict, List
import warnings
warnings.filterwarnings('ignore')

import matplotlib.pyplot as plt
import matplotlib
import platform

# 플랫폼에 따라 backend 설정
if platform.system() == 'Windows':
    matplotlib.use('TkAgg')  # Windows에서는 TkAgg 또는 기본값 사용
else:
    matplotlib.use('Agg')  # Linux/서버에서는 Agg 사용

matplotlib.rcParams['font.family'] = 'DejaVu Sans'


# =========================
# 1. 기본 클래스들 (원본 유지)
# =========================

@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 20.0
    delta: float = 0.0


class BicycleModel:
    def __init__(self, L=2.7, max_steer=np.radians(35), steer_rate_limit=np.radians(60)):
        self.L = L
        self.max_steer = max_steer
        self.steer_rate_limit = steer_rate_limit
        
    def update(self, state: VehicleState, steer_cmd: float, dt: float) -> VehicleState:
        steer_diff = steer_cmd - state.delta
        max_delta_change = self.steer_rate_limit * dt
        steer_diff = np.clip(steer_diff, -max_delta_change, max_delta_change)
        
        new_delta = state.delta + steer_diff
        new_delta = np.clip(new_delta, -self.max_steer, self.max_steer)
        
        beta = np.arctan(0.5 * np.tan(new_delta))
        
        new_x = state.x + state.v * np.cos(state.yaw + beta) * dt
        new_y = state.y + state.v * np.sin(state.yaw + beta) * dt
        new_yaw = state.yaw + (state.v / self.L) * np.sin(beta) * dt
        
        return VehicleState(x=new_x, y=new_y, yaw=new_yaw, v=state.v, delta=new_delta)


class RoadProfile:
    def __init__(self, profile_type: str = "single_curve"):
        self.profile_type = profile_type
        
    def get_reference(self, x: float) -> Tuple[float, float, float]:
        if self.profile_type == "single_curve":
            return self._single_curve(x)
        elif self.profile_type == "s_curve":
            return self._s_curve(x)
        elif self.profile_type == "continuous_curves":
            return self._continuous_curves(x)
        elif self.profile_type == "sudden_obstacle":
            return self._sudden_obstacle(x)
        else:
            return 0.0, 0.0, 0.0
    
    def _single_curve(self, x: float) -> Tuple[float, float, float]:
        if x < 100:
            return 0.0, 0.0, 0.0
        elif x < 200:
            t = (x - 100) / 100
            curvature = 0.01 * np.sin(np.pi * t)
            target_y = 5.0 * (1 - np.cos(np.pi * t)) / 2
            road_yaw = np.arctan(0.05 * np.pi * np.sin(np.pi * t))
            return target_y, curvature, road_yaw
        else:
            return 5.0, 0.0, 0.0
    
    def _s_curve(self, x: float) -> Tuple[float, float, float]:
        if x < 100:
            return 0.0, 0.0, 0.0
        elif x < 200:
            t = (x - 100) / 100
            curvature = 0.015 * np.sin(np.pi * t)
            target_y = 4.0 * (1 - np.cos(np.pi * t)) / 2
            road_yaw = np.arctan(0.04 * np.pi * np.sin(np.pi * t))
            return target_y, curvature, road_yaw
        elif x < 300:
            t = (x - 200) / 100
            curvature = -0.015 * np.sin(np.pi * t)
            target_y = 4.0 - 4.0 * (1 - np.cos(np.pi * t)) / 2
            road_yaw = np.arctan(-0.04 * np.pi * np.sin(np.pi * t))
            return target_y, curvature, road_yaw
        else:
            return 0.0, 0.0, 0.0
    
    def _continuous_curves(self, x: float) -> Tuple[float, float, float]:
        if x < 50:
            return 0.0, 0.0, 0.0
        segment = int((x - 50) / 50)
        t = ((x - 50) % 50) / 50
        direction = 1 if segment % 2 == 0 else -1
        curvature = direction * 0.012 * np.sin(np.pi * t)
        base_y = 0.0
        for s in range(segment):
            d = 1 if s % 2 == 0 else -1
            base_y += d * 3.0
        target_y = base_y + direction * 3.0 * (1 - np.cos(np.pi * t)) / 2
        road_yaw = np.arctan(direction * 0.03 * np.pi * np.sin(np.pi * t))
        return target_y, curvature, road_yaw
    
    def _sudden_obstacle(self, x: float) -> Tuple[float, float, float]:
        if x < 140:
            return 0.0, 0.0, 0.0
        elif x < 160:
            t = (x - 140) / 20
            curvature = 0.025 * np.sin(np.pi * t)
            target_y = 3.5 * (1 - np.cos(np.pi * t)) / 2
            road_yaw = np.arctan(0.175 * np.pi * np.sin(np.pi * t))
            return target_y, curvature, road_yaw
        else:
            return 3.5, 0.0, 0.0


class SensorModel:
    def __init__(self, curvature_noise_std=0.002, position_noise_std=0.05, lookahead_error_std=0.1):
        self.curvature_noise_std = curvature_noise_std
        self.position_noise_std = position_noise_std
        self.lookahead_error_std = lookahead_error_std
        
    def get_noisy_position(self, true_y: float) -> float:
        return true_y + np.random.normal(0, self.position_noise_std)
    
    def get_lookahead_curvature(self, road: RoadProfile, current_x: float, lookahead_dist: float = 30.0) -> float:
        future_x = current_x + lookahead_dist
        _, true_curvature, _ = road.get_reference(future_x)
        noise = np.random.normal(0, self.lookahead_error_std * abs(true_curvature) + 0.001)
        return true_curvature + noise


# =========================
# 2. 제어기 클래스들
# =========================

class PIDController:
    """PID Controller (Baseline)"""
    def __init__(self, kp=0.1, ki=0.0, kd=0.2, dt=0.01,
                 integral_limit=(-1.0, 1.0), output_limit=(-0.6, 0.6)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.kp_base = kp
        self.name = "PID"

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.kp = self.kp_base

    def compute(self, error: float) -> float:
        self.integral += error * self.dt
        if self.integral_limit:
            self.integral = np.clip(self.integral, *self.integral_limit)
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.output_limit:
            output = np.clip(output, *self.output_limit)
        self.prev_error = error
        return output


class PLCWrapper:
    """PLC Controller (항상 활성화)"""
    def __init__(self, pid: PIDController):
        self.pid = pid
        self.curvature_threshold = 0.003
        self.feedforward_gain = 8.0
        
        self.kp_base = pid.kp
        self.kp_current = pid.kp
        self.log_kp = np.log(max(self.kp_current, 1e-6))
        
        self.kp_min = 0.7 * self.kp_base
        self.kp_max = 1.3 * self.kp_base
        self.log_kp_min = np.log(max(self.kp_min, 1e-6))
        self.log_kp_max = np.log(max(self.kp_max, 1e-6))
        
        self.log_clip_bound = 0.01
        self.variance_threshold = 0.01
        
        self.error_history = deque(maxlen=30)
        self.name = "PLC-Always"
        
    def reset(self):
        self.pid.reset()
        self.kp_current = self.kp_base
        self.log_kp = np.log(max(self.kp_current, 1e-6))
        self.error_history.clear()

    def compute(self, error: float, lookahead_curvature: float) -> Tuple[float, float, bool]:
        self.error_history.append(error)
        
        feedforward = 0.0
        ff_active = abs(lookahead_curvature) > self.curvature_threshold
        if ff_active:
            feedforward = lookahead_curvature * self.feedforward_gain
        
        if len(self.error_history) >= 5:
            variance = np.var(self.error_history)
            if variance > self.variance_threshold:
                log_delta = -0.01
            else:
                log_delta = 0.005
            log_delta = np.clip(log_delta, -self.log_clip_bound, self.log_clip_bound)
            self.log_kp += log_delta
            self.log_kp = np.clip(self.log_kp, self.log_kp_min, self.log_kp_max)
            self.kp_current = np.exp(self.log_kp)
            self.pid.kp = self.kp_current
        
        pid_output = self.pid.compute(error)
        return pid_output + feedforward, self.kp_current, ff_active


class ConditionalPLCWrapper:
    """조건부 PLC Controller"""
    def __init__(self, pid: PIDController,
                 curvature_activation_threshold: float = 0.022,
                 curvature_rate_threshold: float = 0.003,
                 curvature_threshold: float = 0.003,
                 feedforward_gain: float = 8.0,
                 activation_holdtime: float = 2.0,
                 dt: float = 0.01):
        
        self.pid = pid
        self.dt = dt
        
        self.curvature_activation_threshold = curvature_activation_threshold
        self.curvature_rate_threshold = curvature_rate_threshold
        self.activation_holdtime = activation_holdtime
        
        self.curvature_threshold = curvature_threshold
        self.feedforward_gain = feedforward_gain
        
        self.kp_base = pid.kp
        self.kp_current = pid.kp
        self.log_kp = np.log(max(self.kp_current, 1e-6))
        
        self.kp_min = 0.7 * self.kp_base
        self.kp_max = 1.3 * self.kp_base
        self.log_kp_min = np.log(max(self.kp_min, 1e-6))
        self.log_kp_max = np.log(max(self.kp_max, 1e-6))
        
        self.log_clip_bound = 0.01
        self.variance_threshold = 0.01
        
        self.error_history = deque(maxlen=30)
        self.curvature_history = deque(maxlen=50)
        
        self.plc_active = False
        self.activation_timer = 0.0
        self.name = "PLC-Conditional"
        
    def reset(self):
        self.pid.reset()
        self.kp_current = self.kp_base
        self.log_kp = np.log(max(self.kp_current, 1e-6))
        self.error_history.clear()
        self.curvature_history.clear()
        self.plc_active = False
        self.activation_timer = 0.0
    
    def _check_activation(self, lookahead_curvature: float) -> bool:
        self.curvature_history.append(lookahead_curvature)
        if abs(lookahead_curvature) >= self.curvature_activation_threshold:
            return True
        return False

    def compute(self, error: float, lookahead_curvature: float) -> Tuple[float, float, bool]:
        self.error_history.append(error)
        
        should_activate = self._check_activation(lookahead_curvature)
        
        if should_activate:
            self.plc_active = True
            self.activation_timer = self.activation_holdtime
        elif self.plc_active:
            self.activation_timer -= self.dt
            if self.activation_timer <= 0:
                self.plc_active = False
        
        if not self.plc_active:
            self.pid.kp = self.kp_base
            pid_output = self.pid.compute(error)
            return pid_output, self.kp_base, False
        
        feedforward = 0.0
        if abs(lookahead_curvature) > self.curvature_threshold:
            feedforward = lookahead_curvature * self.feedforward_gain
        
        if len(self.error_history) >= 5:
            variance = np.var(self.error_history)
            if variance > self.variance_threshold:
                log_delta = -0.01
            else:
                log_delta = 0.005
            log_delta = np.clip(log_delta, -self.log_clip_bound, self.log_clip_bound)
            self.log_kp += log_delta
            self.log_kp = np.clip(self.log_kp, self.log_kp_min, self.log_kp_max)
            self.kp_current = np.exp(self.log_kp)
            self.pid.kp = self.kp_current
        
        pid_output = self.pid.compute(error)
        return pid_output + feedforward, self.kp_current, True


# =========================
# 3. 시뮬레이션 함수
# =========================

def run_simulation(controller_type: str, scenario: str, duration: float = 15.0, 
                   dt: float = 0.01, seed: int = None) -> Dict:
    """시뮬레이션 실행"""
    if seed is not None:
        np.random.seed(seed)
    
    vehicle = BicycleModel()
    road = RoadProfile(scenario)
    sensor = SensorModel()
    
    pid = PIDController(kp=0.1, ki=0.0, kd=0.2, dt=dt)
    
    if controller_type == "pid":
        controller = pid
    elif controller_type == "plc_always":
        controller = PLCWrapper(pid)
    elif controller_type == "plc_conditional":
        controller = ConditionalPLCWrapper(pid)
    else:
        raise ValueError(f"Unknown: {controller_type}")
    
    state = VehicleState(v=20.0)
    
    results = {
        'time': [], 'x': [], 'y': [], 'target_y': [], 
        'lateral_error': [], 'steering': [], 'curvature': [], 
        'kp': [], 'plc_active': [], 'controller_name': controller_type
    }
    
    steps = int(duration / dt)
    
    for step in range(steps):
        t = step * dt
        target_y, true_curvature, _ = road.get_reference(state.x)
        measured_y = sensor.get_noisy_position(state.y)
        lookahead_curv = sensor.get_lookahead_curvature(road, state.x)
        
        lateral_error = target_y - measured_y
        
        if controller_type == "pid":
            steer_cmd = controller.compute(lateral_error)
            current_kp = controller.kp
            plc_active = False
        else:
            steer_cmd, current_kp, plc_active = controller.compute(lateral_error, lookahead_curv)
        
        state = vehicle.update(state, steer_cmd, dt)
        
        results['time'].append(t)
        results['x'].append(state.x)
        results['y'].append(state.y)
        results['target_y'].append(target_y)
        results['lateral_error'].append(target_y - state.y)
        results['steering'].append(steer_cmd)
        results['curvature'].append(true_curvature)
        results['kp'].append(current_kp)
        results['plc_active'].append(plc_active)
    
    # Convert to numpy arrays
    for key in results:
        if key != 'controller_name':
            results[key] = np.array(results[key])
    
    return results


def analyze_results(results: Dict) -> Dict:
    """결과 분석"""
    errors = np.abs(results['lateral_error'])
    curv = np.abs(results['curvature'])
    curve_mask = curv > 0.001
    
    analysis = {
        'mean_all': np.mean(errors),
        'max_all': np.max(errors),
        'std_all': np.std(errors),
        'rms_all': np.sqrt(np.mean(errors**2)),
        'activation_rate': np.mean(results['plc_active'])
    }
    
    if np.any(curve_mask):
        curve_errors = errors[curve_mask]
        analysis['mean_curve'] = np.mean(curve_errors)
        analysis['max_curve'] = np.max(curve_errors)
        analysis['rms_curve'] = np.sqrt(np.mean(curve_errors**2))
        analysis['violation_curve'] = np.mean(curve_errors > 0.3)
    else:
        analysis['mean_curve'] = 0
        analysis['max_curve'] = 0
        analysis['rms_curve'] = 0
        analysis['violation_curve'] = 0
    
    return analysis


# =========================
# 4. 시각화 함수들
# =========================

def plot_scenario_comparison(scenario: str, save_path: str):
    """시나리오별 제어기 비교 그래프"""
    controllers = ['pid', 'plc_always', 'plc_conditional']
    colors = {'pid': '#d62728', 'plc_always': '#1f77b4', 'plc_conditional': '#2ca02c'}
    labels = {'pid': 'PID (Baseline)', 'plc_always': 'PLC-Always', 'plc_conditional': 'PLC-Conditional'}
    
    results_list = []
    for ctrl in controllers:
        res = run_simulation(ctrl, scenario, duration=15.0, seed=42)
        results_list.append(res)
    
    fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
    
    # (1) 차량 궤적 vs 목표 경로
    ax1 = axes[0]
    ax1.plot(results_list[0]['x'], results_list[0]['target_y'], 'k--', 
             linewidth=2, label='Target Path', alpha=0.7)
    for i, res in enumerate(results_list):
        ctrl = controllers[i]
        ax1.plot(res['x'], res['y'], color=colors[ctrl], 
                label=labels[ctrl], linewidth=1.2, alpha=0.8)
    ax1.set_ylabel('Y Position [m]')
    ax1.set_title(f'Scenario: {scenario.upper()} - Vehicle Trajectory')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # (2) 횡방향 오차
    ax2 = axes[1]
    ax2.axhline(y=0, color='black', linestyle='--', alpha=0.5)
    ax2.axhspan(-0.3, 0.3, alpha=0.15, color='green', label='±0.3m threshold')
    for i, res in enumerate(results_list):
        ctrl = controllers[i]
        ax2.plot(res['x'], res['lateral_error'], color=colors[ctrl], 
                label=labels[ctrl], linewidth=1, alpha=0.8)
    ax2.set_ylabel('Lateral Error [m]')
    ax2.set_title('Lateral Tracking Error')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([-1.5, 1.5])
    
    # (3) 도로 곡률
    ax3 = axes[2]
    ax3.fill_between(results_list[0]['x'], results_list[0]['curvature'], 
                     alpha=0.3, color='gray', label='Road Curvature')
    ax3.plot(results_list[0]['x'], results_list[0]['curvature'], 
            color='gray', linewidth=1)
    ax3.axhline(y=0.022, color='red', linestyle=':', alpha=0.7, 
               label='PLC Activation Threshold')
    ax3.set_ylabel('Curvature [1/m]')
    ax3.set_title('Road Curvature Profile')
    ax3.legend(loc='upper right')
    ax3.grid(True, alpha=0.3)
    
    # (4) 조향 명령
    ax4 = axes[3]
    for i, res in enumerate(results_list):
        ctrl = controllers[i]
        ax4.plot(res['x'], np.degrees(res['steering']), color=colors[ctrl], 
                label=labels[ctrl], linewidth=1, alpha=0.8)
    ax4.set_ylabel('Steering Angle [deg]')
    ax4.set_title('Steering Command')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    
    # (5) Kp 적응 및 PLC 활성화
    ax5 = axes[4]
    for i, res in enumerate(results_list):
        ctrl = controllers[i]
        ax5.plot(res['x'], res['kp'], color=colors[ctrl], 
                label=f"{labels[ctrl]} Kp", linewidth=1.5)
    
    # PLC 활성화 영역 표시 (plc_conditional만)
    plc_cond_res = results_list[2]
    active_regions = plc_cond_res['plc_active'].astype(float)
    ax5_twin = ax5.twinx()
    ax5_twin.fill_between(plc_cond_res['x'], active_regions * 0.15, 
                          alpha=0.3, color='green', label='PLC Active')
    ax5_twin.set_ylabel('PLC Active', color='green')
    ax5_twin.set_ylim([0, 0.2])
    ax5_twin.set_yticks([0, 0.15])
    ax5_twin.set_yticklabels(['OFF', 'ON'])
    
    ax5.set_xlabel('X Position [m]')
    ax5.set_ylabel('Kp Value')
    ax5.set_title('Controller Gain Adaptation & PLC Activation')
    ax5.legend(loc='upper left')
    ax5.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()


def plot_trajectory_2d(scenario: str, save_path: str):
    """2D 궤적 시각화"""
    controllers = ['pid', 'plc_always', 'plc_conditional']
    colors = {'pid': '#d62728', 'plc_always': '#1f77b4', 'plc_conditional': '#2ca02c'}
    labels = {'pid': 'PID', 'plc_always': 'PLC-Always', 'plc_conditional': 'PLC-Conditional'}
    
    results_list = []
    for ctrl in controllers:
        res = run_simulation(ctrl, scenario, duration=15.0, seed=42)
        results_list.append(res)
    
    fig, ax = plt.subplots(figsize=(14, 6))
    
    # 목표 경로 (도로)
    target_x = results_list[0]['x']
    target_y = results_list[0]['target_y']
    
    # 도로 폭 표시
    road_width = 3.5
    ax.fill_between(target_x, target_y - road_width/2, target_y + road_width/2, 
                   alpha=0.2, color='gray', label='Road')
    ax.plot(target_x, target_y, 'k--', linewidth=2, label='Center Line')
    
    # 각 제어기 궤적
    for i, res in enumerate(results_list):
        ctrl = controllers[i]
        ax.plot(res['x'], res['y'], color=colors[ctrl], 
               label=labels[ctrl], linewidth=2, alpha=0.9)
    
    ax.set_xlabel('X Position [m]')
    ax.set_ylabel('Y Position [m]')
    ax.set_title(f'Vehicle Trajectories - {scenario.upper()}')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='datalim')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()


def plot_all_scenarios_metrics(save_path: str):
    """모든 시나리오의 성능 지표 비교"""
    scenarios = ["single_curve", "s_curve", "continuous_curves", "sudden_obstacle"]
    controllers = ['pid', 'plc_always', 'plc_conditional']
    
    # Monte Carlo 실행 (간소화: 10회)
    all_metrics = {sc: {ctrl: [] for ctrl in controllers} for sc in scenarios}
    
    print("Monte Carlo 시뮬레이션 실행 중...")
    for sc in scenarios:
        for run in range(10):
            seed = 1000 + run
            for ctrl in controllers:
                res = run_simulation(ctrl, sc, duration=15.0, seed=seed)
                analysis = analyze_results(res)
                all_metrics[sc][ctrl].append(analysis)
    
    # 평균 계산
    avg_metrics = {}
    for sc in scenarios:
        avg_metrics[sc] = {}
        for ctrl in controllers:
            avg_metrics[sc][ctrl] = {
                'mean_all': np.mean([m['mean_all'] for m in all_metrics[sc][ctrl]]),
                'mean_curve': np.mean([m['mean_curve'] for m in all_metrics[sc][ctrl]]),
                'max_curve': np.mean([m['max_curve'] for m in all_metrics[sc][ctrl]]),
                'violation_curve': np.mean([m['violation_curve'] for m in all_metrics[sc][ctrl]]),
                'activation_rate': np.mean([m['activation_rate'] for m in all_metrics[sc][ctrl]])
            }
    
    # 그래프 생성
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    x = np.arange(len(scenarios))
    width = 0.25
    colors = ['#d62728', '#1f77b4', '#2ca02c']
    labels = ['PID', 'PLC-Always', 'PLC-Conditional']
    
    metrics_info = [
        ('mean_all', 'Mean Lateral Error - All [m]', axes[0, 0]),
        ('mean_curve', 'Mean Lateral Error - Curves [m]', axes[0, 1]),
        ('max_curve', 'Max Lateral Error - Curves [m]', axes[1, 0]),
        ('activation_rate', 'PLC Activation Rate', axes[1, 1])
    ]
    
    for metric_key, metric_title, ax in metrics_info:
        for i, ctrl in enumerate(controllers):
            values = [avg_metrics[sc][ctrl][metric_key] for sc in scenarios]
            if metric_key == 'activation_rate':
                values = [v * 100 for v in values]  # 퍼센트로 변환
            bars = ax.bar(x + i * width, values, width, label=labels[i], color=colors[i])
            
            # 값 표시
            for bar, val in zip(bars, values):
                height = bar.get_height()
                if metric_key == 'activation_rate':
                    ax.text(bar.get_x() + bar.get_width()/2, height + 1,
                           f'{val:.0f}%', ha='center', va='bottom', fontsize=8)
                else:
                    ax.text(bar.get_x() + bar.get_width()/2, height + 0.01,
                           f'{val:.3f}', ha='center', va='bottom', fontsize=8)
        
        ax.set_ylabel(metric_title.split(' - ')[-1])
        ax.set_title(metric_title)
        ax.set_xticks(x + width)
        ax.set_xticklabels([s.replace('_', '\n') for s in scenarios], fontsize=9)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3, axis='y')
    
    plt.suptitle('Performance Comparison Across All Scenarios (10 Monte Carlo Runs)', 
                fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    
    return avg_metrics


def plot_plc_mechanism_vehicle(save_path: str):
    """차량 PLC 메커니즘 설명 그래프"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # (1) Log-domain Kp 적응
    ax1 = axes[0, 0]
    kp_base = 0.1
    variance_history = np.concatenate([
        np.ones(50) * 0.005,   # 안정
        np.ones(50) * 0.015,   # 불안정 (진동)
        np.ones(50) * 0.008,   # 감소
        np.ones(50) * 0.003    # 안정
    ])
    
    kp = np.zeros(200)
    log_kp = np.log(kp_base)
    kp_min, kp_max = 0.7 * kp_base, 1.3 * kp_base
    variance_threshold = 0.01
    
    for i in range(200):
        if variance_history[i] > variance_threshold:
            log_delta = -0.01  # 감소 (damping)
        else:
            log_delta = 0.005  # 증가 (recovery)
        log_delta = np.clip(log_delta, -0.01, 0.01)
        log_kp += log_delta
        log_kp = np.clip(log_kp, np.log(kp_min), np.log(kp_max))
        kp[i] = np.exp(log_kp)
    
    t = np.arange(200) * 0.01
    ax1.plot(t, variance_history, label='Error Variance', alpha=0.7, linewidth=1.5)
    ax1.axhline(y=variance_threshold, color='red', linestyle='--', 
               label='Threshold', alpha=0.7)
    ax1.set_ylabel('Variance', color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    
    ax1_twin = ax1.twinx()
    ax1_twin.plot(t, kp, color='green', label='Kp', linewidth=2)
    ax1_twin.axhline(y=kp_base, color='gray', linestyle=':', alpha=0.5)
    ax1_twin.set_ylabel('Kp Value', color='green')
    ax1_twin.tick_params(axis='y', labelcolor='green')
    
    ax1.set_xlabel('Time [s]')
    ax1.set_title('Log-domain Kp Adaptation\n(Variance ↑ → Kp ↓, Variance ↓ → Kp ↑)')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)
    
    # (2) Feedforward 효과
    ax2 = axes[0, 1]
    x = np.linspace(0, 300, 500)
    road = RoadProfile("sudden_obstacle")
    curvature = np.array([road.get_reference(xi)[1] for xi in x])
    
    ax2.plot(x, curvature * 1000, label='Road Curvature × 1000', linewidth=2)
    ax2.axhline(y=3, color='orange', linestyle='--', 
               label='FF Threshold (0.003)', alpha=0.7)
    ax2.axhline(y=22, color='red', linestyle='--', 
               label='PLC Activation (0.022)', alpha=0.7)
    
    # Feedforward 영역 표시
    ff_active = np.abs(curvature) > 0.003
    ax2.fill_between(x, 0, 30, where=ff_active, alpha=0.2, 
                    color='blue', label='Feedforward Active')
    
    ax2.set_xlabel('X Position [m]')
    ax2.set_ylabel('Curvature × 1000 [1/m]')
    ax2.set_title('Curvature-based Feedforward Activation\n(Sudden Obstacle Scenario)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([-5, 35])
    
    # (3) 조건부 활성화 vs 항상 활성화
    ax3 = axes[1, 0]
    scenarios = ['single_curve', 's_curve', 'continuous_curves', 'sudden_obstacle']
    max_curvatures = [0.01, 0.015, 0.012, 0.025]
    
    x_pos = np.arange(len(scenarios))
    bars = ax3.bar(x_pos, np.array(max_curvatures) * 1000, color='steelblue', alpha=0.7)
    ax3.axhline(y=22, color='red', linestyle='--', linewidth=2,
               label='Conditional Activation Threshold (0.022)')
    
    # 임계값 초과 여부 표시
    for i, (bar, curv) in enumerate(zip(bars, max_curvatures)):
        color = 'green' if curv >= 0.022 else 'gray'
        bar.set_color(color)
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                f'{curv*1000:.0f}', ha='center', fontsize=10, fontweight='bold')
    
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels([s.replace('_', '\n') for s in scenarios])
    ax3.set_ylabel('Max Curvature × 1000 [1/m]')
    ax3.set_title('Scenario Max Curvature vs Activation Threshold\n(Green = PLC Activated)')
    ax3.legend(loc='upper left')
    ax3.grid(True, alpha=0.3, axis='y')
    
    # (4) PID vs PLC 응답 비교
    ax4 = axes[1, 1]
    
    # 단순화된 시뮬레이션
    res_pid = run_simulation('pid', 'sudden_obstacle', duration=12.0, seed=42)
    res_plc = run_simulation('plc_conditional', 'sudden_obstacle', duration=12.0, seed=42)
    
    # 급변 구간만 확대
    mask = (res_pid['x'] > 130) & (res_pid['x'] < 200)
    
    ax4.plot(res_pid['x'][mask], res_pid['lateral_error'][mask], 
            'r-', label='PID', linewidth=1.5, alpha=0.8)
    ax4.plot(res_plc['x'][mask], res_plc['lateral_error'][mask], 
            'g-', label='PLC-Conditional', linewidth=1.5, alpha=0.8)
    ax4.axhline(y=0, color='black', linestyle='--', alpha=0.5)
    ax4.axhspan(-0.3, 0.3, alpha=0.15, color='green')
    
    # 장애물 위치 표시
    ax4.axvspan(140, 160, alpha=0.2, color='red', label='Obstacle Zone')
    
    ax4.set_xlabel('X Position [m]')
    ax4.set_ylabel('Lateral Error [m]')
    ax4.set_title('Response Comparison at Sudden Obstacle\n(PLC reduces overshoot)')
    ax4.legend(loc='upper right')
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle('PLC Mechanism for Vehicle Steering Control', 
                fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()


def plot_improvement_summary(avg_metrics: Dict, save_path: str):
    """개선율 요약 그래프"""
    scenarios = list(avg_metrics.keys())
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # (1) PLC-Always vs PID 개선율
    ax1 = axes[0]
    improvements_always = []
    for sc in scenarios:
        pid_val = avg_metrics[sc]['pid']['mean_curve']
        plc_val = avg_metrics[sc]['plc_always']['mean_curve']
        if pid_val > 0:
            imp = (1 - plc_val / pid_val) * 100
        else:
            imp = 0
        improvements_always.append(imp)
    
    colors = ['green' if v > 0 else 'red' for v in improvements_always]
    bars1 = ax1.bar(scenarios, improvements_always, color=colors, alpha=0.7)
    ax1.axhline(y=0, color='black', linestyle='-', linewidth=1)
    ax1.set_ylabel('Improvement vs PID [%]')
    ax1.set_title('PLC-Always Improvement\n(Curve Sections)')
    ax1.set_xticklabels([s.replace('_', '\n') for s in scenarios])
    ax1.grid(True, alpha=0.3, axis='y')
    
    for bar, val in zip(bars1, improvements_always):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2, 
                height + (2 if height >= 0 else -4),
                f'{val:+.1f}%', ha='center', fontsize=10, fontweight='bold')
    
    # (2) PLC-Conditional vs PID 개선율
    ax2 = axes[1]
    improvements_cond = []
    for sc in scenarios:
        pid_val = avg_metrics[sc]['pid']['mean_curve']
        plc_val = avg_metrics[sc]['plc_conditional']['mean_curve']
        if pid_val > 0:
            imp = (1 - plc_val / pid_val) * 100
        else:
            imp = 0
        improvements_cond.append(imp)
    
    colors = ['green' if v > 0 else 'red' for v in improvements_cond]
    bars2 = ax2.bar(scenarios, improvements_cond, color=colors, alpha=0.7)
    ax2.axhline(y=0, color='black', linestyle='-', linewidth=1)
    ax2.set_ylabel('Improvement vs PID [%]')
    ax2.set_title('PLC-Conditional Improvement\n(Curve Sections)')
    ax2.set_xticklabels([s.replace('_', '\n') for s in scenarios])
    ax2.grid(True, alpha=0.3, axis='y')
    
    for bar, val in zip(bars2, improvements_cond):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2, 
                height + (2 if height >= 0 else -4),
                f'{val:+.1f}%', ha='center', fontsize=10, fontweight='bold')
    
    plt.suptitle('PLC Performance Improvement over PID Baseline', 
                fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()


# =========================
# 5. 메인 함수
# =========================

def main():
    print("=" * 80)
    print("PLC-PID Vehicle Steering Control - Visualization")
    print("=" * 80)
    
    # 출력 디렉토리 설정 (현재 디렉토리에 output 폴더 생성)
    import os
    output_dir = os.path.join(os.getcwd(), "plc_output")
    os.makedirs(output_dir, exist_ok=True)
    print(f"\n출력 디렉토리: {output_dir}")
    
    # 시나리오별 상세 비교 그래프
    scenarios = ["single_curve", "s_curve", "continuous_curves", "sudden_obstacle"]
    
    print("\n시나리오별 비교 그래프 생성 중...")
    for scenario in scenarios:
        print(f"  - {scenario}...")
        plot_scenario_comparison(scenario, os.path.join(output_dir, f"vehicle_{scenario}.png"))
        plot_trajectory_2d(scenario, os.path.join(output_dir, f"trajectory_{scenario}.png"))
    
    # 전체 성능 지표 비교
    print("\n전체 성능 지표 비교 그래프 생성 중...")
    avg_metrics = plot_all_scenarios_metrics(os.path.join(output_dir, "vehicle_metrics_all.png"))
    
    # PLC 메커니즘 설명
    print("\nPLC 메커니즘 설명 그래프 생성 중...")
    plot_plc_mechanism_vehicle(os.path.join(output_dir, "vehicle_plc_mechanism.png"))
    
    # 개선율 요약
    print("\n개선율 요약 그래프 생성 중...")
    plot_improvement_summary(avg_metrics, os.path.join(output_dir, "vehicle_improvement.png"))
    
    # 결과 요약 출력
    print("\n" + "=" * 80)
    print("결과 요약")
    print("=" * 80)
    
    print("\n[시나리오별 Mean Lateral Error (Curve Sections)]")
    print("-" * 70)
    print(f"{'Scenario':<20} | {'PID':>10} | {'PLC-Always':>12} | {'PLC-Cond':>12}")
    print("-" * 70)
    for sc in scenarios:
        pid = avg_metrics[sc]['pid']['mean_curve']
        always = avg_metrics[sc]['plc_always']['mean_curve']
        cond = avg_metrics[sc]['plc_conditional']['mean_curve']
        print(f"{sc:<20} | {pid:>10.4f} | {always:>12.4f} | {cond:>12.4f}")
    
    print("\n[PID 대비 개선율 (%)]")
    print("-" * 70)
    print(f"{'Scenario':<20} | {'PLC-Always':>12} | {'PLC-Cond':>12}")
    print("-" * 70)
    for sc in scenarios:
        pid = avg_metrics[sc]['pid']['mean_curve']
        always = avg_metrics[sc]['plc_always']['mean_curve']
        cond = avg_metrics[sc]['plc_conditional']['mean_curve']
        imp_always = (1 - always / pid) * 100 if pid > 0 else 0
        imp_cond = (1 - cond / pid) * 100 if pid > 0 else 0
        print(f"{sc:<20} | {imp_always:>+11.1f}% | {imp_cond:>+11.1f}%")
    
    print("\n" + "=" * 80)
    print("그래프 생성 완료!")
    print("=" * 80)


if __name__ == "__main__":
    main()
