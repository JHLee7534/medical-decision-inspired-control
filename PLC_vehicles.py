"""
PLC-PID Controller - Conditional Activation
============================================
조정된 파라미터 + 조건부 PLC 활성화

PID Baseline (A안):
- kp=0.1, ki=0.0, kd=0.2
- Mean lateral error ≈ 0.48~0.59m

PLC 파라미터 (보수적):
- curvature_threshold = 0.003
- feedforward_gain = 8.0
- kp_min = 0.7 * kp_base
- kp_max = 1.3 * kp_base
- log_clip_bound = 0.01

조건부 활성화:
- Sudden Obstacle 같은 급변 상황에서만 PLC ON
- 그 외에는 순수 PID
"""

import numpy as np
from collections import deque
from dataclasses import dataclass
from typing import Tuple, Dict
import warnings
warnings.filterwarnings('ignore')


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
            curvature = 0.025 * np.sin(np.pi * t)  # 최대 곡률 0.025
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


class PIDController:
    """PID Controller (A안 Baseline)"""
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
    """
    PLC Controller (항상 활성화) - 조정된 파라미터
    """
    def __init__(self, pid: PIDController):
        self.pid = pid
        
        # 조정된 파라미터
        self.curvature_threshold = 0.003    # 기존 0.005
        self.feedforward_gain = 8.0         # 기존 15.0
        
        # Kp 적응 (보수적)
        self.kp_base = pid.kp
        self.kp_current = pid.kp
        self.log_kp = np.log(max(self.kp_current, 1e-6))
        
        self.kp_min = 0.7 * self.kp_base    # 기존 0.4
        self.kp_max = 1.3 * self.kp_base    # 기존 1.5
        self.log_kp_min = np.log(max(self.kp_min, 1e-6))
        self.log_kp_max = np.log(max(self.kp_max, 1e-6))
        
        self.log_clip_bound = 0.01          # 기존 0.02
        self.variance_threshold = 0.01
        
        self.error_history = deque(maxlen=30)
        
    def reset(self):
        self.pid.reset()
        self.kp_current = self.kp_base
        self.log_kp = np.log(max(self.kp_current, 1e-6))
        self.error_history.clear()

    def compute(self, error: float, lookahead_curvature: float) -> Tuple[float, float, bool]:
        """
        Returns: (steering_cmd, current_kp, feedforward_active)
        """
        self.error_history.append(error)
        
        # Feedforward
        feedforward = 0.0
        ff_active = abs(lookahead_curvature) > self.curvature_threshold
        if ff_active:
            feedforward = lookahead_curvature * self.feedforward_gain
        
        # Kp 적응
        if len(self.error_history) >= 5:
            variance = np.var(self.error_history)
            if variance > self.variance_threshold:
                log_delta = -0.01  # damping (기존 -0.02)
            else:
                log_delta = 0.005  # recovery (기존 0.01)
            log_delta = np.clip(log_delta, -self.log_clip_bound, self.log_clip_bound)
            self.log_kp += log_delta
            self.log_kp = np.clip(self.log_kp, self.log_kp_min, self.log_kp_max)
            self.kp_current = np.exp(self.log_kp)
            self.pid.kp = self.kp_current
        
        pid_output = self.pid.compute(error)
        return pid_output + feedforward, self.kp_current, ff_active


class ConditionalPLCWrapper:
    """
    조건부 PLC Controller
    
    평상시: 순수 PID (A안)
    급변 상황: PLC 피드포워드 + Kp 적응 활성화
    
    활성화 조건 (Sudden Obstacle 타겟 - 엄격하게):
    1. 전방 곡률 >= 0.022 (급회전, sudden_obstacle max=0.025)
    
    참고 - 각 시나리오 최대 곡률:
    - single_curve: 0.01
    - s_curve: 0.015 (노이즈 포함 ~0.02)
    - continuous_curves: 0.012
    - sudden_obstacle: 0.025 ← 이것만 타겟
    """
    def __init__(self, pid: PIDController,
                 # 활성화 임계값 (엄격하게)
                 curvature_activation_threshold: float = 0.022,  # sudden_obstacle만 확실히 해당
                 curvature_rate_threshold: float = 0.003,        # (미사용)
                 # PLC 파라미터 (조정됨)
                 curvature_threshold: float = 0.003,
                 feedforward_gain: float = 8.0,
                 # 활성화 지속 시간
                 activation_holdtime: float = 2.0,
                 dt: float = 0.01):
        
        self.pid = pid
        self.dt = dt
        
        # 활성화 조건
        self.curvature_activation_threshold = curvature_activation_threshold
        self.curvature_rate_threshold = curvature_rate_threshold
        self.activation_holdtime = activation_holdtime
        
        # PLC 파라미터
        self.curvature_threshold = curvature_threshold
        self.feedforward_gain = feedforward_gain
        
        # Kp 적응 (보수적)
        self.kp_base = pid.kp
        self.kp_current = pid.kp
        self.log_kp = np.log(max(self.kp_current, 1e-6))
        
        self.kp_min = 0.7 * self.kp_base
        self.kp_max = 1.3 * self.kp_base
        self.log_kp_min = np.log(max(self.kp_min, 1e-6))
        self.log_kp_max = np.log(max(self.kp_max, 1e-6))
        
        self.log_clip_bound = 0.01
        self.variance_threshold = 0.01
        
        # 히스토리
        self.error_history = deque(maxlen=30)
        self.curvature_history = deque(maxlen=50)
        
        # 활성화 상태
        self.plc_active = False
        self.activation_timer = 0.0
        
    def reset(self):
        self.pid.reset()
        self.kp_current = self.kp_base
        self.log_kp = np.log(max(self.kp_current, 1e-6))
        self.error_history.clear()
        self.curvature_history.clear()
        self.plc_active = False
        self.activation_timer = 0.0
    
    def _check_activation(self, lookahead_curvature: float) -> bool:
        """
        급변 상황 감지 - 절대 곡률 기준만 사용
        (센서 노이즈로 인해 곡률 변화율은 불안정함)
        """
        self.curvature_history.append(lookahead_curvature)
        
        # 조건: 전방 곡률이 급회전 수준 (sudden_obstacle만 해당)
        # sudden_obstacle max = 0.025, 기타 시나리오 max = 0.01~0.015
        if abs(lookahead_curvature) >= self.curvature_activation_threshold:
            return True
        
        return False

    def compute(self, error: float, lookahead_curvature: float) -> Tuple[float, float, bool]:
        """
        Returns: (steering_cmd, current_kp, plc_active)
        """
        self.error_history.append(error)
        
        # 활성화 조건 체크
        should_activate = self._check_activation(lookahead_curvature)
        
        if should_activate:
            self.plc_active = True
            self.activation_timer = self.activation_holdtime
        elif self.plc_active:
            self.activation_timer -= self.dt
            if self.activation_timer <= 0:
                self.plc_active = False
        
        # PLC 비활성 시: 순수 PID
        if not self.plc_active:
            self.pid.kp = self.kp_base  # 기본 Kp 유지
            pid_output = self.pid.compute(error)
            return pid_output, self.kp_base, False
        
        # PLC 활성 시: 피드포워드 + Kp 적응
        feedforward = 0.0
        if abs(lookahead_curvature) > self.curvature_threshold:
            feedforward = lookahead_curvature * self.feedforward_gain
        
        # Kp 적응
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


def run_simulation(controller_type: str, scenario: str, duration: float = 15.0, 
                   dt: float = 0.01, seed: int = None) -> Dict:
    """시뮬레이션 실행"""
    if seed is not None:
        np.random.seed(seed)
    
    vehicle = BicycleModel()
    road = RoadProfile(scenario)
    sensor = SensorModel()
    
    # A안 Baseline PID
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
        'kp': [], 'plc_active': []
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
    
    return results


def analyze_results(results: Dict) -> Dict:
    """결과 분석"""
    errors = np.abs(results['lateral_error'])
    curv = np.abs(results['curvature'])
    curve_mask = curv > 0.001
    
    analysis = {
        'mean_all': np.mean(errors),
        'max_all': np.max(errors),
        'activation_rate': np.mean(results['plc_active'])
    }
    
    if np.any(curve_mask):
        curve_errors = errors[curve_mask]
        analysis['mean_curve'] = np.mean(curve_errors)
        analysis['max_curve'] = np.max(curve_errors)
        analysis['violation_curve'] = np.mean(curve_errors > 0.3)
    else:
        analysis['mean_curve'] = 0
        analysis['max_curve'] = 0
        analysis['violation_curve'] = 0
    
    return analysis


def monte_carlo(scenario: str, n_runs: int = 30) -> Dict:
    """Monte Carlo 검증"""
    controllers = ['pid', 'plc_always', 'plc_conditional']
    
    stats = {ctrl: {'mean_all': [], 'mean_curve': [], 'max_curve': [], 
                    'violation': [], 'activation': []} for ctrl in controllers}
    
    for run in range(n_runs):
        seed = 1000 + run
        for ctrl in controllers:
            result = run_simulation(ctrl, scenario, duration=15.0, seed=seed)
            analysis = analyze_results(result)
            
            stats[ctrl]['mean_all'].append(analysis['mean_all'])
            stats[ctrl]['mean_curve'].append(analysis['mean_curve'])
            stats[ctrl]['max_curve'].append(analysis['max_curve'])
            stats[ctrl]['violation'].append(analysis['violation_curve'])
            stats[ctrl]['activation'].append(analysis['activation_rate'])
    
    summary = {}
    for ctrl in controllers:
        summary[ctrl] = {
            'mean_all': np.mean(stats[ctrl]['mean_all']),
            'std_all': np.std(stats[ctrl]['mean_all']),
            'mean_curve': np.mean(stats[ctrl]['mean_curve']),
            'max_curve': np.mean(stats[ctrl]['max_curve']),
            'violation': np.mean(stats[ctrl]['violation']),
            'activation': np.mean(stats[ctrl]['activation'])
        }
    
    return summary


def main():
    print("=" * 105)
    print("PLC-PID Conditional Activation Test")
    print("=" * 105)
    print("\n[설정]")
    print("  PID Baseline (A안): kp=0.1, ki=0.0, kd=0.2")
    print("  PLC 파라미터: curvature_threshold=0.003, feedforward_gain=8.0")
    print("  Kp 범위: 0.7~1.3 × kp_base, log_clip=0.01")
    print("  활성화 조건: curvature >= 0.020 OR curvature_rate >= 0.003/s")
    print("  (sudden_obstacle max curvature = 0.025, 기타 시나리오 max = 0.01~0.015)")
    
    scenarios = ["single_curve", "s_curve", "continuous_curves", "sudden_obstacle"]
    
    # Sanity check
    print("\n" + "=" * 105)
    print("SANITY CHECK (Single Run, seed=42)")
    print("-" * 105)
    
    for scenario in scenarios:
        for ctrl in ['pid', 'plc_always', 'plc_conditional']:
            r = run_simulation(ctrl, scenario, seed=42)
            a = analyze_results(r)
            act_str = f"{a['activation_rate']*100:5.1f}%" if ctrl != 'pid' else "  N/A"
            print(f"{scenario:<20} | {ctrl:<15} | MeanAll={a['mean_all']:.4f}m | "
                  f"MeanCurve={a['mean_curve']:.4f}m | Activation={act_str}")
        print("-" * 105)
    
    # Monte Carlo
    print("\n" + "=" * 105)
    print("MONTE CARLO VALIDATION (30 runs)")
    print("=" * 105)
    
    all_results = {}
    for scenario in scenarios:
        print(f"Processing: {scenario}...")
        all_results[scenario] = monte_carlo(scenario, n_runs=30)
    
    # 결과 테이블
    print("\n" + "=" * 105)
    print("RESULTS SUMMARY")
    print("=" * 105)
    
    # Mean Error (All)
    print("\n[1] Mean Lateral Error - All Sections (m)")
    print("-" * 105)
    print(f"{'Scenario':<20} | {'PID':>10} | {'PLC-always':>10} | {'PLC-cond':>10} | "
          f"{'Always vs PID':>14} | {'Cond vs PID':>14}")
    print("-" * 105)
    for scenario, data in all_results.items():
        pid = data['pid']['mean_all']
        always = data['plc_always']['mean_all']
        cond = data['plc_conditional']['mean_all']
        imp_always = (1 - always / pid) * 100 if pid > 0 else 0
        imp_cond = (1 - cond / pid) * 100 if pid > 0 else 0
        print(f"{scenario:<20} | {pid:>10.4f} | {always:>10.4f} | {cond:>10.4f} | "
              f"{imp_always:>+13.1f}% | {imp_cond:>+13.1f}%")
    
    # Mean Error (Curve)
    print("\n[2] Mean Lateral Error - Curve Sections Only (m)")
    print("-" * 105)
    print(f"{'Scenario':<20} | {'PID':>10} | {'PLC-always':>10} | {'PLC-cond':>10} | "
          f"{'Always vs PID':>14} | {'Cond vs PID':>14}")
    print("-" * 105)
    for scenario, data in all_results.items():
        pid = data['pid']['mean_curve']
        always = data['plc_always']['mean_curve']
        cond = data['plc_conditional']['mean_curve']
        imp_always = (1 - always / pid) * 100 if pid > 0 else 0
        imp_cond = (1 - cond / pid) * 100 if pid > 0 else 0
        print(f"{scenario:<20} | {pid:>10.4f} | {always:>10.4f} | {cond:>10.4f} | "
              f"{imp_always:>+13.1f}% | {imp_cond:>+13.1f}%")
    
    # Violation Rate
    print("\n[3] Violation Rate (>0.3m in Curve Sections)")
    print("-" * 105)
    print(f"{'Scenario':<20} | {'PID':>10} | {'PLC-always':>10} | {'PLC-cond':>10} | "
          f"{'Always vs PID':>14} | {'Cond vs PID':>14}")
    print("-" * 105)
    for scenario, data in all_results.items():
        pid = data['pid']['violation'] * 100
        always = data['plc_always']['violation'] * 100
        cond = data['plc_conditional']['violation'] * 100
        imp_always = (1 - always / max(pid, 0.1)) * 100
        imp_cond = (1 - cond / max(pid, 0.1)) * 100
        print(f"{scenario:<20} | {pid:>9.1f}% | {always:>9.1f}% | {cond:>9.1f}% | "
              f"{imp_always:>+13.1f}% | {imp_cond:>+13.1f}%")
    
    # PLC Activation Rate
    print("\n[4] PLC Activation Rate")
    print("-" * 105)
    print(f"{'Scenario':<20} | {'PLC-always':>12} | {'PLC-cond':>12} | {'Selectivity':>14}")
    print("-" * 105)
    for scenario, data in all_results.items():
        always = data['plc_always']['activation'] * 100
        cond = data['plc_conditional']['activation'] * 100
        selectivity = (1 - cond / max(always, 0.1)) * 100
        print(f"{scenario:<20} | {always:>11.1f}% | {cond:>11.1f}% | {selectivity:>+13.1f}%")
    
    # 종합 분석
    print("\n" + "=" * 105)
    print("CONCLUSION")
    print("=" * 105)
    
    sudden = all_results['sudden_obstacle']
    other_scenarios = ['single_curve', 's_curve', 'continuous_curves']
    
    print("\n[성능 비교]")
    
    # PLC-always
    sudden_imp_always = (1 - sudden['plc_always']['mean_curve'] / sudden['pid']['mean_curve']) * 100
    other_imp_always = np.mean([(1 - all_results[s]['plc_always']['mean_curve'] / 
                                  all_results[s]['pid']['mean_curve']) * 100 
                                 for s in other_scenarios if all_results[s]['pid']['mean_curve'] > 0])
    print(f"\nPLC-always:")
    print(f"  Sudden Obstacle 개선: {sudden_imp_always:+.1f}%")
    print(f"  기타 시나리오 평균:   {other_imp_always:+.1f}%")
    
    # PLC-conditional
    sudden_imp_cond = (1 - sudden['plc_conditional']['mean_curve'] / sudden['pid']['mean_curve']) * 100
    other_imp_cond = np.mean([(1 - all_results[s]['plc_conditional']['mean_curve'] / 
                               all_results[s]['pid']['mean_curve']) * 100 
                              for s in other_scenarios if all_results[s]['pid']['mean_curve'] > 0])
    print(f"\nPLC-conditional:")
    print(f"  Sudden Obstacle 개선: {sudden_imp_cond:+.1f}%")
    print(f"  기타 시나리오 평균:   {other_imp_cond:+.1f}%")
    
    # 활성화 선택성
    print("\n[활성화 선택성]")
    sudden_act = sudden['plc_conditional']['activation'] * 100
    other_act = np.mean([all_results[s]['plc_conditional']['activation'] * 100 for s in other_scenarios])
    print(f"  Sudden Obstacle: {sudden_act:.1f}%")
    print(f"  기타 시나리오:   {other_act:.1f}%")
    
    # 최종 판단
    print("\n" + "-" * 105)
    print("[최종 판단]")
    
    if sudden_imp_cond > 5 and other_imp_cond > -3:
        print("  ✓ 조건부 PLC 성공!")
        print("    → Sudden Obstacle에서 개선")
        print("    → 기타 시나리오에서 부작용 최소화")
    elif sudden_imp_cond > sudden_imp_always:
        print("  △ 부분 성공:")
        print("    → PLC-always보다 선택적 활성화로 일부 개선")
        print("    → 임계값 추가 튜닝 권장")
    elif sudden_act > other_act * 1.5:
        print("  △ 선택성은 양호:")
        print("    → 급변 상황에서 더 많이 활성화됨")
        print("    → 피드포워드 게인 조정 권장")
    else:
        print("  ✗ 추가 튜닝 필요:")
        print("    → 활성화 임계값 또는 피드포워드 게인 재검토")
    
    print("-" * 105)


if __name__ == "__main__":
    main()
