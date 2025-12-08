# PLC-V2: 전력망 주파수 제어를 위한 2단계 적응형 ESS 제어기

## 개요

PLC-V2 (Proactive Latency Control Version 2)는 저관성 전력계통에서 ESS(Energy Storage System)의 주파수 조정 성능을 향상시키기 위한 적응형 제어 알고리즘입니다. 기존 PLC의 안정성과 Adaptive Droop의 반응성을 결합한 **2단계 모드 전환** 방식을 채택하여, 다양한 외란 조건에서 최적의 성능을 제공합니다.

## 핵심 아이디어

```
┌─────────────────────────────────────────────────────────────────┐
│                      PLC-V2 동작 원리                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   주파수 편차 측정                                               │
│         │                                                       │
│         ▼                                                       │
│   ┌─────────────────────────────────────┐                       │
│   │        모드 판단 (히스테리시스)       │                       │
│   │  |freq_dev| > 60mHz → 비상 모드     │                       │
│   │  |freq_dev| < 30mHz → 정상 모드     │                       │
│   └─────────────────────────────────────┘                       │
│         │                                                       │
│    ┌────┴────┐                                                  │
│    ▼         ▼                                                  │
│ ┌──────┐  ┌──────┐                                              │
│ │ 정상 │  │ 비상 │                                              │
│ │ 모드 │  │ 모드 │                                              │
│ │(PLC) │  │(Adap)│                                              │
│ └──────┘  └──────┘                                              │
│    │         │                                                  │
│    ▼         ▼                                                  │
│ Log-bounded  즉각 반응                                          │
│ K 업데이트   K 계산                                             │
│ (±5%/step)  (비례 증가)                                         │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 정상 모드 (PLC 방식)
- **조건**: |주파수 편차| < 30mHz
- **K 업데이트**: Log-bounded (±5%/step 제한)
- **특징**: 노이즈 필터링, 안정적 적응, 과응답 방지

### 비상 모드 (Adaptive 방식)
- **조건**: |주파수 편차| > 60mHz
- **K 계산**: `K = K_init + adapt_gain × |f_nom - f|`
- **특징**: 즉각 반응, 빠른 주파수 복구

## 시뮬레이션 결과

### 시나리오 1: 대규모 외란 (발전기 탈락)
t=10초에 0.5 p.u. 발전기 탈락 시뮬레이션 (60초간)

| 제어기 | RMS 편차 (mHz) | Max 편차 (mHz) | ESS 사용량 (p.u.) | 개선율 |
|--------|:--------------:|:--------------:|:-----------------:|:------:|
| Droop | 278.51 | 313.68 | 0.1653 | - |
| Adaptive | 151.23 | 167.79 | 0.2988 | +45.7% |
| PLC-Original | 181.19 | 201.21 | 0.2678 | +34.9% |
| **PLC-V2** | **151.25** | **167.79** | 0.2988 | **+45.7%** |

**분석**: 대규모 외란에서 PLC-V2는 비상 모드로 전환하여 Adaptive와 동등한 성능 달성

### 시나리오 2: 정상 외란 (재생에너지 변동)
300초간 연속적인 재생에너지 출력 변동

| 제어기 | RMS 편차 (mHz) | Max 편차 (mHz) | ESS 사용량 (p.u.) | 개선율 |
|--------|:--------------:|:--------------:|:-----------------:|:------:|
| Droop | 32.27 | 70.40 | 0.0192 | - |
| Adaptive | 26.92 | 57.33 | 0.0254 | +16.6% |
| PLC-Original | 22.38 | 49.64 | 0.0307 | +30.6% |
| **PLC-V2** | **19.92** | **48.62** | 0.0340 | **+38.3%** |

**분석**: 정상 외란에서 PLC-V2가 모든 제어기 중 최고 성능 (Log-bounded의 노이즈 필터링 효과)

### 시나리오 3: 통계적 검증 (20개 시드)
다양한 난수 시드로 재생에너지 변동 시뮬레이션

| 제어기 | 평균 RMS (mHz) | 표준편차 | 최소 | 최대 |
|--------|:--------------:|:--------:|:----:|:----:|
| Droop | 27.13 | 4.19 | 19.74 | 36.02 |
| Adaptive | 22.74 | 3.01 | 17.45 | 29.52 |
| PLC-Original | 19.48 | 2.32 | 15.79 | 24.76 |
| **PLC-V2** | **17.76** | **1.85** | **15.02** | **22.16** |

**분석**: PLC-V2는 평균 성능과 안정성(낮은 표준편차) 모두에서 최고

### 시나리오 4: 혼합 구성 (점진적 도입)
기존 Droop 시스템에 PLC-V2를 점진적으로 도입하는 시나리오

| 구성 | RMS 편차 (mHz) | 개선율 |
|------|:--------------:|:------:|
| All Droop (3개) | 32.27 | - |
| Mixed (Droop + Adaptive + PLC-V2) | 24.79 | +23.2% |
| All PLC-V2 (3개) | 19.92 | +38.3% |

**분석**: 혼합 구성에서도 효과적, 기존 시스템에 점진적 도입 가능

## 제어기 비교

| 특성 | Droop | Adaptive | PLC-Original | PLC-V2 |
|------|:-----:|:--------:|:------------:|:------:|
| K 변화 | 고정 | 즉시 변화 | 점진적 (±5%) | **상황별 적응** |
| 노이즈 내성 | 없음 | 약함 | 강함 | **강함** |
| 대규모 외란 대응 | 부족 | 우수 | 부족 | **우수** |
| 안정성 | ◎ | △ | ◎ | **◎** |
| 구현 복잡도 | 낮음 | 낮음 | 중간 | 중간 |

## 파라미터 설명

### GridParams (전력계통)
```python
f_nom = 60.0    # 정격 주파수 [Hz]
M = 3.0         # 관성 상수 [s] (저관성 계통)
D = 1.0         # 댐핑 계수
```

### ESSPLC_V2 파라미터
```python
K_init = 0.6              # 초기 게인
K_min = 0.3               # 최소 게인
K_max = 2.0               # 최대 게인
theta = 1.0               # 스트레스 민감도
sensitivity = 1000.0      # 주파수 편차 → 스트레스 변환 계수
alpha_stress = 0.4        # 스트레스 EMA 계수
recovery_rate = 0.02      # 복귀 속도
stress_threshold = 0.3    # 스트레스 임계값
emergency_freq_dev = 0.001  # 비상 모드 진입 임계값 (60mHz)
adapt_gain = 10.0         # 비상 모드 적응 게인
```

## 사용법

### 기본 실행
```bash
python PLC_Grid_v_2.py
```

### 단일 ESS 생성
```python
from PLC_Grid_v_2 import ESSPLC_V2, GridParams

# ESS 생성 (정격 용량 0.33 p.u.)
ess = ESSPLC_V2(P_rated=0.33)

# 제어 계산
freq = 59.95  # 현재 주파수
P_output, K = ess.compute(freq, f_nom=60.0)
```

### 다중 ESS 시뮬레이션
```python
from PLC_Grid_v_2 import (
    ESSPLC_V2, GridParams, SimParams,
    RenewableDisturbance, run_multi_ess_simulation
)

# 파라미터 설정
grid = GridParams(f_nom=60.0, M=3.0, D=1.0)
sim = SimParams(dt=0.1, t_end=300.0, seed=123)

# 3개의 ESS (각 0.33 p.u.)
ess_list = [ESSPLC_V2(P_rated=0.33) for _ in range(3)]

# 외란 생성
P_disturb = RenewableDisturbance().generate(sim)

# 시뮬레이션 실행
results = run_multi_ess_simulation(ess_list, grid, sim, P_disturb)
```

## 출력 파일

실행 시 다음 그래프가 자동 생성됩니다:
- `scenario1_gen_trip.png`: 발전기 탈락 시나리오 결과
- `scenario2_renewable.png`: 재생에너지 변동 시나리오 결과

## 알고리즘 상세

### 모드 전환 로직 (히스테리시스)
```python
if freq_dev > emergency_freq_dev:        # > 60mHz
    mode = "emergency"
elif freq_dev < emergency_freq_dev * 0.5:  # < 30mHz
    mode = "normal"
# else: 현재 모드 유지 (채터링 방지)
```

### 정상 모드 K 업데이트
```python
# 스트레스 EMA 계산
stress_ema = α × stress_instant + (1-α) × stress_ema

if stress_ema > threshold:
    # Log-bounded 증가
    f_r = 1.0 + θ × (stress_ema - threshold)
    delta_log = clip(log(f_r), -0.05, 0.05)
    log_K += delta_log
else:
    # 기본값 복귀
    log_K → log(K_init)

K = clip(exp(log_K), K_min, K_max)
```

### 비상 모드 K 계산
```python
K = K_init + adapt_gain × |f_nom - f|
K = clip(K, K_min, K_max)
```

## 한계 및 향후 연구

### 현재 한계
1. **시뮬레이션 기반**: 실계통 검증 필요
2. **단순화된 모델**: 실제 ESS의 SOC, 효율 등 미반영
3. **통신 지연 미고려**: 실시간 제어 시 지연 영향 분석 필요

### 향후 연구 방향
1. 실계통 데이터 기반 검증
2. ESS 수명 영향 분석 (충방전 사이클)
3. 통신 지연 및 측정 노이즈 영향 분석
4. 다양한 관성 조건에서의 성능 평가
5. MPC, LQR 등 고급 제어기와의 비교 연구

## 결론

PLC-V2는 기존 Droop 대비 **+38.3%** 성능 개선을 달성하며, 다음과 같은 장점을 제공합니다:

| 시나리오 | PLC-V2 성능 |
|---------|:----------:|
| 대규모 외란 (발전기 탈락) | Adaptive와 동등 ✓ |
| 정상 외란 (재생에너지) | **최고 성능** ✓ |
| 다중 ESS 협조 | 문제 없음 ✓ |
| 혼합 구성 (점진 도입) | 효과적 ✓ |

**추가 연구 및 실계통 검증 가치가 있는 알고리즘입니다.**

## 참고문헌

- Lee, J.H. (2025). "Proactive Latency Control: Robust Dual-Loop Adaptation for Predictably Uncertain LEO Networks"
- 전력계통 주파수 제어 관련 IEEE/IEC 표준

## 라이선스

MIT License

## 연락처

질문 및 피드백은 이슈를 통해 남겨주세요.
