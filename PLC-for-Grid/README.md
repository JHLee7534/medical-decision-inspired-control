# ESS Frequency Control Simulation

저관성 전력계통에서 ESS(에너지저장장치)를 활용한 주파수 제어 시뮬레이션 프로그램입니다. PLC(Programmable Logic Controller) 기반 적응형 제어기와 기존 Droop 제어기의 성능을 비교합니다.

## 개요

재생에너지 비중이 높아지면서 전력계통의 관성이 감소하고, 이로 인해 주파수 변동이 심화되고 있습니다. 본 시뮬레이션은 ESS를 활용하여 주파수를 안정화하는 세 가지 제어 방식을 비교합니다.

| 제어기 | 설명 |
|--------|------|
| **Droop** | 고정 이득(K)을 사용하는 전통적 비례 제어 |
| **PLC-ESS** | 스트레스 기반으로 이득을 동적 조정하는 적응형 제어 |
| **Adaptive Droop** | 주파수 편차에 비례하여 이득을 조정하는 적응형 제어 |

## 요구사항

```
Python >= 3.8
numpy
matplotlib
```

설치:
```bash
pip install numpy matplotlib
```

## 실행 방법

```bash
python PLC_Grid.py
```

실행 시 다음 파일들이 스크립트와 같은 디렉토리에 생성됩니다:
- `comparison.png` : 시계열 비교 그래프 (주파수, 편차, ESS 출력, 이득 변화)
- `metrics.png` : 성능 지표 막대 그래프

## 시뮬레이션 결과

### 성능 비교 (Droop 대비 개선율)

| 지표 | PLC-ESS | Adaptive Droop |
|------|---------|----------------|
| RMS 주파수 편차 | **+46.4%** | +16.6% |
| Max 주파수 편차 | **+30.7%** | +18.5% |
| 95th 주파수 편차 | **+48.4%** | +19.0% |

PLC-ESS 제어기가 모든 지표에서 가장 우수한 성능을 보입니다.

## 주요 구성요소

### 1. 계통 파라미터 (`GridParams`)
- `f_nom`: 정격 주파수 (60 Hz)
- `M`: 관성 상수 (낮을수록 저관성 계통)
- `D`: 주파수 댐핑 계수
- `P_rated_ess`: ESS 정격 용량 (p.u.)

### 2. 재생에너지 변동 모델 (`RenewableDisturbance`)
풍력/태양광 출력 변동을 다음 요소로 모사:
- Ornstein-Uhlenbeck 프로세스 기반 느린 드리프트
- 고주파 노이즈
- 확률적 계단(step) 이벤트

### 3. 제어기

#### Droop 제어 (`ESSDroopController`)
```
P_ess = K_droop × (f_nom - f) × P_rated
```
고정 이득을 사용하는 가장 단순한 제어 방식입니다.

#### PLC-ESS 제어 (`ESSPLCController`)
핵심 특징:
- 주파수 편차로부터 스트레스(stress) 계산
- 지수이동평균(EMA)으로 스트레스 추적
- 스트레스가 임계값 초과 시 이득 증가 → 강한 대응
- 스트레스가 낮으면 기본값으로 서서히 복귀
- Log-domain bounded update로 안정성 보장

#### Adaptive Droop 제어 (`ESSAdaptiveDroopController`)
```
K = K_base + adapt_gain × |f_nom - f|
```
주파수 편차에 비례하여 이득을 즉각 조정합니다.

### 4. 계통 동특성 (`update_frequency`)
1차 스윙 방정식 기반:
```
df/dt = (P_ess - P_disturb - D×(f - f_nom)) / M
```

## 파라미터 조정

### 계통 조건 변경
```python
grid = GridParams(
    f_nom=60.0,
    M=3.0,      # 관성 상수 (낮추면 저관성 계통)
    D=1.0,      # 댐핑 계수
    P_rated_ess=1.0
)
```

### 외란 강도 변경
```python
renew = RenewableDisturbance(
    sigma_noise=0.08,      # 고주파 노이즈 강도
    drift_tau=40.0,        # 드리프트 시상수
    step_prob_per_sec=0.03, # 계단 이벤트 발생 확률
    step_mag_mean=0.4      # 계단 이벤트 평균 크기
)
```

### PLC 제어기 튜닝
```python
ESSPLCController(
    grid,
    K_init=0.6,           # 초기 이득
    K_min=0.3,            # 최소 이득
    K_max=3.0,            # 최대 이득
    theta=2.0,            # 이득 증가 민감도
    sensitivity=1000.0,   # 스트레스 변환 계수
    alpha_stress=0.4,     # EMA 계수 (클수록 빠른 반응)
    stress_threshold=0.3  # 스트레스 임계값
)
```

## 출력 그래프 설명

### comparison.png
1. **Frequency Response**: 세 제어기의 주파수 응답 비교
2. **Frequency Deviation**: 정격 주파수 대비 편차 (mHz)
3. **ESS Power Output**: ESS 출력 (p.u.)
4. **Controller Gain**: 제어기 이득(K) 변화 추이

### metrics.png
RMS, Max, 95th percentile 주파수 편차를 막대 그래프로 비교

## 라이선스

MIT License
