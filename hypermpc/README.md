# hypermpc — HyperMPC 정식화를 이 차에 올리기

HyperMPC 논문([arXiv:2508.06181](https://arxiv.org/abs/2508.06181))의 정식화를
우리 차량에 적용하는 작업. 원본 코드는
[hyper-mpc/hypermpc_code](https://github.com/hyper-mpc/hypermpc_code).

## 지금 어디까지

| 단계 | 상태 |
|---|---|
| pip + casadi (venv) | ✅ casadi 3.8.1 |
| acados 소스 빌드 (aarch64) | ✅ 최소 OCP 0.659 ms |
| **오프라인 OCP** | ⏳ **코드 생성·빌드까지 확인, 풀이 미검증** |
| 트랙 변환 (track_5 → Frenet) | 대기 |
| ROS 노드 | 대기 |
| 파라미터 동정 | 대기 |
| HyperPM (torch + l4casadi) | 대기 |

## 실행

```bash
source ~/hmpc-venv/bin/activate
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib

git clone https://github.com/hyper-mpc/hypermpc_code.git ~/hypermpc_code
python3 hypermpc/offline_ocp.py --hypermpc ~/hypermpc_code
```

**첫 빌드는 10분을 넘긴다.** Pacejka 가 들어간 9상태 모델에 `EXACT` 헤시안이라
심볼릭 미분이 무겁다. 생성된 코드는 남으므로 두 번째부터는 빠르다.
(2026-09-30 시도: `.so` 까지 나왔고 10분 제한에 잘렸다 — 실패가 아니라 시간 초과)

## 왜 이 스크립트가 필요한가

`docs/mpc_formulation_car.py` 는 원본의 `mpc/mpc_formulation_car.py` **한 파일**이고,
동역학·비용·파라미터를 전부 `model` 객체에서 받아쓴다. **그 model 파일이 없다.**

그래서 여기서는 정식화를 직접 세운다. 동역학만 원본
`robot_model/car/casadi_car_model_naive.py` 를 그대로 import 한다 —
`casadi` + `numpy` 만 쓰는 유일한 경로다 (다른 모델 파일은 `torch` 와
`l4casadi` 를 import 한다).

## 상태·입력

```
x = [ s, n, mu,            Frenet 포즈 (진행, 횡편차, 헤딩편차)
      vx, vy, r, friction, 동역학  ← naive 모델
      wheel_speed, delta ] 액추에이터 (1차 지연)     → 9
u = [ wheel_speed_ref, delta_ref ]                   → 2
p = [ 차량 10 + 앞타이어 10 + 뒤타이어 10 + 곡률 1 ] → 31
```

파라미터 기본값은 원본 `single_track_params.py` / `pacejka_params.py` 에서 가져왔다.

## 지금 MPC 와의 차이

|  | 현재 `mpc_node` | 이 정식화 |
|---|---|---|
| 상태 | 운동학 자전거 | 9개 (동역학 + Pacejka) |
| 경로 | 참조 궤적 추종 | Frenet 컨투어링 |
| 예측 | N=20, dt=0.05 → 1.0 s | N=80, dt=0.033 → 2.64 s |
| 솔버 | iLQR (numpy) | acados SQP_RTI + HPIPM |
| 트랙 | x,y 중심선 | Frenet + 경계 폭 |

제어 아키텍처 전환이지 솔버 교체가 아니다.

## ⚠ 아직 논문 차량 값이다

`casadi_car_model_naive.py` 에 `m = 5.1`, `L = 0.33` 이 하드코딩돼 있다.
휠베이스는 우리와 같지만 **질량은 다르다** (우리 `vehicle.yaml` 은 3.5 로
가정 — 미측정). 저울에 올려 재고 바꿔야 예측이 맞는다.

`I_z`, `lr`, Pacejka 계수도 전부 논문 차량 값이다.

## 막힐 만한 곳

- **학습 가중치를 못 받을 수 있다.** 원본 `conf_mpc/config_car.yaml` 이
  W&B 아티팩트(`hpm_car/model:v23679`)를 가리키는데 저자들 프로젝트라
  접근이 안 될 가능성이 높다. 그러면 직접 학습해야 하고, 그것이 36분
  데이터가 필요한 이유다 (`docs/ROADMAP.md` 2단계).
- **l4casadi** 가 가장 어려운 부품이다 — libtorch C++ 에 대해 컴파일한다.
  다행히 위 표의 "HyperPM" 단계 전까지는 필요 없다.
