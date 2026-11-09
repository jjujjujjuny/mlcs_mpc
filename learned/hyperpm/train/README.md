# hpm — 우리 데이터로 HyperPM 학습

저자 `main.py` 를 고치지 않고 차량 값과 데이터 경로만 얹어 돌린다.

| 파일 | 역할 |
|---|---|
| `train.sh` | 실행. 뒤 인자는 hydra 로 넘어간다 |
| `run_main.py` | 저장소에 없는 `robot_model.acrobot` import 를 빈 모듈로 채워 `main.py` 실행 |
| `mlcs_car_model.py` | 질량 3.284, I_z 0.089, lr 0.1537, NaN 방지 타이어 식, `lr` 고정 상수 모델 |
| `conf/` | 데이터 경로(`~/mlcs_data/hpm`), 모델, 신경망 설정 |

## 메인 PC 준비

```bash
git clone https://github.com/hyper-mpc/hypermpc_code.git ~/hypermpc_code
python3 -m venv ~/hpm-venv
~/hpm-venv/bin/pip install "numpy<2" pandas scipy matplotlib torch hydra-core \
    hydra-joblib-launcher wandb torch-interpol tqdm
mkdir -p ~/mlcs_data/hpm
tar -xzf learned/hyperpm/train/data/hpm_data_20261107.tar.gz -C ~/mlcs_data/hpm
```

## 실행

```bash
learned/hyperpm/train/train.sh hpm.train.delta_p_L2_loss=0.2 hpm.train.epoch=1 hpm.device=cuda   # 먼저 1 에폭
learned/hyperpm/train/train.sh hpm.train.delta_p_L2_loss=0.2 hpm.device=cuda                     # 2000 에폭
```

신경망 설정은 저자 `conf_mpc/config_car.yaml:60` 의 `8 ctrl points 0.5 sigma l 0.2 THE USED ONE` 에 맞췄다.
제어점 8, `pred_noise_sigma` 0.5 는 `conf/hmodel/mlcs_hyper.yaml` 에 있고 `delta_p_L2_loss` 0.2 는 위 인자로 준다.

결과는 `~/mlcs_data/hpm/runs/` 아래 `data/models/` 와 `outputs/`.

### 현재 메인 PC (Ubuntu 20.04, RTX 3080, 드라이버 470)

위 명령 그대로는 안 된다. 2026-11-04 에 이렇게 1 에폭을 끝까지 돌렸다.

```bash
# python3-venv 가 없으면 conda 로 같은 경로에 만든다
conda create -y -p ~/hpm-venv python=3.10 pip
# 드라이버 470 은 CUDA 11.x 까지라 cu118 빌드를 쓴다 (torch 2.7.1+cu118 확인)
~/hpm-venv/bin/pip install --index-url https://download.pytorch.org/whl/cu118 torch
~/hpm-venv/bin/pip install "numpy<2" pandas scipy matplotlib hydra-core \
    hydra-joblib-launcher wandb torch-interpol tqdm
# compile 을 켜면 Triton Error: device kernel image is invalid
learned/hyperpm/train/train.sh hpm.train.delta_p_L2_loss=0.2 hpm.train.epoch=1 hpm.device=cuda hpm.compile=false
```

- 1 에폭 182 s. 저자 설정 2000 에폭이면 약 4.2 일이다.
- 중간 저장은 `hpm.wandb.send_interval` 마다다 (기본 200). 짧게 돌릴 때는 줄인다.
- 미래 입력 경로는 **에폭 1부터** 켜진다. 1 에폭 실행으로는 이 경로를 검증하지 못한다.

★ `train.sh` 의 `WANDB_MODE=disabled` 를 빼지 말 것. 저자 `main.py` 에 저자 계정
API 키가 들어 있어서, 켜면 우리 로그와 모델이 그 계정으로 올라간다.

## 고정 파라미터 동정 (신경망 없이 상수만)

약 1시간 설정이다. 윈도 간격을 8 로 넓혀 메인 PC 기준 에폭당 약 2.5분(추정)으로 맞췄다.
저자 학습률 5e-4 로는 이 스텝 수에서 파라미터가 많아야 2배쯤 움직이므로 올린다.

```bash
tar -xzf learned/hyperpm/train/data/hpm_data_20261107.tar.gz -C ~/mlcs_data/hpm   # 기존 csv 를 덮어쓴다
learned/hyperpm/train/train.sh hmodel=const hpm.device=cuda hpm.compile=false \
    hpm.train.epoch=25 hpm.optimizer.lr=0.002 hpm.wandb.send_interval=5 \
    dataset.train_dataset.stride=8 dataset.val_dataset.stride=8
```

- `lr` 을 저울 실측값으로 묶으려면 `hmodel=const` 대신 `hmodel=const_fixed_lr` 를 쓴다.
- 결과 파라미터는 로그의 `p_const` (에폭마다 출력). 순서는 `I_z, lr, Cd0, Cd2, Cd1, I_e, K_fi, b1, b0, R`, 앞 타이어 10개, 뒤 타이어 10개다.
- 모델은 `~/mlcs_data/hpm/runs/data/models/<실행이름>/`.
- `val_dataset.stride` 는 100 이하여야 한다 (`main.py` 가 `100 // stride` 로 슬라이스).
- 젯슨 CPU 에서 배치 32, 2 에폭으로 `main.py` 끝까지 정상 종료를 확인했다. 전체 설정은 아직 안 돌렸다.
- 이 실행은 HyperPM 기본값을 얻는 용도다. `stride` 와 학습률을 바꿨으니 논문의 `const_l` 기준선으로는 쓰지 말고, 기준선은 HyperPM 과 같은 조건으로 다시 돌린다.
- 결과를 HyperPM 에 넘길 때 `pretrain_init_model_path` 는 쓰지 않는다 (wandb 아티팩트가 필요하고 `free_params` 를 안 넘긴다). `mlcs_car_model.py` 기본값에 직접 넣는다.

## 데이터 만들기 (젯슨)

```bash
./drive.sh cleanlog   # mocap 끊김·튐 제거 → ~/mlcs_data/clean
./drive.sh hpmdata    # 저자 형식 → ~/mlcs_data/hpm
```

변환 내용은 `tools/analysis/prep_hpm_dataset.py` 머리말 참고.

## 데이터 (hpm_data_20261107)

전진만 44분 (train 30.5 / val 8.7 / test 4.8).

- `run_id` < 10000: 이전 `hpm_data_20261105` 그대로 (11-03~11-05 사람 주행, 33분). 이전 수치와 비교하려면 이것만 쓴다
- `run_id` ≥ 10000: 새로 더한 10.8분. 11-07 저속 원선회 (0.6, 0.7, 0.8 m/s × 조향 ±0.20, ±0.30, ±0.38, 조향 계단 약 10회)와 `hmpc` 자율주행 4개 (11-05 `202948`, 11-06 `161526`, `162811`, `172147`). 같은 규칙으로 따로 나눠 붙였다 (seed 7)
- 0.5~0.8 m/s 비율이 2.7% → 25.8% 가 됐다. MPC 가 0.7 m/s 로 달리는 구간이다

`lr` 0.1537, 마찰 0.41. 오프셋 변경 전 로그는 조향과 yaw 를 보정했고, 전 로그에 조향 맵을 적용했다 (`tools/analysis/prep_hpm_dataset.py`).

## 아직 안 정한 것

- **`v_y` 기준점.** 모델 정의대로 무게중심으로 옮겼다 (`v_y += lr·r`). 그런데 `v_y`~`r`
  기울기는 뒤축 기준일 때 저자 데이터와 더 가깝다 (`|ay|` 3~6 에서 저자 -0.241,
  뒤축 -0.277, 무게중심 -0.127). `prep_hpm_dataset.py --lr 0` 으로 만들어 검증 손실을
  비교해 볼 것.
- **SG 설정.** 저자 데이터에서 역산한 값(창 31, 2차)이고 잔차가 남는다 (v_x RMSE 0.14 m/s).
