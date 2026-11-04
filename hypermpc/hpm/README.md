# hpm — 우리 데이터로 HyperPM 학습

저자 `main.py` 를 고치지 않고 차량 값과 데이터 경로만 얹어 돌린다.

| 파일 | 역할 |
|---|---|
| `train.sh` | 실행. 뒤 인자는 hydra 로 넘어간다 |
| `run_main.py` | 저장소에 없는 `robot_model.acrobot` import 를 빈 모듈로 채워 `main.py` 실행 |
| `mlcs_car_model.py` | 질량 3.097, I_z 0.084, lr 0.1498 |
| `conf/` | 데이터 경로(`~/mlcs_data/hpm`), 모델, 신경망 설정 |

## 메인 PC 준비

```bash
git clone https://github.com/hyper-mpc/hypermpc_code.git ~/hypermpc_code
python3 -m venv ~/hpm-venv
~/hpm-venv/bin/pip install "numpy<2" pandas scipy matplotlib torch hydra-core \
    hydra-joblib-launcher wandb torch-interpol tqdm
mkdir -p ~/mlcs_data/hpm
tar -xzf hypermpc/hpm/data/hpm_data_20261104.tar.gz -C ~/mlcs_data/hpm
```

## 실행

```bash
hypermpc/hpm/train.sh hpm.train.epoch=1 hpm.device=cuda   # 먼저 1 에폭
hypermpc/hpm/train.sh hpm.device=cuda                     # 저자 설정 2000 에폭
```

결과는 `~/mlcs_data/hpm/runs/` 아래 `data/models/` 와 `outputs/`.

★ `train.sh` 의 `WANDB_MODE=disabled` 를 빼지 말 것. 저자 `main.py` 에 저자 계정
API 키가 들어 있어서, 켜면 우리 로그와 모델이 그 계정으로 올라간다.

## 데이터 만들기 (젯슨)

```bash
./drive.sh cleanlog   # mocap 끊김·튐 제거 → ~/mlcs_data/clean
./drive.sh hpmdata    # 저자 형식 → ~/mlcs_data/hpm
```

변환 내용은 `tools/prep_hpm_dataset.py` 머리말 참고.

## 아직 안 정한 것

- **`v_y` 기준점.** 모델 정의대로 무게중심으로 옮겼다 (`v_y += lr·r`). 그런데 `v_y`~`r`
  기울기는 뒤축 기준일 때 저자 데이터와 더 가깝다 (`|ay|` 3~6 에서 저자 -0.241,
  뒤축 -0.277, 무게중심 -0.127). `prep_hpm_dataset.py --lr 0` 으로 만들어 검증 손실을
  비교해 볼 것.
- **SG 설정.** 저자 데이터에서 역산한 값(창 31, 2차)이고 잔차가 남는다 (v_x RMSE 0.14 m/s).
