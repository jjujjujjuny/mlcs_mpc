#!/usr/bin/env bash
# HyperPM 학습 — 저자 main.py 를 고치지 않고 우리 설정만 얹어 돌린다.
#
#   hypermpc/hpm/train.sh                        기본 (저자 설정 2000 에폭)
#   hypermpc/hpm/train.sh hpm.train.epoch=3      뒤 인자는 hydra 로 넘어간다
#
# ★ WANDB_MODE=disabled 는 빼지 말 것. 저자 main.py 에 저자 계정 API 키가
#   박혀 있어서, 켜면 우리 로그·모델이 그 계정으로 올라간다.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HPM_CODE="${HPM_CODE:-$HOME/hypermpc_code}"
OUT="${HPM_OUT:-$HOME/mlcs_data/hpm/runs}"
export WANDB_MODE=disabled
export PYTHONPATH="$HERE:$HPM_CODE:$PYTHONPATH"
mkdir -p "$OUT"
cd "$OUT"
exec "${HPM_PY:-$HOME/hpm-venv/bin/python}" "$HERE/run_main.py" \
    --config-dir "$HERE/conf" \
    hmodel=mlcs_hyper dmodel=mlcs_car dataset=mlcs_car \
    hmodel/time_series_encoder=gru \
    "$@"
