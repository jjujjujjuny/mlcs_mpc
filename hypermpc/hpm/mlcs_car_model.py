"""
우리 차의 단일 트랙 모델 — 저자 PacejkaTiresSingleTrack 에서 차량 값만 바꾼다.

  저자 저장소를 고치지 않으려고 상속한다. 값은 config/vehicle.yaml 과 같다.
  L 은 저자 차도 0.33 m 라 그대로다.

  main.py 가 free_params 를 넘기므로 hmodel 은 그것을 받는 CausalHyperModel2p
  를 쓴다. free_params 가 비어 있으면 저자 차 설정의 CausalHyperModel2 와 같은
  계산이다 (conf/hmodel/mlcs_hyper.yaml).

  ★ 저자 base_tire_model 의 뒷바퀴 슬립각은 lr 자리에 lf 를 쓴다 (저자 FIXME).
    저자 결과와 비교하려고 고치지 않고 둔다.
"""
import torch

from robot_model.car.single_track import PacejkaTiresSingleTrack
from robot_model.car.pacejka_params import PacejkaParameters
from robot_model.car.single_track_params import VehicleParameters

MASS = 3.097    # 저울 실측 2026-10-02
I_Z = 0.084     # 추정 — docs/CALIBRATION.md
LR = 0.1498     # 앞/뒤 축 하중으로 계산


class MlcsSingleTrack(PacejkaTiresSingleTrack):
    def __init__(self, init_params=None) -> None:
        super().__init__(init_params=init_params)
        self.vehicle_parameters.m.fill_(MASS)

    @staticmethod
    def get_default_params_static(batch_size=1):
        vehicle = VehicleParameters.default_params_tensor(batch_size)
        vehicle[..., 0] = I_Z
        vehicle[..., 1] = LR
        return torch.cat([
            vehicle,
            PacejkaParameters.default_params_tensor(batch_size),
            PacejkaParameters.default_params_tensor(batch_size),
        ], dim=-1)

    def positive_params(self):
        return torch.arange(self.get_default_params_static().shape[-1])

    def free_params(self):
        # main.py 가 요구한다. 저자 차 모델에는 없다. None 을 넘기면 2p 모델이
        # 파라미터 전체를 신경망 출력으로 덮어쓰므로 빈 텐서를 준다.
        return torch.tensor([], dtype=torch.long)
