"""
우리 차의 단일 트랙 모델 — 저자 PacejkaTiresSingleTrack 에서 차량 값만 바꾼다.

  저자 저장소를 고치지 않으려고 상속한다. 값은 config/vehicle.yaml 과 같다.
  L 은 저자 차도 0.33 m 라 그대로다.

  main.py 가 free_params 를 넘기므로 hmodel 은 그것을 받는 CausalHyperModel2p
  를 쓴다. free_params 가 비어 있으면 저자 차 설정의 CausalHyperModel2 와 같은
  계산이다 (conf/hmodel/mlcs_hyper.yaml).

  ★ 저자 base_tire_model 의 뒷바퀴 슬립각은 lr 자리에 lf 를 쓴다 (저자 FIXME).
    저자 결과와 비교하려고 고치지 않고 둔다.

  ★ 저자 타이어 식은 슬립이 정확히 0 이면 0/0 이 되어 NaN 이 난다. 조향이 정확히 0
    인 직진에서 v_y, r 이 0 으로 줄고 v_x 가 바퀴 속도와 같아지면 실제로 생긴다
    (2026-11-05, 고정 파라미터 동정이 첫 에폭에 NaN 으로 멈춤). SafePacejkaTireModel
    이 막는다. 슬립이 있을 때의 값은 사실상 그대로다.
"""
import torch

from robot_model.car.single_track import PacejkaTiresSingleTrack
from robot_model.car.pacejka_params import PacejkaParameters
from robot_model.car.single_track_params import VehicleParameters
from robot_model.car.pacejka_tire_model import PacejkaTireModel
from hyper_prediction_models.const_param_model import ConstParamModel

MASS = 3.284    # 저울 실측 2026-11-05 (라이다 포함)
I_Z = 0.089     # 추정 — docs/CALIBRATION.md
LR = 0.1537     # 앞/뒤 축 하중으로 계산


class ConstParamFixedLr(ConstParamModel):
    """저자 상수 모델에서 lr(인덱스 1)만 기본값(저울 실측)으로 묶는다.

    상수 모델을 풀어 두면 lr 이 0.193 까지 가서 실측과 어긋났다 (2026-11-05).
    AdamW 의 weight decay 가 grad 와 무관하게 값을 옮기므로 매번 되돌린다.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.register_buffer('lr_log', self.p[..., 1].detach().clone())

    def forward(self, M, P=None, enable_prediction_head=True):
        with torch.no_grad():
            self.p[..., 1] = self.lr_log
        return super().forward(M, P, enable_prediction_head)


class SafePacejkaTireModel(PacejkaTireModel):
    """저자 tire_forces_model 과 같고, S_resultant 에 EPS 만 더했다."""

    EPS = 1e-12

    @staticmethod
    def tire_forces_model(slip_angle_rad, slip_ratio, wp):
        slip_angle_deg = slip_angle_rad * 180.0 / torch.pi

        Sx_norm = slip_ratio / wp.Sx_p
        Alpha_norm = slip_angle_deg / wp.Alpha_p

        S_resultant = torch.sqrt(
            Sx_norm**2 + Alpha_norm**2 + SafePacejkaTireModel.EPS)

        Sx_mod = S_resultant * wp.Sx_p
        Alpha_mod = S_resultant * wp.Alpha_p

        Fy = ((Alpha_norm / S_resultant) * wp.Dy * torch.sin(wp.Cy * torch.atan(
            (wp.By * Alpha_mod) - wp.Ey * -1.0
            * (wp.By * Alpha_mod - torch.atan(wp.By * Alpha_mod)))))

        Fx = ((Sx_norm / S_resultant) * wp.Dx * torch.sin(wp.Cx * torch.atan(
            (wp.Bx * Sx_mod) - wp.Ex * -1.0
            * (wp.Bx * Sx_mod - torch.atan(wp.Bx * Sx_mod)))))

        return Fx, - Fy


class MlcsSingleTrack(PacejkaTiresSingleTrack):
    def __init__(self, init_params=None) -> None:
        super().__init__(init_params=init_params)
        self.tire_model = SafePacejkaTireModel()
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
