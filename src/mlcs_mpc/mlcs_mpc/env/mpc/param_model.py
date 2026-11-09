"""
param_model — MPC 의 스테이지별 동역학 파라미터 p[30] 을 주는 쪽. 학습 모델은 여기에 붙는다.

  기본 (상수) 은 여기 있고, 학습 모델 구현은 저장소 최상위 learned/ 에 있다 (쓸 때만 import 한다).
  MPC (env/mpc/worker.py) 는 매 풀이마다 이것만 부른다:

    pm.observe(t, x)              지금 상태 (OCP x[9]) 를 기록에 넣는다
    P, w = pm.stage_params(t, x)  (N+1, 30) 스테이지별 파라미터와 학습 모델을 쓴 비율 w (0~1)
    pm.set_plan(t, xs)            이번 해의 예측 궤적 (N+1, 9) — 다음 예측의 입력이 될 수 있다
    pm.clear_plan()               해가 비정상이거나 초기 추측을 다시 깔았을 때

  기본은 ConstantParams (config/mpcc.yaml 의 값을 모든 스테이지에). 학습 모델은 같은 네 메서드를 가진
  클래스를 만들고 load() 에 이름을 하나 더하면 된다.

  설정 (mpcc.yaml)  param_model: ''                         상수
                    param_model: 'hyperpm:config/hyper/J_ep10.npz'   HyperPM (learned/hyperpm)
                    param_model_v_min, param_model_blend    학습 모델을 쓰기 시작하는 속도와 섞는 폭
"""
import numpy as np

NP = 30

class ConstantParams:
    name = 'constant'

    def __init__(self, params, N):
        self.P = np.tile(np.asarray(params, dtype=float), (N + 1, 1))

    def observe(self, t, x):
        pass

    def stage_params(self, t, x):
        return self.P, 0.0

    def set_plan(self, t, xs):
        pass

    def clear_plan(self):
        pass


def load(spec, N, dt, params, v_min=0.0, blend=0.0, root=''):
    """spec: '' | 'hyperpm:<.npz>' (상대 경로는 root 기준). <.npz> 만 줘도 hyperpm 으로 본다."""
    import os
    spec = (spec or '').strip()
    if not spec:
        return ConstantParams(params, N)
    kind, _, path = spec.partition(':') if ':' in spec else ('hyperpm', '', spec)
    path = path if os.path.isabs(path) else os.path.join(root, path)
    if kind == 'hyperpm':
        import sys
        repo = os.path.abspath(os.path.join(os.path.dirname(__file__), *['..'] * 5))
        if repo not in sys.path:
            sys.path.insert(0, repo)        # 저장소 최상위 (learned/)
        from learned.hyperpm.hyper_params import HyperStage
        return HyperStage(path, N, dt, np.asarray(params, dtype=float), v_min=v_min, blend=blend)
    raise ValueError(f'모르는 param_model: {kind} (hyperpm 만 있다)')
