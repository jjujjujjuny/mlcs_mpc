#!/usr/bin/env python3
"""
export_hyper.py — 학습한 HyperPM(시변 파라미터 모델)을 numpy 용 .npz 로 내보낸다.

  ~/hpm-venv/bin/python tools/export_hyper.py <hyper_model.pt> <.hydra/config.yaml> <출력.npz> [검증용 csv ...]

  차(젯슨)에는 torch 가 없다. 네트워크는 GRU(5→16) 와 Linear 몇 개, B-스플라인 행렬뿐이라
  가중치만 꺼내 hypermpc/hyper_params.py 가 numpy 로 같은 계산을 한다.

  검증용 csv (prep_hpm_dataset 형식) 를 주면 torch 와 numpy 의 파라미터 차이를 재고,
  파라미터가 실제 데이터에서 움직인 범위(p0.5~p99.5)를 같이 적어 둔다 (차에서 진단용).
"""
import os
import sys
import types

import numpy as np

for n in ('robot_model.acrobot', 'robot_model.acrobot.acrobot_model',
          'robot_model.acrobot.observation_preprocesor', 'robot_model.acrobot.acrobot_params'):
    sys.modules[n] = types.ModuleType(n)
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.expanduser('~/hypermpc_code'), os.path.join(HERE, '..', 'hypermpc', 'hpm'),
                os.path.join(HERE, '..', 'hypermpc')]

import torch  # noqa: E402
from omegaconf import OmegaConf  # noqa: E402
from hydra.utils import instantiate  # noqa: E402
from mlcs_car_model import MlcsSingleTrack  # noqa: E402
from hyper_params import HyperParams  # noqa: E402


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 1
    pt, cfg_path, out = sys.argv[1:4]
    cfg = OmegaConf.load(cfg_path)
    cfg.hpm.device = 'cpu'
    cfg.hpm.compile = False
    base = MlcsSingleTrack()
    hm = instantiate(cfg.hmodel.model)(default_params=base.get_default_params(),
                                       always_positive=base.positive_params(),
                                       free_params=base.free_params()).to('cpu')
    hm.load_state_dict(torch.load(pt, map_location='cpu'))
    hm.eval()
    hi = instantiate(cfg.hmodel.interpoler)
    assert len(base.free_params()) == 0, '자유(부호 없는) 파라미터가 있으면 hyper_params.py 를 고쳐야 한다'
    assert not cfg.hmodel.model.last_ctrl_point_on_const

    g = lambda t: t.detach().cpu().numpy().astype(np.float64)  # noqa: E731
    tse = hm.time_series_encoder
    pre = tse.observations_preprocessor
    rnn = tse.rnn
    assert isinstance(rnn, torch.nn.GRU) and rnn.num_layers == 1
    W = dict(
        obs_scale=np.array([pre.max_velocity, pre.max_velocity, pre.max_yaw_rate,
                            pre.max_omega_wheels, pre.max_delta], dtype=np.float64),
        gru_w_ih=g(rnn.weight_ih_l0), gru_w_hh=g(rnn.weight_hh_l0),
        gru_b_ih=g(rnn.bias_ih_l0), gru_b_hh=g(rnn.bias_hh_l0),
        up_w=g(hm.hidden_state_upscale.weight), up_b=g(hm.hidden_state_upscale.bias),
        h12_w=g(hm.hl1_to_hl2.weight), h12_b=g(hm.hl1_to_hl2.bias),
        c1_w=g(hm.causal_mlp_1.linear.weight * hm.causal_mlp_1.mask),
        h1m_w=g(hm.hl1_to_m1.weight), h1m_b=g(hm.hl1_to_m1.bias),
        c2_w=g(hm.causal_mlp_2.linear.weight * hm.causal_mlp_2.mask),
        h2p_w=g(hm.hl2_to_params.weight), h2p_b=g(hm.hl2_to_params.bias),
        pred_div=g(hm.future_predictions_encoder.channels_deviders),
        pred_pinv=g(hm.future_predictions_encoder.inv_Ns).reshape(-1, int(cfg.dataset.prediction_horizon)),
        interp_Ns=g(hi.Ns).reshape(-1, int(hm.param_ctrl_point)),
        p_const=g(hm.const_params.exp()).reshape(-1),
        max_param_scaler=np.float64(hm.max_param_scaler),
        n_ctrl=np.int64(hm.param_ctrl_point),
        obs_window=np.int64(cfg.dataset.observation_window),
        horizon=np.int64(cfg.dataset.prediction_horizon),
        tp=np.float64(cfg.dataset.Tp),
        source=np.array(os.path.abspath(pt)),
    )
    for k in ('c1_w', 'c2_w'):     # causal 층에 bias 가 없어야 한다 (bias=False)
        lin = getattr(hm, 'causal_mlp_' + k[1]).linear
        assert lin.bias is None
    assert W['pred_pinv'].shape == (W['n_ctrl'], W['horizon'])
    assert W['interp_Ns'].shape[0] == 2 * W['horizon']     # rk4 반 칸 격자

    np.savez_compressed(out, **W)
    hp = HyperParams(out)
    print(f'저장: {out}  ({os.path.getsize(out) / 1024:.0f} KB)')

    csvs = sys.argv[4:]
    if not csvs:
        return 0
    from dataset_reader.f1tenth_dataset_new import F1tenthHyperDataset2 as DS
    errs, P_all, ref = [], [], None
    for c in csvs:
        ds = DS(dataset_path=c, observation_window=int(W['obs_window']),
                prediction_horizon=int(W['horizon']), stride=20, device='cpu')
        d = torch.utils.data.DataLoader(ds, batch_size=256, shuffle=False)
        with torch.no_grad():
            for M, P, U, X0, X in d:
                pp, _ = hm(M, P, True)
                pt_ = g(hi(pp)[:, ::2, :])
                if ref is None:     # 차에서 torch 없이 numpy 이식을 확인할 기준 (test_hyper_params.py)
                    ref = dict(ref_M=g(M[0]), ref_P=g(P[0]), ref_out=pt_[0])
                for b in range(len(M)):
                    pn = hp.predict(g(M[b]), g(P[b]))
                    errs.append(np.max(np.abs(pn - pt_[b]) / np.abs(pt_[b])))
                    P_all.append(pn[::10])
    P_all = np.concatenate(P_all)
    lo, hi_ = np.quantile(P_all, 0.005, axis=0), np.quantile(P_all, 0.995, axis=0)
    np.savez_compressed(out, **W, **ref, data_lo=lo, data_hi=hi_)
    print(f'torch ↔ numpy 상대 오차 최대 {max(errs):.2e}  ({len(errs)}창)')
    print('파라미터 / 기준값  (데이터 p0.5 ~ p99.5)')
    names = ['I_z', 'lr', 'Cd0', 'Cd2', 'Cd1', 'I_e', 'K_fi', 'b1', 'b0', 'R'] + \
        ['f_' + x for x in 'Sx_p Alpha_p By Cy Dy Ey Bx Cx Dx Ex'.split()] + \
        ['r_' + x for x in 'Sx_p Alpha_p By Cy Dy Ey Bx Cx Dx Ex'.split()]
    for i, nm in enumerate(names):
        print(f'   {nm:10s} {W["p_const"][i]:9.4f}   ×{lo[i] / W["p_const"][i]:.2f} ~ ×{hi_[i] / W["p_const"][i]:.2f}')
    return 0 if max(errs) < 1e-4 else 1


if __name__ == '__main__':
    sys.exit(main())
