"""
ppo — 배치 환경 (env/vec/race_env.py) 용 PPO. arXiv:2504.02420 III-E 의 설정을 기본값으로.

  actor   MLP 2×256, critic MLP 2×512, LeakyReLU(0.2)          (논문)
  γ 0.99, 학습률 1e-3 → 1e-4 선형 감소, 미니배치 1024,
  환경 400 × 롤아웃 1024 스텝 (한 번 갱신에 409,600 샘플)      (논문)
  GAE λ 0.95, clip 0.2, 에포크 10, 그래디언트 클립 0.5, 엔트로피 0  (논문에 없음 → stable-baselines3 PPO 기본값, 논문 [28])

■ 행동 — 가우시안 잠재 u 를 뽑아 tanh(u) 를 내보낸다 (codemoo/vectorized-vehicle-control 의 PPO 와 같은 방식).
  버퍼에는 u 와 그 log π 를 저장하고 다시 계산할 때도 같은 u 를 쓴다 (atanh(action) 으로 되돌리지 않는다).
  log(1 − tanh(u)²) = 2·(log 2 − u − softplus(−2u))  (수치 안정)
  배포 (결정적) 행동 = tanh(평균).

■ 종료·시간 제한 — terminated (벽) 는 V = 0, truncated (시간 제한) 는 리셋 전 final obs 의 V 로 부트스트랩.
"""
import math
from dataclasses import asdict, dataclass

import torch
import torch.nn as nn
import torch.nn.functional as F

LOG2 = math.log(2.0)


def mlp(n_in, hidden, n_out, slope=0.2):
    layers, d = [], n_in
    for h in hidden:
        layers += [nn.Linear(d, h), nn.LeakyReLU(slope)]
        d = h
    layers.append(nn.Linear(d, n_out))
    return nn.Sequential(*layers)


class ActorCritic(nn.Module):

    def __init__(self, obs_dim, act_dim, actor_hidden=(256, 256), critic_hidden=(512, 512), log_std0=-0.5):
        super().__init__()
        self.actor = mlp(obs_dim, actor_hidden, act_dim)
        self.critic = mlp(obs_dim, critic_hidden, 1)
        self.log_std = nn.Parameter(torch.full((act_dim,), float(log_std0)))
        with torch.no_grad():                       # 처음에는 작은 행동
            self.actor[-1].weight.mul_(0.01)
            self.actor[-1].bias.zero_()

    def value(self, obs):
        return self.critic(obs).squeeze(-1)

    def dist(self, obs):
        mean = self.actor(obs)
        return torch.distributions.Normal(mean, self.log_std.clamp(-5, 1).exp().expand_as(mean))

    @staticmethod
    def squash_logp(dist, u):
        return (dist.log_prob(u) - 2.0 * (LOG2 - u - F.softplus(-2.0 * u))).sum(-1)

    @torch.no_grad()
    def act(self, obs, deterministic=False):
        d = self.dist(obs)
        u = d.mean if deterministic else d.sample()
        return torch.tanh(u), u, self.squash_logp(d, u)


@dataclass
class PPOConfig:
    n_steps: int = 1024
    batch_size: int = 1024
    epochs: int = 10
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip: float = 0.2
    lr_start: float = 1e-3
    lr_end: float = 1e-4
    vf_coef: float = 0.5
    ent_coef: float = 0.0
    max_grad_norm: float = 0.5
    target_kl: float = 0.0          # 0 = 끔
    log_std0: float = -0.5

    def to_dict(self):
        return asdict(self)


class Rollout:

    def __init__(self, T, B, obs_dim, act_dim, device):
        z = lambda *s: torch.zeros(*s, device=device)  # noqa: E731
        self.obs, self.u, self.logp = z(T, B, obs_dim), z(T, B, act_dim), z(T, B)
        self.rew, self.val, self.next_val = z(T, B), z(T, B), z(T, B)
        self.term, self.done = z(T, B), z(T, B)
        self.adv, self.ret = z(T, B), z(T, B)

    def gae(self, gamma, lam):
        """delta = r + γ(1−term)V(final obs) − V(obs). 흔적은 term·trunc 모두에서 끊는다."""
        T = self.rew.shape[0]
        last = torch.zeros_like(self.rew[0])
        for t in reversed(range(T)):
            delta = self.rew[t] + gamma * (1 - self.term[t]) * self.next_val[t] - self.val[t]
            last = delta + gamma * lam * (1 - self.done[t]) * last
            self.adv[t] = last
        self.ret = self.adv + self.val


def update(ac, opt, buf, cfg: PPOConfig, lr):
    for g in opt.param_groups:
        g['lr'] = lr
    T, B = buf.rew.shape
    N = T * B
    obs = buf.obs.reshape(N, -1)
    u = buf.u.reshape(N, -1)
    logp_old = buf.logp.reshape(N)
    adv = buf.adv.reshape(N)
    adv = (adv - adv.mean()) / (adv.std() + 1e-8)
    ret = buf.ret.reshape(N)
    stats = dict(pg=0.0, vf=0.0, kl=0.0, clipfrac=0.0, n=0)
    for _ in range(cfg.epochs):
        perm = torch.randperm(N, device=obs.device)
        for k in range(0, N, cfg.batch_size):
            i = perm[k:k + cfg.batch_size]
            d = ac.dist(obs[i])
            logp = ac.squash_logp(d, u[i])
            ratio = torch.exp(logp - logp_old[i])
            pg = -torch.min(ratio * adv[i], ratio.clamp(1 - cfg.clip, 1 + cfg.clip) * adv[i]).mean()
            vf = F.mse_loss(ac.value(obs[i]), ret[i])
            ent = d.entropy().sum(-1).mean()
            loss = pg + cfg.vf_coef * vf - cfg.ent_coef * ent
            if not torch.isfinite(loss):
                raise FloatingPointError('PPO 손실이 유한하지 않습니다')
            opt.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(ac.parameters(), cfg.max_grad_norm)
            opt.step()
            with torch.no_grad():
                kl = (logp_old[i] - logp).mean().item()
                stats['pg'] += pg.item()
                stats['vf'] += vf.item()
                stats['kl'] += kl
                stats['clipfrac'] += ((ratio - 1).abs() > cfg.clip).float().mean().item()
                stats['n'] += 1
        if cfg.target_kl > 0 and stats['kl'] / max(stats['n'], 1) > cfg.target_kl:
            break
    n = max(stats.pop('n'), 1)
    return {k: v / n for k, v in stats.items()}
