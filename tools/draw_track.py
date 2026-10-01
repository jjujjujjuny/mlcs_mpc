#!/usr/bin/env python3
"""
draw_track.py — 손으로 레이싱 라인을 찍으면 스플라인 + 양쪽 벽을 만든다.

  ./drive.sh drawtrack [이름]

■ 하는 일

  ① 실험실 경계 19각형을 띄운다
     (lab_map/data/boundary/map_boundary_data.json — mocap 월드 좌표계)
  ② 마우스로 대충 클릭하면 **닫힌 주기 스플라인**이 지나간다
  ③ 중심선에서 ±half_width 로 좌/우 벽을 만든다
  ④ 클릭할 때마다 바로 검사한다 — 곡률 한계, 경계 포함, 벽까지 여유
  ⑤ s 로 저장

■ ★ 왜 "그려지는 대로" 가 아니라 검사를 하는가

  우리 차는 R_min = L/tan(δ_max) = 0.33/tan(0.40) = 0.781 m 보다
  급한 코너를 **물리적으로 못 돈다** (2026-09-30 실측). 손으로 찍으면
  코너가 쉽게 그보다 급해지고, 그러면 조향을 끝까지 줘도 경로를
  벗어난다 — 게인 튜닝으로 해결되는 문제가 아니다.

  화면에서 **빨간 구간이 그 "못 도는 곳"** 이다. 하나도 없게 만들고
  저장하는 것이 이 도구를 쓰는 이유다.

  실제로 track_1 은 이 검사 없이 만들어져서 31% 가 주행 불가였다.
  자세한 경위는 docs/TRACKS.md 에 있다.

■ 트랙 폭은 왜 ±0.25 인가

  HyperMPC 원저자 값이다. conf_mpc/config_car.yaml 의 track_width: 0.25 가
  robot_model/car/casadi_car_model.py:106 에서 이렇게 쓰인다:

      h_left  = cfg.track_width - n        →  n ≤ +0.25
      h_right = cfg.track_width + n        →  n ≥ -0.25

  **/2 가 없으므로 반폭**이다 (전폭 0.50 m). 같은 저장소의 drift-parking
  모델은 width_s(s)/2 - n 으로 전폭을 받으니, 이름이 파일마다 다른 뜻으로
  쓰인다 — 차량 모델 쪽이 반폭이고 그것이 논문 결과를 낸 설정이다.

  ⚠ 이것은 **소프트 제약**이다. log(1+exp(-200·h)) 를 비용에 더하는
  배리어이고, ocp.constraints.lbx 는 비어 있다 — 하드 상한이 아예 없다.
  즉 0.25 는 "벽" 이 아니라 "중심선에서 이만큼 벗어나면 비용이 물린다" 는
  튜닝값이다. 실제로 저자들 트랙 중 lab_curvy_v1 은 반폭 0.125 로
  **차폭(0.27 m)보다도 좁다.**

  track_5 는 반폭 0.30 으로 만들어져 있어 다르다. --half-width 로 바꿀 수 있다.

■ 키

    왼쪽 클릭    점 추가
    오른쪽 클릭  가장 가까운 점 삭제
    u            마지막 점 취소        c  전부 지우기
    [  ]         트랙 폭  ∓0.02 / ±0.02 m
    -  =         평활화   덜 / 더  (손떨림을 얼마나 무시할지)
    f            자동 평활화 — 통과하는 최소값을 찾아 준다
    r            진행 방향 뒤집기
    t            track_5 겹쳐 보기     h  HyperMPC 전처리 겹쳐 보기
    s            저장                  q  끝내기

  점은 4개 이상이어야 스플라인이 생긴다 (3차 주기 스플라인).

■ 무엇이 막고 무엇이 경고인가

  손으로 찍으면 클릭점 사이에서 스플라인이 국소적으로 튄다. **피할 수
  없다.** 그래서 "한 점이라도 넘으면 불합격" 으로는 통과가 거의 불가능하다.
  5 cm 한 칸이 넘는 것과 2 m 가 내리 넘는 것은 전혀 다른 문제이므로,
  비율이 아니라 **최장 연속 구간**으로 판정한다.

    막는다    곡률이 한계를 넘는 구간이 **연속 15 cm 를 넘을 때**
              (--max-bad-run, 기본 0.15 m = 5cm 세 칸)
              그리고 실험실 경계를 벗어날 때

    경고만    벽 접힘 (hw·κ ≥ 1). 접힌 벽이 망치는 것은
              left/right_boundary.csv (참고용) 와 그 구간의 여유 숫자뿐이다.
              **MPC 는 벽 좌표를 받지 않는다** — 스칼라 cfg.track_width 만
              쓴다 (casadi_car_model.py:106). 주행 가능성을 가르는 것은
              곡률이므로, 막는 것은 곡률 하나로 한다.

              거슬리면 [ 로 폭을 줄이면 사라진다 (hw 가 작아지면 hw·κ < 1).

  f 를 누르면 통과하는 **최소 평활화**를 찾아 준다. 그래도 안 되면 평활화로
  풀 문제가 아니다 — 빨간 구간의 점을 우클릭으로 지우고 더 벌려 찍어야 한다.

■ ★ 선이 두 개 보이는 이유 — 둘 다 통과해야 저장된다

  초록/주황/빨강 굵은 선   우리 스플라인 (k=3, 5 cm). 이게
                           waypoints/<이름>.csv 가 되고 Stanley 가 따라간다.
  주황 파선                저자 전처리기를 통과한 것 (k=5, s=2.0, 20 cm).
                           **HyperMPC 가 실제로 받는 중심선이다.**
                           한계를 넘는 점에 × 가 찍힌다.

  두 스플라인은 설정이 달라 곡률이 **양방향으로** 달라진다 (실측):

      트랙      우리 R_min   저자 R_min
      넉넉        1.074 m     1.274 m    ← 전처리가 완만하게
      보통        0.684 m     0.921 m    ← 우리만 불합격
      급함        0.406 m     0.510 m    ← 둘 다 불합격

  그래서 한쪽만 보면 "그려서 ✓ → preptrack 에서 ✗" 가 난다. 저장은
  **둘 다** 통과해야 되고, 전처리 결과는 ./drive.sh preptrack 이 내는
  값과 같다 (검증: R_min 1.274 양쪽 일치).

  원저장소가 없으면 미리보기만 빠지고 나머지는 그대로 쓴다.
"""
import argparse
import json
import math
import os
import pathlib
import sys
import time

import numpy as np

# ★ 백엔드를 먼저 정한다. NoMachine 으로 보는 젯슨에는 TkAgg 가 있다.
#   import pyplot 뒤에 use() 를 부르면 늦다.
import matplotlib
if not os.environ.get('DISPLAY'):
    print('✗ DISPLAY 가 없습니다. 이 도구는 창을 띄웁니다 — '
          'NoMachine 으로 젯슨 화면에 접속한 뒤 실행하세요.')
    sys.exit(1)
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt                             # noqa: E402
from matplotlib.collections import LineCollection           # noqa: E402
from matplotlib.path import Path as MplPath                 # noqa: E402
from scipy.interpolate import splev, splprep                # noqa: E402


def setup_font():
    """한글이 □ 로 나오지 않게 폰트를 잡는다.

    matplotlib 기본 폰트(DejaVu Sans)에는 한글 글리프가 없어서, 그냥 띄우면
    화면의 라벨과 안내문이 전부 네모로 보인다 (젯슨에서 실제로 그렇다).
    Noto Sans CJK 가 깔려 있으면 그것을 쓴다 — .ttc 묶음이라 matplotlib 은
    첫 face 이름(보통 'Noto Sans CJK JP')으로만 등록하지만, pan-CJK 폰트라
    한글 글리프가 같이 들어 있다.

    패널은 monospace 로 숫자를 맞추므로 monospace 목록에도 끼워 넣는다.
    못 찾으면 경고만 하고 넘어간다 — 도구는 그대로 쓸 수 있다.
    """
    import matplotlib.font_manager as fm
    have = {f.name for f in fm.fontManager.ttflist}
    for cand in ('Noto Sans CJK KR', 'Noto Sans CJK JP', 'NanumGothic',
                 'NanumBarunGothic', 'UnDotum', 'Baekmuk Gulim',
                 'Noto Sans KR'):
        if cand in have:
            matplotlib.rcParams['font.family'] = [cand, 'DejaVu Sans']
            matplotlib.rcParams['font.monospace'] = [cand, 'DejaVu Sans Mono']
            # CJK 폰트에는 U+2212(−) 가 없는 경우가 있어 ASCII 하이픈을 쓴다
            matplotlib.rcParams['axes.unicode_minus'] = False
            return cand
    print('⚠ 한글 폰트를 못 찾았습니다 — 화면 글자가 □ 로 보일 수 있습니다.')
    print('  sudo apt install fonts-noto-cjk   (또는 fonts-nanum)')
    return None

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOUNDARY = os.path.join(REPO, 'lab_map/data/boundary/map_boundary_data.json')
WAYPOINTS = os.path.join(REPO, 'src/mlcs_mpc/waypoints')
OUTDIR = os.path.join(REPO, 'lab_map/data/handdrawn')


def load_track_reader(hypermpc):
    """저자들 TrackReader 를 가져온다. 없으면 None — 도구는 그대로 쓴다.

    ★ 저자 코드가 `import scipy` 만 하고 scipy.signal / interpolate /
      integrate 를 쓴다. 먼저 올려 줘야 AttributeError 가 안 난다.
      (그쪽 환경에서는 map_reader 가 먼저 로드돼 가려졌던 문제다)
    """
    tracks = os.path.join(os.path.expanduser(hypermpc), 'mpc/tracks')
    if not os.path.isdir(tracks):
        return None
    try:
        import scipy.integrate      # noqa: F401
        import scipy.interpolate    # noqa: F401
        import scipy.signal         # noqa: F401
        if tracks not in sys.path:
            sys.path.insert(0, tracks)
        from track_preprocesor import TrackReader
        return TrackReader
    except Exception as e:
        print(f'⚠ 저자 전처리기를 못 불렀습니다 ({e}) — 미리보기 없이 갑니다')
        return None


def run_prep(TrackReader, craw, half_width, spacing):
    """저자 전처리기를 그대로 돌려 MPC 가 실제로 받을 값을 낸다.

    TrackReader 가 경로를 받으므로 임시 파일을 거친다. 1 cm 1000점에
    11.6 ms (젯슨 실측) — 클릭마다 돌려도 된다.
    """
    import tempfile
    import warnings
    f = tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False,
                                    encoding='utf-8')
    try:
        f.write('x_m,y_m,w_tr_right_m,w_tr_left_m\n')
        for q in craw:
            f.write(f'{q[0]:.6f},{q[1]:.6f},{half_width:.6f},{half_width:.6f}\n')
        f.close()
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')     # splprep 의 per= 경고
            t = TrackReader(pathlib.Path(f.name))
    except Exception as e:
        return {'err': str(e)}
    finally:
        try:
            os.unlink(f.name)
        except OSError:
            pass
    ak = np.abs(t.re_curvature)
    return dict(
        c=np.c_[t.re_x, t.re_y], kappa=t.re_curvature, rmse=float(t.rmse),
        n=int(t.re_N), total=float(t.track_lenght), k_max=float(ak.max()),
        r_min=float(1.0 / max(ak.max(), 1e-9)),
        w_min=float(t.re_track_width_corrected.min()),
        w_max=float(t.re_track_width_corrected.max()),
        spacing=float(t.track_lenght / max(t.re_N, 1)),
    )


# ─────────────────────────────────────────────────────────────────────
#  기하
# ─────────────────────────────────────────────────────────────────────
def load_boundary(path):
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    v = np.array([[p['x'], p['y']] for p in d['boundary']['vertices']],
                 dtype=float)
    cf = d.get('coordinate_frame', {})
    if cf.get('unit') not in (None, 'meter', 'm'):
        print(f'⚠ 경계 단위가 meter 가 아닙니다: {cf.get("unit")}')
    return v, d


def seg_dist(P, A, B):
    """점 P 에서 선분 AB 까지 거리."""
    AB = B - A
    L2 = AB @ AB
    if L2 < 1e-12:
        return np.linalg.norm(P - A)
    t = np.clip(((P - A) @ AB) / L2, 0.0, 1.0)
    return np.linalg.norm(P - (A + t * AB))


def dist_to_polygon(pts, V):
    """각 점에서 다각형 둘레까지의 최소 거리 (안/밖 구분 없음)."""
    out = np.empty(len(pts))
    n = len(V)
    for i, p in enumerate(pts):
        out[i] = min(seg_dist(p, V[j], V[(j + 1) % n]) for j in range(n))
    return out


def fit_closed_spline(pts, dev, spacing):
    """클릭한 점들을 지나는 닫힌 주기 스플라인 → 등간격 재샘플.

    dev 는 "손떨림을 얼마나 무시할지" 를 m 단위로 받는다. FITPACK 의
    s 는 잔차 제곱합이므로 s = m·dev² 로 환산한다. dev=0 이면 찍은 점을
    정확히 통과하고(대신 울퉁불퉁), 키우면 매끄러워지며 점에서 멀어진다.

    반환: (중심선 Nx2, 곡률 N, 총길이, tck) — 실패하면 None
    """
    if len(pts) < 4:
        return None
    # per=1 은 마지막 점이 첫 점과 같다고 보므로 직접 닫아 준다
    P = np.vstack([pts, pts[:1]])
    s = len(P) * dev * dev
    try:
        tck, _ = splprep([P[:, 0], P[:, 1]], s=s, per=1, k=3)
    except Exception as e:                                   # 중복점 등
        print(f'  스플라인 실패: {e}')
        return None

    # 호길이로 등간격 재샘플 — 촘촘히 깔고 누적거리로 u 를 역산한다
    uu = np.linspace(0.0, 1.0, 4000)
    dense = np.array(splev(uu, tck)).T
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
    total = seg[-1]
    if total < 0.5:
        return None
    c, kappa = resample(tck, seg, uu, total, spacing)
    return c, kappa, total, tck


def resample(tck, seg, uu, total, spacing):
    """호길이 등간격으로 다시 깔고 곡률을 해석적으로 낸다."""
    u = np.interp(np.arange(0.0, total, spacing), seg, uu)
    c = np.array(splev(u, tck)).T
    d1 = np.array(splev(u, tck, der=1)).T
    d2 = np.array(splev(u, tck, der=2)).T
    sp = np.linalg.norm(d1, axis=1)
    kappa = ((d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
             / np.maximum(sp ** 3, 1e-12))
    return c, kappa


def arclen_table(tck, n=4000):
    uu = np.linspace(0.0, 1.0, n)
    dense = np.array(splev(uu, tck)).T
    seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
    return seg, uu, seg[-1]


def shoelace(xy):
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def winding(kappa, spacing):
    """∮κ ds / 2π. 단순 폐곡선이면 ±1 이다.

    ★ 평활화를 세게 걸면 스플라인이 **붕괴한다** — 고리가 접혀서 거의
      직선이 되고 곡률이 0 에 수렴한다. 그러면 R_min 이 10^9 로 나오면서
      "곡률 한계 통과" 로 보인다. 실제로 자동 평활화가 그 거짓 통과를
      받아들이는 것을 겪었다 (작은 고리에 평활 0.24).

      ∮κ ds 는 그걸 잡는다 — 붕괴하면 0 에 가까워지고, 8자로 교차하면
      0, 정상 폐곡선이면 ±2π 다.
    """
    return float((kappa * spacing).sum() / (2.0 * np.pi))


def longest_run(mask, spacing):
    """폐곡선에서 True 가 연속으로 이어지는 최대 호길이 [m].

    ★ "한 점이라도 넘으면 불합격" 은 손으로 찍는 입력에 쓸 수 없다.
      스플라인이 클릭점 사이에서 국소적으로 튀는 것은 피할 수 없고,
      5 cm 한 칸이 넘는 것과 2 m 가 내리 넘는 것은 전혀 다른 문제다.
      그래서 비율과 함께 **최장 연속 구간**을 본다.
    """
    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return 0.0
    if mask.all():
        return float(len(mask) * spacing)
    # 폐곡선이므로 두 번 이어 붙여 감긴 구간도 잡는다
    run = best = 0
    for b in np.r_[mask, mask]:
        run = run + 1 if b else 0
        best = max(best, run)
    return float(min(best, int(mask.sum())) * spacing)


def offset_walls(c, half_width):
    """중심선 양쪽으로 벽. 폐곡선이므로 차분을 감아서 접선을 낸다."""
    t = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-12)
    n = np.c_[-t[:, 1], t[:, 0]]            # 좌측 법선 (CCW 기준 바깥)
    return c + half_width * n, c - half_width * n


# ─────────────────────────────────────────────────────────────────────
#  편집기
# ─────────────────────────────────────────────────────────────────────
class TrackDrawer:
    def __init__(self, args):
        self.a = args
        self.V, self.bmeta = load_boundary(args.boundary)
        self.poly = MplPath(np.vstack([self.V, self.V[:1]]))
        self.pts = []
        self.dev = args.smooth
        self.hw = args.half_width
        self.show_ref = True
        self.show_prep = True
        self.fit = None
        self.prep = None
        self.tck = None
        # 저자 전처리기 — 없으면 미리보기만 빠지고 나머지는 그대로 쓴다
        self.TrackReader = load_track_reader(args.hypermpc)
        if self.TrackReader is None:
            print(f'⚠ {args.hypermpc} 가 없어 HyperMPC 미리보기를 끕니다.')
            print(f'  git clone https://github.com/hyper-mpc/hypermpc_code.git '
                  f'{args.hypermpc}')
        self.stats = {}

        self.kappa_lim = math.tan(args.max_steer) / args.wheelbase
        self.kappa_tgt = args.kappa_target

        self.ref = None
        rp = os.path.join(WAYPOINTS, 'track_5.csv')
        if os.path.isfile(rp):
            self.ref = np.loadtxt(rp, delimiter=',', comments='#',
                                  usecols=(0, 1))

        setup_font()
        self.fig, self.ax = plt.subplots(figsize=(9.5, 10))
        self.fig.canvas.manager.set_window_title('draw_track — 레이싱 라인')
        self.txt = self.fig.text(0.015, 0.012, '', family='monospace',
                                 fontsize=8.5, va='bottom')
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        # ★★ 이 자기참조가 없으면 클릭이 안 먹는다.
        #   matplotlib 의 CallbackRegistry 는 bound method 를 WeakMethod 로
        #   들고 있다 (3.5.1 에서 확인). 그래서 이 인스턴스를 아무도 안 들고
        #   있으면 GC 되고, 창은 떠 있는데 콜백만 조용히 죽는다 — __init__
        #   안의 첫 redraw() 는 이미 끝났으니 경계는 보이고 클릭만 안 먹는
        #   증상이 된다. 그림에 묶어 두면 그림이 살아 있는 동안 안 죽는다.
        self.fig._mlcs_drawer = self
        self.redraw()

    # ── 입력 ────────────────────────────────────────────────────────
    def on_click(self, event):
        if event.inaxes is not self.ax or event.xdata is None:
            return
        p = np.array([event.xdata, event.ydata])
        if event.button == 1:
            if not self.poly.contains_point(p):
                print('  ✗ 경계 밖입니다 — 무시')
                return
            self.pts.append(p)
        elif event.button == 3 and self.pts:
            d = [np.linalg.norm(p - q) for q in self.pts]
            self.pts.pop(int(np.argmin(d)))
        self.redraw()

    def on_key(self, event):
        k = event.key
        if k == 'u' and self.pts:
            self.pts.pop()
        elif k == 'c':
            self.pts = []
        elif k == ']':
            self.hw = min(1.0, self.hw + 0.02)
        elif k == '[':
            self.hw = max(0.05, self.hw - 0.02)
        elif k == '=':
            self.dev = min(0.50, self.dev + 0.01)
        elif k == '-':
            self.dev = max(0.0, self.dev - 0.01)
        elif k == 'f':
            self.autosmooth()
            return
        elif k == 'r':
            self.pts = self.pts[::-1]
        elif k == 't':
            self.show_ref = not self.show_ref
        elif k == 'h':
            self.show_prep = not self.show_prep
        elif k == 's':
            self.save()
            return
        elif k == 'q':
            plt.close(self.fig)
            return
        else:
            return
        self.redraw()

    def autosmooth(self):
        """통과하는 가장 작은 평활화를 찾는다.

        손으로 찍으면 클릭점 사이에서 스플라인이 국소적으로 튀는 것을 피할
        수 없다. 평활화를 올리면 사라지지만, 많이 올리면 찍은 모양에서
        멀어진다. 그래서 **통과하는 최소값**을 찾아 준다.

        우리 스플라인과 저자 전처리 결과를 둘 다 본다 — 저장 조건과 같다.
        """
        if len(self.pts) < 4:
            print('✗ 점이 4개 미만입니다')
            return
        keep = self.dev
        print(f'■ 자동 평활화 — {keep:.2f} m 부터 올려 봅니다 '
              f'(허용 연속 {self.a.max_bad_run*100:.0f}cm)')
        collapsed = 0
        for dev in np.arange(keep, 0.501, 0.01):
            self.dev = round(float(dev), 3)
            self.recompute()
            if not self.stats:
                continue
            s = self.stats
            # 한 번 붕괴하면 더 올려도 돌아오지 않는다 — 몇 번 보고 끊는다
            collapsed = collapsed + 1 if s['degenerate'] else 0
            if collapsed >= 3:
                print(f'  붕괴 평활 {self.dev:.2f} — 더 올려도 돌아오지 '
                      f'않습니다. 중단합니다.')
                break
            ok = self.ours_ok() and self.prep_ok() is not False
            flag = '✓' if ok else ('붕괴' if s['degenerate'] else '  ')
            pr_s = ''
            if self.prep and 'err' not in self.prep:
                pr_s = f'  전처리 {min(self.prep["r_min"], 999.0):.3f} m'
            print(f'  {flag:>2} 평활 {self.dev:.2f}  R_min {s["r_min"]:7.3f} m  '
                  f'최장불가 {s["bad_run"]*100:3.0f}cm  '
                  f'∮κds/2π {s["wind"]:+.2f}{pr_s}')
            if ok:
                print(f'→ 평활 {self.dev:.2f} m 로 맞췄습니다. '
                      f'모양이 뭉개졌으면 - 로 내리고 그 코너를 다시 찍으세요.')
                self.redraw()
                return
        reached = self.dev
        self.dev = keep
        print(f'✗ 평활 {reached:.2f} m 까지 올려도 통과하지 못했습니다 — '
              f'평활화로 풀 문제가 아닙니다. (평활은 {keep:.2f} 로 되돌립니다)')
        print(f'  코너 하나가 R {1/self.kappa_lim:.2f} m 보다 급하게 찍혀 '
              f'있을 가능성이 큽니다. 빨간 구간의 점을 우클릭으로 지우고 '
              f'더 벌려서 다시 찍으세요.')
        self.redraw()

    # ── 계산 + 그리기 ───────────────────────────────────────────────
    def recompute(self):
        self.fit = None
        self.prep = None
        self.stats = {}
        if len(self.pts) < 4:
            return
        r = fit_closed_spline(np.array(self.pts), self.dev, self.a.spacing)
        if r is None:
            return
        c, kap, total, tck = r
        self.tck = tck
        left, right = offset_walls(c, self.hw)
        ak = np.abs(kap)

        inside = (self.poly.contains_points(c).all()
                  and self.poly.contains_points(left).all()
                  and self.poly.contains_points(right).all())
        # 벽이 다각형에 얼마나 붙었나 (바깥으로 나간 경우도 거리는 양수라
        # inside 와 같이 봐야 한다)
        clr = min(dist_to_polygon(left, self.V).min(),
                  dist_to_polygon(right, self.V).min())

        self.fit = dict(c=c, kappa=kap, left=left, right=right)

        # ★ MPC 가 실제로 받을 값 — 저자 전처리기를 그대로 돌린다.
        #   우리 스플라인(k=3, 5cm)과 저자 것(k=5, s=2.0, 20cm)이 다르므로
        #   여기서 같이 봐야 "그려서 ✓ → preptrack 에서 ✗" 를 막는다.
        if self.TrackReader is not None and self.tck is not None:
            seg, uu, T = arclen_table(self.tck)
            craw, _ = resample(self.tck, seg, uu, T, self.a.raw_spacing)
            self.prep = run_prep(self.TrackReader, craw, self.hw,
                                 self.a.raw_spacing)
            if 'err' not in self.prep:
                pk = np.abs(self.prep['kappa'])
                self.prep['bad'] = float((pk > self.kappa_lim).mean())
                self.prep['warn'] = float((pk > self.kappa_tgt).mean())
        wind = winding(kap, self.a.spacing)
        area = shoelace(c)
        area_click = shoelace(np.array(self.pts)) if len(self.pts) >= 3 else 0.0
        # 단순 폐곡선인가 — 붕괴/8자 교차를 잡는다
        degenerate = (abs(abs(wind) - 1.0) > 0.15
                      or area < max(0.25 * area_click, 0.05))
        bad_m = ak > self.kappa_lim
        fold_m = self.hw * ak >= 1.0
        sp = self.a.spacing
        self.stats = dict(
            n=len(c), total=total, k_max=ak.max(),
            r_min=min(1.0 / max(ak.max(), 1e-9), 999.0),
            bad=float(bad_m.mean()), bad_n=int(bad_m.sum()),
            bad_run=longest_run(bad_m, sp),
            warn=float((ak > self.kappa_tgt).mean()),
            inside=inside, clear=clr,
            fold=bool(fold_m.any()), fold_n=int(fold_m.sum()),
            fold_run=longest_run(fold_m, sp),
            wind=wind, area=area, area_click=area_click,
            degenerate=bool(degenerate),
        )

    def prep_ok(self):
        """저자 전처리 후에도 주행 가능한가. 전처리기가 없으면 판정 보류."""
        if not self.prep or 'err' in self.prep:
            return None
        pk = np.abs(self.prep['kappa'])
        run = longest_run(pk > self.kappa_lim, self.prep['spacing'])
        self.prep['bad_run'] = run
        return run <= self.a.max_bad_run

    def ours_ok(self):
        """우리 스플라인이 통과하는가.

        ★ 접힘은 여기서 막지 않는다. 접힌 벽이 망치는 것은
          left/right_boundary.csv (참고용) 와 그 구간의 경계 여유 숫자뿐이고,
          **MPC 는 벽 좌표를 안 받는다** — 스칼라 cfg.track_width 만 쓴다
          (casadi_car_model.py:106). 주행 가능성을 가르는 것은 곡률이다.
          그래서 접힘은 경고로만 알리고, 막는 것은 곡률 하나로 한다.
        """
        s = self.stats
        if not s:
            return False
        if s['degenerate']:
            return False
        return s['inside'] and s['bad_run'] <= self.a.max_bad_run

    def redraw(self):
        self.recompute()
        ax = self.ax
        ax.clear()
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=0.25, linewidth=0.5)
        ax.set_xlabel('x [m]  (mocap world)')
        ax.set_ylabel('y [m]')

        B = np.vstack([self.V, self.V[:1]])
        ax.plot(B[:, 0], B[:, 1], '-', color='#444', linewidth=2.4,
                label=f'실험실 경계 {len(self.V)}각형')
        ax.plot(self.V[:, 0], self.V[:, 1], '.', color='#444', markersize=4)

        if self.show_ref and self.ref is not None:
            ax.plot(self.ref[:, 0], self.ref[:, 1], ':', color='#8a8a8a',
                    linewidth=1.3, label='track_5 (참고)')

        if self.pts:
            P = np.array(self.pts)
            ax.plot(P[:, 0], P[:, 1], 'o', color='#2B45C4', markersize=5,
                    zorder=5, label=f'클릭 {len(P)}점')
            if self.fit is None:
                ax.plot(np.r_[P[:, 0], P[0, 0]], np.r_[P[:, 1], P[0, 1]],
                        '-', color='#2B45C4', alpha=0.4, linewidth=1.0)

        if self.fit:
            c = self.fit['c']
            ak = np.abs(self.fit['kappa'])
            # 곡률로 색을 입힌다 — 빨강이 "못 도는 곳"
            segs = np.stack([c, np.roll(c, -1, axis=0)], axis=1)
            col = np.where(ak > self.kappa_lim, '#c0392b',
                           np.where(ak > self.kappa_tgt, '#d9a24e', '#1B6F4A'))
            ax.add_collection(LineCollection(segs, colors=col, linewidths=2.6,
                                             zorder=4))
            for w, lb in ((self.fit['left'], '좌 / 우 벽'),
                          (self.fit['right'], None)):
                ax.plot(w[:, 0], w[:, 1], '-', color='#2B45C4', alpha=0.55,
                        linewidth=1.2, label=lb)
            bad = ak > self.kappa_lim
            if bad.any():
                ax.plot(c[bad, 0], c[bad, 1], '.', color='#c0392b',
                        markersize=3, zorder=6)

        # ★ MPC 가 실제로 받는 중심선 — 우리 것과 눈으로 비교된다
        if self.show_prep and self.prep and 'err' not in self.prep:
            pc = self.prep['c']
            pk = np.abs(self.prep['kappa'])
            ax.plot(np.r_[pc[:, 0], pc[0, 0]], np.r_[pc[:, 1], pc[0, 1]],
                    '--', color='#8E5A05', linewidth=1.6, zorder=7,
                    label=f'HyperMPC 전처리 ({self.prep["n"]}점 '
                          f'{self.prep["spacing"]*100:.0f}cm)')
            over = pk > self.kappa_lim
            if over.any():
                ax.plot(pc[over, 0], pc[over, 1], 'x', color='#c0392b',
                        markersize=7, markeredgewidth=2, zorder=8)

        pad = 0.35
        ax.set_xlim(self.V[:, 0].min() - pad, self.V[:, 0].max() + pad)
        ax.set_ylim(self.V[:, 1].min() - pad, self.V[:, 1].max() + pad)
        ax.legend(loc='upper right', fontsize=8, framealpha=0.9)
        ax.set_title(self.title(), fontsize=10, loc='left')
        self.txt.set_text(self.panel())
        self.fig.canvas.draw_idle()

    def title(self):
        s = self.stats
        if not s:
            return (f'점 {len(self.pts)}개 — 4개부터 스플라인이 생깁니다  '
                    f'│ 폭 {2*self.hw:.2f} m  평활 {self.dev:.2f} m')
        ok = self.ours_ok() and self.prep_ok() is not False
        return (f'{"✓ 저장 가능" if ok else "✗ 아직 안 됨"}   '
                f'길이 {s["total"]:.2f} m   R_min {s["r_min"]:.3f} m   '
                f'불가 {100*s["bad"]:.1f}% (최장 {s["bad_run"]*100:.0f}cm '
                f'/ 허용 {self.a.max_bad_run*100:.0f}cm)   폭 {2*self.hw:.2f} m')

    def panel(self):
        s = self.stats
        L = [f'키: 좌클릭 추가 · 우클릭 삭제 · u 취소 · c 비우기 · [ ] 폭 · '
             f'- = 평활 · f 자동평활 · r 뒤집기 · t 참고 · h 전처리 · '
             f's 저장 · q 끝',
             f'차 한계: R_min {1/self.kappa_lim:.3f} m '
             f'(κ {self.kappa_lim:.3f}, max_steer {self.a.max_steer})   '
             f'권장 여유: κ ≤ {self.kappa_tgt:.2f} (R {1/self.kappa_tgt:.2f} m)']
        if not s:
            return '\n'.join(L)
        def mark(b):
            return '✓' if b else '✗'
        L.append(
            f'{mark(s["bad_run"] <= self.a.max_bad_run)} 곡률  '
            f'R_min {s["r_min"]:.3f} m   한계초과 {s["bad_n"]}점 '
            f'{100*s["bad"]:.1f}%  최장 연속 {s["bad_run"]*100:.0f}cm   '
            f'권장초과 {100*s["warn"]:.1f}%'
            f'    {mark(s["inside"])} 경계 포함   여유 {s["clear"]:.3f} m')
        if s['degenerate']:
            L.append(f'✗ 단순 폐곡선이 아닙니다 — ∮κ ds / 2π = {s["wind"]:+.2f} '
                     f'(±1 이어야 함), 면적 {s["area"]:.2f} m² '
                     f'/ 클릭 {s["area_click"]:.2f} m². '
                     f'평활화가 과해 고리가 붕괴했거나 선이 교차했습니다 — '
                     f'- 로 내리세요')
        if s['fold']:
            # 경고만 — 막지 않는다. 이유는 ours_ok() 주석 참고.
            L.append(f'⚠ 벽 접힘 {s["fold_n"]}점 (최장 {s["fold_run"]*100:.0f}cm) '
                     f'— left/right_boundary.csv 가 그 구간에서 꼬입니다. '
                     f'MPC 는 벽 좌표를 안 쓰니 주행에는 영향 없습니다. '
                     f'[ 로 폭을 줄이면 사라집니다')

        # ★ MPC 가 실제로 받는 값. 우리 스플라인과 다르므로 이 줄이 기준이다.
        if self.TrackReader is None:
            L.append('· HyperMPC 전처리 미리보기 꺼짐 '
                     '(--hypermpc 경로에 원저장소가 없습니다)')
        elif self.prep and 'err' in self.prep:
            L.append(f'· HyperMPC 전처리 실패: {self.prep["err"]}')
        elif self.prep:
            pr = self.prep
            L.append(
                f'{mark(self.prep_ok() is not False)} HyperMPC 전처리 후 (k=5, s=2.0, '
                f'{pr["n"]}점 {pr["spacing"]*100:.0f}cm)  '
                f'R_min {pr["r_min"]:.3f} m  한계초과 {100*pr["bad"]:.1f}% '
                f'(최장 {pr.get("bad_run", 0)*100:.0f}cm)  '
                f'RMSE {pr["rmse"]:.4f} m  폭보정 {pr["w_min"]:.3f}~{pr["w_max"]:.3f} m')
        return '\n'.join(L)

    # ── 저장 ────────────────────────────────────────────────────────
    def save(self):
        s = self.stats
        if not self.fit:
            print('✗ 점이 4개 미만이거나 스플라인이 안 만들어졌습니다')
            return
        if s['fold']:
            # 막지 않는다 — 알리기만 한다 (ours_ok() 주석 참고)
            print(f'⚠ 벽 접힘 {s["fold_n"]}점 (최장 {s["fold_run"]*100:.0f}cm) '
                  f'— left/right_boundary.csv 가 그 구간에서 꼬입니다.')
            print(f'  MPC 는 벽 좌표를 안 받으니 (스칼라 cfg.track_width) '
                  f'주행에는 영향 없습니다. 거슬리면 [ 로 폭을 줄이세요.')
        if s['degenerate']:
            print(f'✗ 단순 폐곡선이 아닙니다 — ∮κ ds / 2π = {s["wind"]:+.2f} '
                  f'(±1 이어야 함), 면적 {s["area"]:.2f} / '
                  f'클릭 {s["area_click"]:.2f} m²')
            print('  평활화가 과해 고리가 붕괴했거나 선이 스스로 교차합니다. '
                  '- 로 평활화를 내리세요.')
            return
        if not s['inside']:
            print('✗ 중심선이나 벽이 실험실 경계를 벗어납니다')
            return
        if s['bad_run'] > self.a.max_bad_run:
            print(f'⚠ 주행 불가 구간이 연속 {s["bad_run"]*100:.0f}cm 입니다 '
                  f'(허용 {self.a.max_bad_run*100:.0f}cm, '
                  f'{s["bad_n"]}점 {100*s["bad"]:.1f}%, '
                  f'R_min {s["r_min"]:.3f} < 한계 {1/self.kappa_lim:.3f} m)')
            print('  f 를 눌러 자동 평활화를 시도하거나, 그 코너를 더 완만하게 '
                  '찍어 보세요.')
            print('  허용치를 바꾸려면 --max-bad-run, 강행하려면 '
                  '--allow-infeasible.')
            if not self.a.allow_infeasible:
                return
        elif s['bad_n']:
            print(f'· 한계초과 {s["bad_n"]}점 (최장 연속 '
                  f'{s["bad_run"]*100:.0f}cm ≤ 허용 '
                  f'{self.a.max_bad_run*100:.0f}cm) — 국소 스파이크로 보고 넘어갑니다')
        # ★ 우리 스플라인은 통과했는데 저자 전처리 후에 걸리는 경우.
        #   MPC 가 받는 건 뒤쪽이므로 여기서 막는 게 맞다.
        if self.prep_ok() is False:
            pr = self.prep
            print(f'⚠ 우리 스플라인은 통과했지만 HyperMPC 전처리 후에 '
                  f'연속 {pr["bad_run"]*100:.0f}cm 가 주행 불가입니다 '
                  f'(R_min {s["r_min"]:.3f} → {pr["r_min"]:.3f} m)')
            print('  저자 전처리기가 k=5 / s=2.0 으로 다시 매끄럽게 하면서 '
                  '곡률이 바뀝니다.')
            print('  f 로 자동 평활화를 시도해 보세요. 강행하려면 '
                  '--allow-infeasible.')
            if not self.a.allow_infeasible:
                return

        name = self.a.name
        c, kap = self.fit['c'], self.fit['kappa']
        left, right = self.fit['left'], self.fit['right']

        # ① 주행용 — path_manager 가 읽는 형식 (# x,y). 속도는 안 적는다.
        #    적지 않으면 path_manager 가 곡률로 v_ref 를 만든다.
        wp = os.path.join(WAYPOINTS, f'{name}.csv')
        with open(wp, 'w', encoding='utf-8') as f:
            f.write(f'# x,y   {name} — tools/draw_track.py '
                    f'{time.strftime("%Y-%m-%d %H:%M")}\n')
            f.write(f'# 길이 {s["total"]:.2f} m, {len(c)}점 {self.a.spacing} m 간격, '
                    f'폭 {2*self.hw:.2f} m, R_min {s["r_min"]:.3f} m\n')
            for p in c:
                f.write(f'{p[0]:.4f},{p[1]:.4f}\n')

        # ② 트랙 자료 — lab_map/reference/tracks/* 와 같은 idx,x,y 형식
        d = os.path.join(OUTDIR, name)
        os.makedirs(d, exist_ok=True)
        for fn, arr in (('centerline.csv', c), ('left_boundary.csv', left),
                        ('right_boundary.csv', right)):
            with open(os.path.join(d, fn), 'w', encoding='utf-8') as f:
                f.write('idx,x,y\n')
                for i, p in enumerate(arr):
                    f.write(f'{i},{p[0]:.6f},{p[1]:.6f}\n')
        # ③ ★ HyperMPC 원저자 형식 — 그쪽 TrackReader 가 먹는 원본
        #    x_m,y_m,w_tr_right_m,w_tr_left_m  (한쪽씩 적는다. 저자 코드가
        #    track_width = w_r + w_l 로 전폭을 만든다)
        #    저자들 트랙은 1.1 cm 간격이다. s=2.0 평활화가 점 개수에 걸리므로
        #    같은 밀도로 깔아야 같은 결과가 나온다 (5cm 로 넣으면 과평활).
        seg, uu, total = arclen_table(self.tck)
        craw, _ = resample(self.tck, seg, uu, total, self.a.raw_spacing)
        with open(os.path.join(d, f'{name}.csv'), 'w', encoding='utf-8') as f:
            f.write('x_m,y_m,w_tr_right_m,w_tr_left_m\n')
            for q in craw:
                f.write(f'{q[0]:.6f},{q[1]:.6f},'
                        f'{self.hw:.6f},{self.hw:.6f}\n')

        with open(os.path.join(d, 'curvature_profile.csv'), 'w',
                  encoding='utf-8') as f:
            f.write('idx,s,kappa\n')
            for i, k in enumerate(kap):
                f.write(f'{i},{i*self.a.spacing:.4f},{k:.6f}\n')

        meta = {
            'name': name,
            'created': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'source': 'tools/draw_track.py (손클릭 + 닫힌 주기 스플라인)',
            'coordinate_frame': {
                'name': 'mocap_world', 'unit': 'meter',
                'note': '경계 JSON 과 같은 프레임. /car/pose 와 변환 없이 비교 가능',
            },
            'boundary_source': os.path.relpath(self.a.boundary, REPO),
            'generation': {
                'clicked_points': [[float(p[0]), float(p[1])] for p in self.pts],
                'spline': 'scipy.interpolate.splprep(k=3, per=1)',
                'smooth_dev_m': self.dev,
                'spacing_m': self.a.spacing,
                'half_width_m': self.hw,
            },
            'vehicle_limits': {
                'wheelbase': self.a.wheelbase,
                'max_steer': self.a.max_steer,
                'kappa_limit': self.kappa_lim,
                'r_min_car': 1.0 / self.kappa_lim,
            },
            'geometry': {
                'closed': True,
                'path_length_m': float(s['total']),
                'num_points': int(len(c)),
                'max_abs_kappa': float(s['k_max']),
                'min_radius_m': float(s['r_min']),
                'infeasible_fraction': float(s['bad']),
                'infeasible_points': int(s['bad_n']),
                'infeasible_longest_run_m': float(s['bad_run']),
                'max_bad_run_m': float(self.a.max_bad_run),
                'wall_fold_points': int(s['fold_n']),
                'wall_fold_longest_run_m': float(s['fold_run']),
                'winding_number': float(s['wind']),
                'enclosed_area_m2': float(s['area']),
                'track_width_m': 2 * self.hw,
                'wall_to_boundary_clearance_m': float(s['clear']),
                'inside_lab_boundary': bool(s['inside']),
            },
        }
        if self.prep and 'err' not in self.prep:
            pr = self.prep
            meta['hypermpc_preprocessed'] = {
                'note': '저자 track_preprocesor.TrackReader 로 낸 값. '
                        'prep_<이름>.csv 는 ./drive.sh preptrack 으로 만든다',
                'rmse_m': pr['rmse'], 'num_points': pr['n'],
                'spacing_m': pr['spacing'], 'path_length_m': pr['total'],
                'min_radius_m': pr['r_min'],
                'infeasible_fraction': pr['bad'],
                'track_width_corrected_m': [pr['w_min'], pr['w_max']],
            }
        with open(os.path.join(d, 'track.json'), 'w', encoding='utf-8') as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)

        print(f'\n✓ 저장했습니다')
        print(f'    주행용   {os.path.relpath(wp, REPO)}')
        print(f'    트랙자료 {os.path.relpath(d, REPO)}/')
        print(f'    HyperMPC {os.path.relpath(os.path.join(d, name + ".csv"), REPO)}'
              f'   ({len(craw)}점 {self.a.raw_spacing*100:.1f} cm — 저자 형식)')
        print(f'    길이 {s["total"]:.2f} m · {len(c)}점 · 폭 {2*self.hw:.2f} m · '
              f'R_min {s["r_min"]:.3f} m · 벽~경계 {s["clear"]:.3f} m')
        if self.prep and 'err' not in self.prep:
            pr = self.prep
            print(f'    전처리 후 R_min {pr["r_min"]:.3f} m · {pr["n"]}점 '
                  f'{pr["spacing"]*100:.0f}cm · RMSE {pr["rmse"]:.4f} m')
        print(f'\n  다음:')
        print(f'    ① src/mlcs_mpc/config/track.yaml 의 waypoint_file 을 '
              f"'{name}.csv' 로")
        print(f'    ② ./drive.sh trackinfo        기하 재확인')
        print(f'    ③ ./drive.sh stanley --bag    첫 주행은 느리게')
        print(f'    ④ ./drive.sh preptrack {name}   HyperMPC 용 prep_*.csv 생성\n')


def main():
    p = argparse.ArgumentParser(
        description='손클릭 레이싱 라인 → 스플라인 → 좌/우 벽',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('name', nargs='?', default=None,
                   help='트랙 이름 (기본: hand_날짜시각)')
    p.add_argument('--boundary', default=BOUNDARY)
    p.add_argument('--half-width', type=float, default=0.25,
                   help='중심선에서 벽까지 (기본 0.25 — HyperMPC 원저자 값)')
    p.add_argument('--spacing', type=float, default=0.05,
                   help='주행용 웨이포인트 간격 m (기본 0.05)')
    p.add_argument('--raw-spacing', type=float, default=0.01,
                   help='HyperMPC 원본 CSV 간격 m (기본 0.01 — 저자들 트랙이 '
                        '1.1 cm 다. 그쪽 s=2.0 평활화가 점 개수에 걸리므로 '
                        '밀도를 맞춰야 같은 결과가 나온다)')
    p.add_argument('--smooth', type=float, default=0.03,
                   help='손떨림을 무시할 정도 m (기본 0.03). 0 이면 찍은 점을 '
                        '정확히 통과하지만 울퉁불퉁해진다')
    p.add_argument('--wheelbase', type=float, default=0.33)
    p.add_argument('--max-steer', type=float, default=0.40,
                   help='실측값 (2026-09-30). vehicle.yaml 과 같아야 한다')
    p.add_argument('--kappa-target', type=float, default=1.0,
                   help='권장 곡률 상한. 한계 1.281 에 붙이면 여유가 없다')
    p.add_argument('--hypermpc', default='~/hypermpc_code',
                   help='원저장소 경로. 그리는 중에 저자 전처리 결과를 같이 '
                        '보여준다 (없으면 미리보기만 빠진다)')
    p.add_argument('--max-bad-run', type=float, default=0.15,
                   help='한계를 넘는 구간이 연속으로 이 길이를 넘으면 막는다 '
                        '(m, 기본 0.15 = 5cm 세 칸). "한 점이라도" 로 막으면 '
                        '손으로 찍는 입력에서는 통과가 거의 불가능하다. '
                        '0 으로 주면 예전처럼 한 점도 허용하지 않는다')
    p.add_argument('--allow-infeasible', action='store_true',
                   help='주행 불가 구간이 있어도 저장 (권장하지 않음)')
    a = p.parse_args()
    if a.name is None:
        a.name = 'hand_' + time.strftime('%m%d_%H%M')

    if not os.path.isfile(a.boundary):
        print(f'✗ 경계 파일이 없습니다: {a.boundary}')
        sys.exit(1)

    V, _ = load_boundary(a.boundary)
    print(f'■ 실험실 경계 {len(V)}각형  '
          f'x [{V[:,0].min():+.2f}, {V[:,0].max():+.2f}]  '
          f'y [{V[:,1].min():+.2f}, {V[:,1].max():+.2f}]')
    print(f'■ 차 한계  R_min {a.wheelbase/math.tan(a.max_steer):.3f} m  '
          f'(max_steer {a.max_steer})')
    print(f'■ 창에서 클릭하세요. 4점부터 스플라인이 생깁니다. s 로 저장.\n')

    drawer = TrackDrawer(a)     # ★ 반환값을 반드시 받아 둔다 (위 주석 참고)
    plt.show()
    del drawer


if __name__ == '__main__':
    main()
