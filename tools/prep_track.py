#!/usr/bin/env python3
"""
prep_track.py — HyperMPC 원저자 전처리기를 우리 트랙에 돌린다.

  ./drive.sh preptrack <이름> [--hypermpc ~/hypermpc_code]

■ 왜 이 단계가 따로 있나

  저자들 MPC 는 트랙을 두 단계로 쓴다.

    <이름>.csv        x_m,y_m,w_tr_right_m,w_tr_left_m
                      ↓  mpc/tracks/track_preprocesor.py : TrackReader
    prep_<이름>.csv   s,x,y,heading,curvature,track_width
                      ↓  mpc/tracks/map_reader.py : getTrackCustom
                      MPC

  **우리가 계산을 다시 하지 않는다.** 저자 코드를 그대로 import 해서
  돌린다. 곡률·heading·s 를 우리 방식으로 다시 내면 "같은 정식화" 라고
  말할 수 없게 된다.

■ 저자 설정 (track_preprocesor.py 그대로)

    splprep(k=5, s=2.0, w=1/전폭, per=True)   5차 주기 스플라인
    points_per_meter = 5                      → 20 cm 간격으로 재샘플
    savgol_filter(w_r, 10, 3)                 오른쪽 폭만 평활
    track_width = w_r + w_l                   전폭
    track_width_corrected = (전폭/2 - e)·2    스플라인 오차만큼 좁힌다

■ ★ 알고 있어야 할 두 가지

  ① **진행 방향이 뒤집힌다.** TrackReader.__init__ 이 reverse=False 일 때
     np.flip(data, axis=0) 을 한다 (저자 기본값). 그래서 CCW 로 그린
     트랙이 CW 가 되고 곡률 부호가 반대로 나온다. --reverse 로 끈다.

  ② **s=2.0 은 점 개수에 걸린다.** scipy 의 s 는 가중 잔차 제곱합이라
     점이 적으면 점당 허용 오차가 커진다. 저자들 원본은 1.1 cm 간격
     (1367~1703점) 이다. 우리도 draw_track.py 가 --raw-spacing 0.01 로
     깔아 두므로 맞는다. 5 cm 로 넣으면 RMSE 가 3.0 → 4.8 cm 로 벌어지고
     R_min 이 줄어든다 (실측).

■ scipy 서브모듈

  저자 코드가 `import scipy` 만 하고 scipy.signal / interpolate / integrate
  를 쓴다. 그쪽 환경에서는 다른 import 가 먼저 올려 줘서 가려졌던 문제다.
  여기서 먼저 올려 준다 — 저자 코드는 손대지 않는다.
"""
import argparse
import os
import pathlib
import shutil
import sys

import numpy as np

# ★ 저자 코드가 안 올리는 서브모듈. TrackReader import 보다 먼저.
import scipy.integrate      # noqa: F401
import scipy.interpolate    # noqa: F401
import scipy.signal         # noqa: F401

os.environ.setdefault('MPLBACKEND', 'Agg')   # 창을 띄우지 않는다

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HANDDRAWN = os.path.join(REPO, 'lab_map/data/tracks')


def main():
    p = argparse.ArgumentParser(
        description='HyperMPC 원저자 TrackReader 로 prep_<이름>.csv 생성',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('name', help='트랙 이름 (lab_map/data/tracks/<이름>/)')
    p.add_argument('--hypermpc', default=os.path.expanduser('~/hypermpc_code'),
                   help='원저장소 경로 (기본 ~/hypermpc_code)')
    p.add_argument('--reverse', action='store_true',
                   help='저자 기본값인 방향 뒤집기를 끈다 (그린 방향 유지)')
    p.add_argument('--no-corrected', action='store_true',
                   help='폭을 스플라인 오차만큼 좁히지 않는다')
    p.add_argument('--install', action='store_true',
                   help='원저장소 mpc/tracks/ 에도 복사한다')
    p.add_argument('--max-steer', type=float, default=0.40)
    p.add_argument('--wheelbase', type=float, default=0.33)
    a = p.parse_args()

    tracks = os.path.join(a.hypermpc, 'mpc/tracks')
    if not os.path.isdir(tracks):
        print(f'✗ 원저장소가 없습니다: {tracks}')
        print(f'  git clone https://github.com/hyper-mpc/hypermpc_code.git '
              f'{a.hypermpc}')
        sys.exit(1)
    sys.path.insert(0, tracks)
    from track_preprocesor import TrackReader       # noqa: E402
    from map_reader import getTrackCustom           # noqa: E402

    d = os.path.join(HANDDRAWN, a.name)
    raw = os.path.join(d, f'{a.name}.csv')
    if not os.path.isfile(raw):
        print(f'✗ 원본이 없습니다: {raw}')
        print(f'  먼저 ./drive.sh drawtrack {a.name} 으로 만드세요.')
        sys.exit(1)

    src = np.loadtxt(raw, delimiter=',', skiprows=1)
    print(f'■ 원본  {os.path.relpath(raw, REPO)}')
    L = np.sum(np.linalg.norm(
        np.diff(np.vstack([src[:, :2], src[:1, :2]]), axis=0), axis=1))
    print(f'   {len(src)}점  길이 {L:.3f} m  간격 {L/len(src)*100:.2f} cm'
          f'   반폭 {src[:,2].min():.3f}~{src[:,3].max():.3f} m')
    if L / len(src) > 0.02:
        print(f'   ⚠ 저자들 트랙은 1.1 cm 간격입니다. 이보다 성기면 s=2.0 '
              f'평활화가 과하게 걸립니다.')

    t = TrackReader(pathlib.Path(raw), reverse=a.reverse,
                    corrected=not a.no_corrected)
    ak = np.abs(t.re_curvature)
    k_lim = np.tan(a.max_steer) / a.wheelbase
    bad = float((ak > k_lim).mean())

    print(f'\n■ TrackReader (저자 설정: k=5, s={t.splprep_s}, w=1/전폭, '
          f'{t.points_per_meter}점/m)')
    print(f'   RMSE        {t.rmse:.4f} m   스플라인이 원본에서 벗어난 정도')
    print(f'   트랙 길이   {t.track_lenght:.3f} m')
    print(f'   재샘플      {t.re_N}점  ({t.track_lenght/t.re_N*100:.1f} cm 간격)')
    print(f'   곡률        {t.re_curvature.min():+.4f} ~ '
          f'{t.re_curvature.max():+.4f}  /m')
    print(f'   ∮κ ds      {np.trapz(t.re_curvature, t.re_path_s):+.4f}'
          f'   (닫힌 고리면 ±{2*np.pi:.4f})')
    print(f'   폭 원본     {t.track_width.min():.4f}~{t.track_width.max():.4f} m'
          f'  (전폭 = w_r + w_l)')
    print(f'   폭 보정     {t.re_track_width_corrected.min():.4f}~'
          f'{t.re_track_width_corrected.max():.4f} m')
    sign = '시계(CW)' if np.trapz(t.re_curvature, t.re_path_s) < 0 else '반시계(CCW)'
    print(f'   진행 방향   {sign}'
          f'   {"(저자 기본값대로 뒤집음)" if not a.reverse else "(그린 방향 유지)"}')

    print(f'\n■ 우리 차 검사  (max_steer {a.max_steer} → R_min '
          f'{1/k_lim:.3f} m, κ ≤ {k_lim:.3f})')
    print(f'   {"✓" if bad == 0 else "✗"} R_min {1/ak.max():.3f} m'
          f'   한계초과 {100*bad:.1f}%')
    if bad > 0:
        print(f'   ⚠ 전처리가 곡률을 바꿉니다. draw_track 에서 통과해도 '
              f'여기서 걸릴 수 있습니다.')

    out = os.path.join(d, f'prep_{a.name}.csv')
    t.save_track(out)
    print(f'\n✓ {os.path.relpath(out, REPO)}')
    print(f'   {open(out).readline().strip()}')

    # 저자 코드로 역로드 검증 — mpc/tracks 안에 있어야 읽는다
    tmp = os.path.join(tracks, f'_verify_{a.name}.csv')
    try:
        shutil.copy(out, tmp)
        s, x, y, h, k = getTrackCustom(os.path.basename(tmp))
        print(f'✓ getTrackCustom 역로드  {len(s)}점  s {s[0]:.2f}~{s[-1]:.2f}  '
              f'heading {h.min():+.2f}~{h.max():+.2f}')
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)

    if a.install:
        for f in (raw, out):
            shutil.copy(f, os.path.join(tracks, os.path.basename(f)))
        print(f'✓ 원저장소에 복사: {tracks}/')
        print(f'   conf_mpc/config_car.yaml 의 track_name 을 '
              f"'{a.name}' 으로 바꾸면 그쪽에서 바로 씁니다.")
    else:
        print(f'\n  원저장소에 넣으려면 --install')


if __name__ == '__main__':
    main()
