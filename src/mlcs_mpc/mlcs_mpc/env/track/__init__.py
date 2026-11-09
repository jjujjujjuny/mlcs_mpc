"""
track — 트랙 코드 (track.py) 와 트랙 데이터 (data/<이름>/prep_<이름>.csv).

  from mlcs_mpc.env.track import Track, find
  Track(find('comp1_track'))        이름 또는 prep csv 경로
"""
import os

from mlcs_mpc.env.track.track import Track, wrap  # noqa: F401

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


def find(name):
    """트랙 이름 (data/<이름>/prep_<이름>.csv) 또는 파일 경로 → prep csv 절대경로."""
    if os.path.isfile(name):
        return os.path.abspath(name)
    path = os.path.join(DATA_DIR, name, f'prep_{name}.csv')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'트랙을 못 찾았습니다: {name} ({path})')
    return path
