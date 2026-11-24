# 대회장 트랙 (MAGP 2026, 11-14 ~ 11-15)

출처: GitHub `jjujjujjuny/mlcs_mpc` 커밋 `cf8b4ad` (11-15 결승 최종). 원본과 md5 일치 확인.

## 폴더

| 폴더 | 내용 | 원래 위치 |
|---|---|---|
| `maps/` | Cartographer 맵 — `<이름>.yaml` (해상도 0.05 m, 원점), `.pgm` (점유 격자), `.pbstream` (위치추정용) | `maps/` |
| `tracks/<이름>_track/` | MPC 트랙 파일 | `lab_map/data/tracks/` |
| `그림/` | 전체 맵 그림, HyperMPC 트랙과 같은 축척 비교 | (새로 그림) |

트랙 폴더 안의 파일:
- `prep_<이름>_track.csv` — **MPC 가 읽는 파일**. 열: `s, x, y, heading, curvature, track_width, w_right, w_left` (맵 좌표, 0.05 m 간격, 닫힌 트랙)
- `centerline.csv`, `left_boundary.csv`, `right_boundary.csv`, `curvature_profile.csv`, `<이름>_track.csv`, `track.json` (생성 정보·기하 요약), `map_track.png`, `preview.png`

## 어떤 맵을 언제 썼나

| 맵 | 쓴 때 |
|---|---|
| track1, track2 | 11-14 오전 첫 맵 (노이즈 많아 다시 만듦) |
| track1a, track2a | 11-14 연습 (번갈아) |
| track1b | 11-15 오전 |
| **track1c** | 11-15 예선 2차 ~ 결승 (최종) |

## 크기 (기준선 기준)

| 트랙 | 둘레 | 크기 | 폭 (중앙) | 최소 반경 |
|---|---|---|---|---|
| track1c | 58.5 m | 11.3 × 9.0 m | 1.81 m | 0.57 m |
| track2a | 54.6 m | 11.0 × 8.4 m | 1.73 m | 0.48 m |
| HyperMPC lab_curvy_v1 | 18.2 m | 3.7 × 6.2 m | 0.23 m | 0.56 m |
| HyperMPC lab_monza | 17.5 m | 4.2 × 5.8 m | 0.52 m | 0.44 m |
| HyperMPC abudhabi | 18.0 m | 4.4 × 5.9 m | 0.56 m | 0.48 m |
| HyperMPC lab_usa_v3 | 14.2 m | 2.6 × 5.6 m | 1.10 m | 0.98 m |

HyperMPC 트랙은 저자 저장소 `hypermpc_code/mpc/tracks/prep_*.csv` 의 `track_width` 기준.

**HyperMPC 크기로 만들 때 주의**: 대회장 트랙은 둘레가 약 3배다. 그대로 0.3 배 줄이면 최소 반경이
0.17 m 가 되는데, 우리 차의 최대 조향 반경은 약 0.83 m (실측 0.82~0.86) 라 달릴 수 없다.
코너 반경은 유지하고 직선·코너 수를 줄인 축소판으로 다시 그려야 한다.

그림의 벽선이 안쪽 코너에서 뾰족하게 튀는 곳은 벽까지 거리(w_left/w_right)를 기준선에서 그대로
옮겨 그려서 생긴 표현상의 꺾임이다. 실제 벽은 바탕의 점유 격자(검은 선)를 보면 된다.
