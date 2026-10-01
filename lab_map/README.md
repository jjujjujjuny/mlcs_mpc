# lab_map — 실험실 경계와 트랙 생성 도구

메인 PC(`~/Desktop/map_data`, `~/Desktop/turtlebot_data/code/lab_map`)에서
가져온 것이다. DeepRacer 시절 만든 자산이고, 트랙을 **새로 만들거나
곡률을 다시 제한할 때** 필요하다.

> 미리보기 PNG(6.1 MB)는 뺐다. 데이터·스크립트·트랙 원본만 1.8 MB.

---

## 지금 당장 필요한 것은 하나다

```
data/boundary/map_boundary_data.json
```

**실험실 경계 19각형** (면적 24.13 m², 5.78 × 6.74 m). mocap 월드 좌표계
기준이라 우리 트랙·`/mpc/state` 와 **같은 프레임**이다. 변환이 필요 없다.

용도 두 가지:

1. **트랙이 실험실 안에 들어가는지 검사** — `track_5/_generate.py` 가
   이 파일로 중심선·좌·우 경계의 모든 점을 확인한다
2. **`safety_node` 경계값의 근거** — 지금 `safety.yaml` 의 ±3.0 은
   예시값이다. 이 경계가 실측이다

```python
import json, numpy as np
d = json.load(open('data/boundary/map_boundary_data.json'))
v = np.array([[p['x'], p['y']] for p in d['boundary']['vertices']])
# v.shape == (19, 2),  x[-2.79,+2.98]  y[-3.32,+3.42]
```

---

## 디렉터리

```
data/
  boundary/              ★ 실험실 경계 (19각형)
      map_boundary_data.json       ← 이게 핵심
      map_boundary_vertices.csv
      map_boundary_summary.md

  centerline/            손으로 클릭한 중심선 (track_1 의 원본)
      latest_reclicked_centerline.json
      latest_clicked_centerline_waypoints.csv
      latest_smooth_centerline.csv

  curvature_limited/     곡률 제한을 건 결과 = track_1 그 자체
      latest_centerline.csv          566점 / 17.00 m
      latest_left_boundary.csv
      latest_right_boundary.csv
      latest_curvature_profile.csv
      latest_curvature_limited_track.json    ← converged:false 기록

  linear/                직선 트랙 (참고용)

scripts/                 트랙 생성 (ROS 불필요 — 젯슨에서 돌아감)
  make_curvature_limited_track.py     ★ 곡률 제한
  make_curved_track.py / make_smooth_curved_track.py
  make_linear_track.py
  reclick_centerline_with_reference.py   중심선 손클릭 (GUI)
  draw_track_on_boundary.py / redraw_track_on_boundary.py
  make_outer_boundary_map_from_trace.py
  make_polygon_map_from_trace.py
  create_lab_map_from_corners.py / create_map_from_manual_stops.py
  plot_manual_boundary_trace.py
  test_track_query.py

scripts/ros1_only/       ★ 젯슨에서 못 돈다 (rospy 필요)
  record_manual_lab_boundary.py       경계를 차로 돌며 기록
  collect_lab_corners.py
  arrow_key_teleop.py
  number_key_turtlebot_api_teleop.py
  run_pure_pursuit_track.py

reference/
  track_5_generate.py    ★ 해석적 트랙 생성 본보기 (새 트랙은 이걸 복사)
  tracks/track_1..5/     DeepRacer 트랙 5종 원본 (centerline/경계/곡률/json)
```

> `reference/tracks/` 는 `deepracer_mpc_low_level/tracks/` 를 그대로 옮긴
> 것이다(PNG 제외). 젯슨에서 `import_track` 으로 가져다 쓸 수 있다:
>
> ```bash
> ros2 run mlcs_mpc import_track --ros-args \
>     -p src:=~/mlcs_mpc/lab_map/reference/tracks/track_5 \
>     -p out:=~/mlcs_mpc/src/mlcs_mpc/waypoints/track_5.csv
> ```

---

## ⚠ 경로가 하드코딩돼 있다

스크립트들이 `/home/user/Desktop/map_data/...` 를 직접 가리킨다
(메인 PC 기준). 젯슨에서 돌리려면 바꿔야 한다.

가장 간단한 방법 — **심링크를 만든다**:

```bash
sudo mkdir -p /home/user/Desktop
sudo ln -sfn ~/mlcs_mpc/lab_map/data /home/user/Desktop/map_data
```

> 실제 경로를 흉내 내는 것이 스크립트를 고치는 것보다 안전하다. 스크립트가
> 서로를 참조하고 중간 산출물 경로도 공유하므로, 한 군데만 고치면 어긋난다.

또는 각 스크립트 상단의 경로 상수를 직접 고친다 (파일마다 다름):

```python
BOUNDARY_JSON = "/home/user/Desktop/map_data/map_boundary_data/map_boundary_data.json"
SAVE_ROOT     = "/home/user/Desktop/map_data/track_curvature_data"
```

---

## ★ 곡률 제한을 다시 돌릴 때 — 이게 핵심이다

`docs/TRACKS.md` 에 적었듯, track_1 의 곡률 제한은 **수렴에 실패했다**:

```json
"parameters": { "kappa_max": 1.2,  "max_iter": 250 },
"metrics":    { "max_abs_kappa": 3.406 },
"converged":  false
```

요청 κ≤1.2 인데 결과가 3.406 — 약 3배 급하다. 그래서 track_1~4 는
우리 차로 33~42% 구간을 못 돈다.

다시 돌린다면:

```bash
python3 scripts/make_curvature_limited_track.py --help    # 인자 확인
```

**우리 차 기준값:**

| | 값 |
|---|---|
| 최소 선회반경 | `L/tan(δmax)` = 0.33/tan(0.40) = **0.781 m** |
| 곡률 한계 | κ = **1.281** |
| 권장 `kappa_max` | **1.0** (반경 1.0 m, 여유 있음) |

> `max_steer` 가 0.36 → 0.40 으로 바뀌었다 (실측). 한계가 조금 넉넉해졌지만
> 여전히 κ=1.0 정도로 여유를 두는 것이 안전하다.

⚠ 곡률을 낮추면 코너가 완만해져 **트랙이 커진다.** 경계 안에 들어가는지
스크립트가 검사하므로, 안 들어가면 트랙 크기를 줄여야 한다.

---

## 새 트랙을 만드는 두 가지 길

### ① 해석적 생성 (권장)

`reference/track_5_generate.py` 가 본보기다.
반경을 상수로 두므로 **곡률이 설계로 보장**되고, 경계 포함 검사도 들어 있다.
수치 최적화의 "수렴 실패" 가 구조적으로 불가능하다.

8자나 역방향 트랙도 이쪽이 쉽다 — 수식만 바꾸면 된다.

### ② 손클릭 + 곡률 제한 (원래 방식)

```
reclick_centerline_with_reference.py   경계 위에 중심선 클릭 (GUI 필요)
            ↓
make_curvature_limited_track.py        곡률 제한 (κ_max=1.0 으로)
```

실험실 모양에 맞춘 비정형 트랙을 만들 때 쓴다. 다만 수렴을 확인해야 한다
(`converged: true` 인지 JSON 에서 확인).

---

## 출처

- 데이터: `~/Desktop/map_data/` (메인 PC)
- 스크립트: `~/Desktop/turtlebot_data/code/lab_map/` (메인 PC)
- 관련 문서: [`docs/TRACKS.md`](../docs/TRACKS.md) — 트랙 5종 조사 결과
