# 캘리브레이션 — mocap 을 자로 쓴다

MPC 는 "명령한 대로 차가 움직인다" 를 전제로 1초 앞을 예측한다.
그 전제가 틀리면 예측이 통째로 틀리고, 게인을 아무리 튜닝해도 낫지 않는다.

**이 문서의 절차를 끝내기 전에는 저속(1~1.5 m/s)으로만 주행하세요.**

---

## 왜 이게 먼저인가 — 실제 사례

26-jetson 저장소 2호차는 `speed_to_erpm_gain` 이 1호차 값(4203)을 그대로
빌려 쓴 상태로 달렸다. 나중에 재보니 정답은 4403 이었다:

```
지령 3.0 m/s → erpm 4203×3.0 = 12609 → 실제 12609/4403 = 2.864 m/s
                                      → odom 은 12609/4203 = 3.000 이라 보고
```

**차는 4.5% 느리게 달리고, odom 은 거리를 4.8% 과대보고** 하고 있었다.
그 상태에서 잰 다른 값들(`speed_lag`, `slip_gain`)도 전부 다시 재야 했다.

우리는 mocap 이 있다. 줄자보다 정확하고 빠르다 — 처음부터 제대로 재자.

---

## 준비

```bash
# 터미널 1 — mocap
ros2 launch natnet_ros2 natnet_ros2.launch.py ...

# 터미널 2 — mocap 브릿지
ros2 run mlcs_mpc mocap_bridge --ros-args \
    --params-file ~/mlcs_mpc/src/mlcs_mpc/config/mocap.yaml

# 터미널 3 — VESC 드라이버
ros2 launch f1tenth_stack bringup_launch.py
```

확인:
```bash
ros2 topic hz /mpc/state      # 100Hz 근처
```

> 조이스틱으로 차를 옮겨야 하면 터미널을 하나 더 띄우세요.
> **`joy:=false` 를 꼭 붙이세요** — bringup 이 joy_node 를 이미 띄웁니다.
>
> ```bash
> ros2 launch mlcs_mpc joystick.launch.py joy:=false
> ```
>
> 캘리브레이션 중에는 `calibrate` 노드가 `/drive` 로 명령을 내고, 조이스틱은
> `/teleop` 으로 냅니다. mux 우선순위상 **조이스틱이 이기므로**, RT 를 건드리면
> 측정이 오염됩니다. 측정 중에는 조이스틱에서 손을 떼세요.

> ⚠ **넓은 공간에서, 킬스위치를 손에 들고** 하세요.
> `steer` 모드는 차가 원을 그리며 돕니다.

---

## 순서 — 이 순서를 지킬 것

뒤 단계가 앞 단계에 의존한다. `speed` 가 틀리면 그 위 전부가 틀린다.

### ① 조향 중립 (offset)

직진 명령을 줬을 때 실제로 직진하는가?

```bash
ros2 run mlcs_mpc calibrate --ros-args -p mode:=neutral -p speed_for_steer:=1.0
```

출력의 "등가 조향각" 이 0 에 가까워야 한다. 아니면 안내대로
`f1tenth_stack/config/vesc.yaml` 의 `steering_angle_to_servo_offset` 을 고친다.

```yaml
steering_angle_to_servo_offset: 0.5304   # ← 이 값을 고친다
```

**고친 뒤 다시 재서 요레이트가 0 에 가까워졌는지 확인한다.** 한 번에 안 맞으면
반복한다.

### ② 속도 게인

```bash
ros2 run mlcs_mpc calibrate --ros-args -p mode:=speed -p value:=1.5
ros2 run mlcs_mpc calibrate --ros-args -p mode:=speed -p value:=2.5   # 다른 속도로도
```

두 속도에서 나온 보정계수가 비슷해야 한다. 많이 다르면 선형 가정이
안 맞는 것이니 (배터리 전압 강하, 마찰) 실제 주행 속도 범위에서 재라.

```yaml
speed_to_erpm_gain: 4614.0    # ← 출력이 알려주는 값으로 고친다
```

> **부호 주의:** 양수 speed 명령에 차가 **후진** 하면 gain 부호가 반대다.
> 26-jetson 2호차가 실제로 그랬고, 그 탓에 SLAM 맵이 방사형으로 깨졌다.
> 조이스틱만 invert 로 덮여 있어서 수동 주행은 정상처럼 보였다 —
> **증상을 덮으면 원인이 숨는다.**

### ③ 조향 게인

좌우 **양쪽** 을 잰다.

```bash
ros2 run mlcs_mpc calibrate --ros-args -p mode:=steer -p value:=0.2
ros2 run mlcs_mpc calibrate --ros-args -p mode:=steer -p value:=-0.2
```

좌우 비율이 다르면 ①의 중립이 아직 안 맞은 것이다 — ①로 돌아간다.

```yaml
steering_angle_to_servo_gain: -1.2135   # ← 출력이 알려주는 값
```

> 26-jetson 은 손으로 밀어 그린 원에서 얻은 값이 22% 틀렸고, 주행 로그로
> 다시 잡았다. **차가 실제로 달리는 상태에서** 재는 것이 중요하다 —
> 정지 마찰과 주행 중 타이어 거동이 다르기 때문이다.

### ④ 휠베이스

줄자로 앞바퀴축 중심 ~ 뒷바퀴축 중심.

```yaml
# config/vehicle.yaml
wheelbase: 0.33
```

교차 검증: ③에서 나온 선회반경 R 과 조향각 δ 가
`tan(δ) = L/R` 을 만족해야 한다.

### ⑤ 최대 조향각

서보를 기구 스토퍼까지 돌려서 확인한다. 스토퍼를 넘게 명령하면 서보가
계속 힘을 쓰다 탄다.

```yaml
max_steer: 0.36    # rad
```

---

## 반영할 파일 — 두 군데다

| 값 | 파일 | 쓰는 곳 |
|---|---|---|
| `speed_to_erpm_gain` | `f1tenth_stack/config/vesc.yaml` | VESC 드라이버 |
| `steering_angle_to_servo_gain/offset` | `f1tenth_stack/config/vesc.yaml` | VESC 드라이버 |
| `wheelbase`, `max_steer`, `mu` | `mlcs_mpc/config/vehicle.yaml` | MPC 예측 모델 |

> ⚠ `wheelbase` 는 **양쪽에 다 있다** (`vesc.yaml` 의 `vesc_to_odom_node`,
> `vehicle.yaml` 의 MPC). 한쪽만 고치면 어긋난다.

---

## 검증 — 잘 됐는지 어떻게 아는가

캘리브레이션 후 원을 한 바퀴 돌려보고:

```bash
ros2 topic echo /mpc/state --field twist.twist.linear.x
```

- 명령 속도와 실제 속도가 **5% 이내**
- 좌/우 선회반경이 **비슷**
- 직진 명령에서 요레이트가 **0 근처**

세 가지가 다 맞으면 속도를 올려도 된다. `config/vehicle.yaml` 의
`max_speed` 를 0.5 m/s 씩 올리면서, 매번 경로 추종 오차를 확인한다.

---

## 다음에 재야 할 것 (동역학 모델로 갈 때)

| 값 | 방법 |
|---|---|
| `mass` | 저울 |
| `lf`, `lr` | 앞/뒤 축을 각각 저울에 올려 하중 분포로 계산 |
| `mu` | 점점 빠르게 선회시키며 미끄러지기 시작하는 속도 |

→ `docs/ROADMAP.md`
