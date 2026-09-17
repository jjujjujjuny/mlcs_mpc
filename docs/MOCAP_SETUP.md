# 모션캡처(OptiTrack) 연동 — Ground Truth 위치를 토픽으로

라이다가 없으므로 **이 단계가 위치추정의 전부**다. 여기가 안 되면 MPC 도,
캘리브레이션도, 웨이포인트 기록도 못 한다.

목표:

```
Motive (2번 PC)  ──NatNet/UDP──▶  natnet_ros2 (젯슨)  ──▶  /<이름>/pose
                                                              │
                                              mocap_bridge ◀──┘
                                                   │
                                          /mpc/state (Odometry)  ← MPC 가 쓰는 것
```

---

## ★ 이 장비 구성에서 먼저 알아야 할 것

**2번 PC(Motive)와 1번 PC는 유선, 젯슨은 무선.** 이 구성 때문에 두 가지가
강제된다.

### ① Transmission Type 은 반드시 **Unicast**

Multicast 는 무선 AP 에서 걸러지거나 유선↔무선 구간을 못 넘는 일이 흔하다.
Motive 쪽이 Multicast 면 젯슨에서 **연결은 되는데 데이터가 안 오는** 상태가
되고, 원인을 찾기 아주 어렵다 (에러가 안 난다).

> Unicast 는 클라이언트가 필요한 데이터만 구독해 패킷이 작아지므로,
> 패킷 손실에 취약한 **무선 클라이언트에 특히 유리**하다 — OptiTrack 공식
> 문서가 그렇게 권한다.

### ② 젯슨과 Motive PC 가 **같은 서브넷**이어야 한다

유선(예: `192.168.1.x`)과 무선(예: `192.168.0.x`)이 서로 다른 대역이면
NatNet 이 닿지 않는다. 공유기가 같아도 유선/무선을 다른 대역으로 쪼개는
설정이 있으니 **IP 를 직접 확인**해야 한다 (아래 1단계).

### ③ 무선은 지연·유실이 있다 — 나중에 이게 원인이 될 수 있다

mocap 이 100Hz 인데 무선에서 몇 프레임씩 빠지면 `mocap_bridge` 의
워치독(`timeout: 0.2`)이 걸려 차가 멈춘다. 처음엔 정상이다가 주행 중에만
끊기면 **무선을 의심**하라. 정 안 되면 젯슨을 유선으로 옮기는 것이 정답이다.

---

## 1단계 — 네트워크 확인 (제일 먼저)

**젯슨에서:**

```bash
ip -4 addr show | grep inet
```

**2번 PC(Motive, Windows)에서** 명령 프롬프트:

```cmd
ipconfig
```

두 IP 의 **앞 세 자리가 같아야 한다**:

```
Motive PC : 192.168.1.10
젯슨      : 192.168.1.52     ← 앞 세 자리 192.168.1 이 일치 ✓
```

다르면 여기서 멈추고 네트워크부터 고쳐야 한다. 같은 공유기에 붙어도
유선/무선 대역이 갈려 있으면 안 된다.

서로 보이는지 확인 — **젯슨에서**:

```bash
ping <Motive PC IP>
```

응답이 없으면 Windows 방화벽이 ICMP 를 막는 것일 수 있다(정상일 수도
있음). 그래도 아래 4단계에서 데이터가 오면 문제없다.

메모해 둘 것:

```
Motive PC IP (serverIP) = ____________
젯슨 IP      (clientIP) = ____________
```

---

## 2단계 — 차량에 RigidBody 만들기 (Motive)

### 마커 부착 — 여기서 대충 하면 나중에 전부 고생한다

**마커는 4개 이상, 비대칭으로 붙인다.**

- 3개면 자세(yaw)가 불안정하고, 가려지면 바로 추적이 끊긴다
- **대칭으로 붙이면 Motive 가 앞뒤를 헷갈린다.** 차가 가만히 있는데 yaw 가
  180° 뒤집히는 사고가 나고, 그러면 MPC 가 반대로 조향한다
- 높이를 서로 다르게 두면(하나는 높은 기둥에) 구분이 쉬워진다
- 차체에 **단단히** 고정한다. 주행 중 흔들리면 그게 그대로 위치 노이즈다

> 여러 대를 쓸 거면 차마다 마커 배치를 다르게 한다. 같으면 Motive 가
> 서로 바꿔 인식한다.

#### Y자 지그를 쓰는 경우 — 대칭성에 주의

Y자 구조물(가운데 1 + 양쪽 갈래 2+2 + 아래 막대 3 = 8개)처럼 **좌우 대칭
형상**에 마커를 균등하게 붙이면, Motive 가 **좌우를 뒤집어 인식**할 수
있다. 그러면 yaw 가 180° 튀고 MPC 가 반대로 조향한다.

대칭을 깨는 방법:

- 한쪽 갈래의 마커 하나만 **간격을 다르게** 한다 (예: 4cm → 6cm)
- 마커 하나를 **높이를 다르게** 올린다 (스페이서/기둥)
- 아래 막대의 3개 중 하나를 **살짝 옆으로** 옮긴다

> Motive 의 RigidBody Properties 에서 마커 간 거리들이 서로 충분히
> 다른지 확인할 수 있다. 몇 mm 차이로는 부족하고 **1cm 이상** 권장.

붙인 뒤 반드시 확인한다 — 차를 **제자리에서 천천히 한 바퀴** 돌리며:

```bash
ros2 topic echo /car/pose --field pose.orientation
```

중간에 값이 **불연속으로 튀면** 좌우 혼동이다. 마커 배치를 더 비대칭으로
바꿔야 한다.

### RigidBody 정의

1. Motive 에서 차량 마커들을 **모두 선택**
2. 우클릭 → **Rigid Body → Create From Selected Markers**
3. 이름을 정한다 — 예: **`car`**
   - ★ 이 이름이 **그대로 토픽 이름이 된다** (`/car/pose`)
   - 공백·한글 없이 영문 소문자를 권장

### ★ 원점과 축 맞추기 — 안 하면 나중에 보정해야 한다

MPC 의 운동학 모델은 **뒤축 중심** 기준이고, x축이 전진 방향이다.

1. 차를 트랙 위에 **똑바로(전진 방향이 +x)** 놓는다
2. RigidBody 선택 → Properties → **Pivot** 을 뒤축 중심으로 옮긴다
3. **Orientation → Reset to Current** 로 현재 자세를 기준(0°)으로 잡는다

여기서 못 맞췄으면 `config/mocap.yaml` 의 `offset_x` / `offset_y` /
`yaw_offset` 으로 보정할 수 있다 (6단계).

---

## 3단계 — Motive 스트리밍 설정

**View → Data Streaming** (또는 Settings → Streaming):

| 항목 | 값 |
|---|---|
| **Enable / Broadcast Frame Data** | ✅ 체크 |
| **Local Interface** | 젯슨과 **같은 대역의** IP 선택 ← 여러 개면 주의 |
| **Transmission Type** | ★ **Unicast** (위 설명 참고) |
| **Rigid Bodies** | ✅ 체크 |
| Markers / Labeled Markers | 필요 없으면 끈다 (무선 대역 절약) |
| Up Axis | **Z-Up** 권장 (아래 참고) |

> **Local Interface 를 잘못 고르는 실수가 잦다.** Motive PC 에 랜카드가
> 여러 개면 목록에 다 나오는데, 젯슨과 같은 대역이 아닌 것을 고르면
> 스트리밍은 "켜진 것처럼" 보이는데 젯슨엔 아무것도 안 온다.

> **Up Axis:** Motive 기본은 Y-Up 이고 ROS 는 Z-Up 이다. Motive 에서
> **Z-Up 으로 바꾸는 것**이 가장 깔끔하다. Y-Up 인 채로 두면 x/y/z 가
> 뒤섞여 들어와 위치가 이상해지는데, 원인을 찾기 번거롭다.

**Windows 방화벽**에서 Motive 를 허용했는지 확인한다. 막혀 있으면
데이터가 안 나간다.

---

## 4단계 — 젯슨에 natnet 드라이버 설치

```bash
cd ~/f1tenth_ws/src
git clone https://github.com/L2S-lab/natnet_ros2.git
cd ~/f1tenth_ws
rosdep install --from-paths src/natnet_ros2 -y --ignore-src
colcon build --packages-select natnet_ros2 --symlink-install
source install/setup.bash
```

실행 — **1단계에서 메모한 IP 를 넣는다**:

```bash
ros2 launch natnet_ros2 natnet_ros2.launch.py \
    serverIP:=<Motive PC IP> \
    clientIP:=<젯슨 IP> \
    serverType:=unicast
```

> 인자 이름이 `serverIP` / `clientIP` / `serverType` 다 (server_address 아님).

---

## 5단계 — 데이터가 오는지 확인

다른 터미널에서:

```bash
source ~/f1tenth_ws/install/setup.bash

ros2 topic list | grep -i pose        # /car/pose 같은 게 보여야 한다
ros2 topic hz /car/pose               # 100Hz 근처
ros2 topic echo /car/pose --once      # 좌표가 말이 되는지
```

### 잘 나오는가?

차를 **손으로 1m 옮겨보고** 값이 그만큼 변하는지 본다. 단위는 **미터**여야
한다 (밀리미터로 나오면 Motive 단위 설정 문제).

### 안 나올 때

| 증상 | 원인 |
|---|---|
| 토픽 자체가 없음 | 드라이버가 연결 실패 — IP·방화벽 확인 |
| 토픽은 있는데 `hz` 가 0 | ★ **Multicast 로 되어 있다** → Unicast 로 |
| 값이 계속 0 또는 NaN | Motive 에서 차가 추적 안 됨 (마커 가림) |
| 갑자기 끊김 | 무선 문제 또는 마커 가림 |

`hz` 가 0 인 경우가 가장 흔하고, 대부분 **Transmission Type** 이 범인이다.

---

## 6단계 — mocap_bridge 연결

`/car/pose` 를 `mocap_bridge` 가 `/mpc/state` (속도 포함) 로 바꿔준다.

`~/mlcs_mpc/src/mlcs_mpc/config/mocap.yaml` 에서 **토픽 이름을 실제와 맞춘다**:

```yaml
mocap_topic: '/car/pose'      # ← 5단계에서 본 실제 토픽 이름
```

> 기본값이 `/natnet_ros/car/pose` 로 되어 있는데, `natnet_ros2` 는
> `/<RigidBody이름>/pose` 로 낸다. **거의 확실히 고쳐야 한다.**

실행:

```bash
ros2 run mlcs_mpc mocap_bridge --ros-args \
    --params-file ~/mlcs_mpc/src/mlcs_mpc/config/mocap.yaml
```

확인:

```bash
ros2 topic hz /mpc/state                      # 100Hz 근처
ros2 topic echo /mpc/state --field pose.pose.position
ros2 topic echo /mocap/valid                  # true 가 계속 나와야 한다
```

### 속도 추정이 되는지 — 이게 핵심이다

`mocap_bridge` 의 존재 이유는 **속도를 만들어내는 것**이다 (mocap 은 위치만
준다). 차를 손으로 밀면서:

```bash
ros2 topic echo /mpc/state --field twist.twist.linear.x
```

- 밀 때 **양수**, 뒤로 당기면 **음수**
- 세워두면 **0 근처** (±0.02 이내)

정지 상태에서 값이 크게 튀면 칼만 필터 설정(`q_vel`, `r_meas`)을 손봐야
한다. 하지만 먼저 마커가 흔들리지 않는지부터 확인하라.

---

## 7단계 — ★ 마커 오프셋 보정 (지그를 차량 앞에 붙였다면 필수)

마커 구조물을 **차량 형상과 무관한 자리**(예: 앞바퀴 앞의 Y자 지그)에
붙였다면 이 단계를 반드시 해야 한다.

### 무엇이 문제이고 무엇이 문제가 아닌가

| | 영향 | 왜 |
|---|---|---|
| **yaw** | ✅ **문제없음** | 강체의 각도·각속도는 기준점과 **무관**하다 (강체운동의 성질). 지그를 어디에 붙이든 차가 30° 돌면 지그도 30° 돈다 |
| **yaw_rate** | ✅ 문제없음 | 위와 같은 이유 |
| **위치 x, y** | ❌ **보정 필요** | 마커 원점이 곧 보고 위치인데, 그건 차량 기준점(뒤축 중심)이 아니다 |
| **속도** | ❌ 보정 필요 | 위치를 미분해서 얻으므로 위치가 틀리면 같이 틀린다 |

즉 **조치가 필요한 것은 위치뿐**이고, yaw 는 "0도가 어느 방향이냐" 만
맞추면 된다.

### 보정 안 하면 얼마나 틀리는가 (계산)

마커가 뒤축 중심보다 **30cm 앞**에 있고 반경 1m 로 선회할 때:

```
뒤축 궤적 반경 : 1.000 m
마커 궤적 반경 : 1.044 m      ← 4.4% 큰 원을 그린다
```

- 직진 중: 30cm 앞으로 밀린 위치 (상수 오프셋)
- 선회 중: **그 오프셋의 방향이 계속 바뀐다** — 상수로 못 없앤다
- MPC 경로추종 목표가 보통 5cm 인데 **30cm** 가 틀어진다

### 해결 — `mocap_bridge` 가 이미 보정한다

```
model_x = raw_x + cos(yaw)·offset_x − sin(yaw)·offset_y
model_y = raw_y + sin(yaw)·offset_x + cos(yaw)·offset_y
```

`offset_x/offset_y` 는 **마커 원점에서 뒤축 중심으로 가는 벡터**를
차체 좌표계(x=전방, y=좌)로 표현한 값이다. 마커가 30cm 앞이면
`offset_x: -0.30` 이다 (뒤로 가야 하므로 음수).

> 검증: 이 식으로 복원한 위치의 오차는 1.6e-16 m — 수치오차 수준이다.
> **보정만 제대로 하면 지그를 어디에 붙이든 상관없다.**

### 재는 방법 ① — 줄자 (가장 빠르고 충분하다)

Motive 에서 RigidBody 의 **Pivot 이 어디인지** 확인하고, 그 점과
**뒤축 중심** 사이 거리를 잰다.

```yaml
offset_x: -0.30     # 마커가 뒤축보다 30cm 앞 → 뒤로 30cm
offset_y:  0.0      # 좌우 대칭으로 붙였으면 0
```

> Y자 지그를 좌우 대칭으로 붙였다면 `offset_y` 는 0 이다. 한쪽으로
> 치우쳤으면 그만큼 적는다 (왼쪽이 +).

### 재는 방법 ② — 제자리 회전 (교차 검증용)

줄자 값이 맞는지 확인하거나, Pivot 위치가 애매할 때 쓴다.

```bash
./drive.sh mocapcal spin
```

조향을 **한쪽 끝까지** 주고 **한 바퀴 이상** 천천히 돈다. 마커 궤적이
그리는 원의 중심이 회전 중심이고, 거기서 마커까지의 벡터로 오프셋을 낸다.

> 합성 데이터 검증에서 **0.1~0.2mm** 정확도로 복원된다.
> ⚠ 반 바퀴만 돌면 원 피팅이 부정확하다. 노드가 총 회전각을 알려주니
> 확인할 것.

### yaw_offset 재기

```bash
./drive.sh mocapcal straight
```

차를 **앞으로 곧게 2m 이상** 민다. 진행 방향과 보고된 yaw 의 차이가
`yaw_offset` 이다.

> 값이 크게 나오면(>2°) `config/mocap.yaml` 에 적는 것보다 **Motive 에서
> RigidBody Orientation 을 "Reset to Current" 로 맞추는 편이 깔끔하다.**
> 차를 전진 방향으로 똑바로 놓고 리셋하면 된다.

### 확인

보정을 넣은 뒤 `mocap_bridge` 를 다시 띄우고, **차를 제자리에서 회전**시켜
본다:

```bash
ros2 topic echo /mpc/state --field pose.pose.position
```

- 보정 전: x, y 가 크게 원을 그린다 (반경 = 오프셋 크기)
- 보정 후: **거의 제자리** (뒤축 중심이 회전 중심 근처이므로)

이게 오프셋이 맞는지 보는 가장 확실한 눈으로 보는 검사다.

---

## 8단계 — 실험 공간 경계 재기

`safety_node` 가 쓸 경계값이다. **기본값은 예시이므로 반드시 실측해야 한다.**

차를 실험 공간 네 귀퉁이에 놓고 각각:

```bash
ros2 topic echo /mpc/state --field pose.pose.position --once
```

나온 값보다 **안쪽으로** `config/safety.yaml` 을 채운다:

```yaml
x_min: -3.0
x_max:  3.0
y_min: -3.0
y_max:  3.0
margin: 0.3
```

---

## 완료 확인

이 셋이 다 되면 다음 단계(캘리브레이션)로 갈 수 있다:

```bash
ros2 topic hz /mpc/state        # 100Hz 근처, 끊김 없음
ros2 topic echo /mocap/valid    # true 유지
```

- [ ] 차를 옮기면 위치가 실제와 맞게 변한다 (단위: m)
- [ ] 밀면 속도가 양수, 세우면 0
- [ ] 트랙 전 구간에서 추적이 안 끊긴다 ← **주행 전에 꼭 확인**

마지막 항목이 중요하다. 트랙 한쪽 구석에서만 마커가 가리면, 그 지점을
지날 때마다 차가 멈춘다.

---

## 다음

→ [CALIBRATION.md](CALIBRATION.md) — mocap 을 자로 삼아 VESC 값 측정
