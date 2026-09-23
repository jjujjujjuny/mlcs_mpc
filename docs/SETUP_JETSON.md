# Jetson Orin Nano 환경 구축 (Ubuntu 22.04 Jammy)

아무것도 설치되지 않은 젯슨을 MPC 주행 가능한 상태로 만드는 절차다.
**위에서부터 순서대로** 진행한다 — 뒤 단계가 앞 단계에 의존한다.

> 이 문서는 실제로 젯슨에서 실행해 보며 검증해야 한다. 지금은 개발 PC
> (Ubuntu 20.04 / ROS 1 Noetic) 에서 작성되었고, 알고리즘 부분만 시뮬로
> 검증된 상태다. 명령 하나하나가 예상대로 되는지 확인하며 진행하고,
> 다르면 이 문서를 고쳐 두세요.

---

## 0. 왜 Humble 인가

Ubuntu 22.04 (Jammy) 의 ROS 2 기본 배포판은 **Humble** 이다.
받아 둔 `f1tenth_system-foxy-devel` 은 **Foxy(20.04)용** 이라 그대로 쓰면 안 된다.
아래 3단계에서 `humble-devel` 브랜치를 새로 받는다.

| | 이 저장소 | 받아둔 참고본 |
|---|---|---|
| f1tenth_system | `humble-devel` 브랜치를 새로 clone | `foxy-devel` (20.04용, 참고만) |
| f1tenth_gym_ros | `main` 그대로 사용 가능 | 받아둔 것 사용 |

> ⚠ 브랜치 이름은 **`humble-devel`** 이다. `humble` 이라는 브랜치는 **없다**
> (2026-09 확인). 실제 브랜치 목록: `foxy-devel`, `humble-devel`,
> `jazzy-devel`, `melodic`, `braking`.

---

## 1. ROS 2 Humble 설치

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
```

확인:

```bash
source /opt/ros/humble/setup.bash
ros2 --version        # humble 이 나와야 한다
```

`~/.bashrc` 에 추가:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

> ⚠ `ROS_DOMAIN_ID` 는 `.bashrc` 에 박지 마세요. 아래 6절에서 설명하듯
> 모션캡처 PC 와 값을 맞춰야 하는데, 박아두면 나중에 왜 안 보이는지
> 찾기 어려워집니다.

---

## 2. 의존 패키지

```bash
sudo apt install -y \
    ros-humble-ackermann-msgs \
    ros-humble-joy \
    ros-humble-asio-cmake-module \
    ros-humble-io-context \
    ros-humble-serial-driver \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf-transformations \
    ros-humble-xacro \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    python3-colcon-common-extensions \
    python3-transforms3d \
    python3-scipy \
    python3-matplotlib \
    git
```

> **numpy 는 1.26.x 로 고정한다.** 2.x 로 올라가면 cv2/matplotlib/colcon 이
> 깨진다 — 26-jetson 저장소가 같은 문제를 겪고 문서에 남겨둔 사항이다.
>
> ```bash
> pip3 install "numpy==1.26.4"
> python3 -c "import numpy; print(numpy.__version__)"
> ```

---

## 3. 워크스페이스 + f1tenth_system (humble-devel)

```bash
mkdir -p ~/f1tenth_ws/src && cd ~/f1tenth_ws/src

git clone -b humble-devel https://github.com/f1tenth/f1tenth_system.git
cd f1tenth_system

# ★ --remote 가 중요하다 (아래 설명)
git submodule update --init --recursive --remote
```

> 받아 둔 `f1tenth_system-foxy-devel` 의 서브모듈 폴더(`vesc/`,
> `ackermann_mux/`, `teleop_tools/`)는 **비어 있다.** zip 다운로드는
> 서브모듈을 포함하지 않기 때문이다. 그래서 반드시 서브모듈을 따로 받아야 한다.

**왜 `--remote` 인가** — `humble-devel` 의 `.gitmodules` 는 서브모듈마다
가리키는 브랜치가 다르다:

| 서브모듈 | 브랜치 |
|---|---|
| `vesc` | `ros2` |
| `teleop_tools` | `humble-devel` |
| `ackermann_mux` | **지정 없음** → 기본 브랜치 `foxy-devel` |

`--remote` 없이 받으면 상위 저장소에 고정(pin)된 커밋을 쓰는데, 공식 README 가
`--remote` 를 안내하므로 그쪽을 따른다.

> `ackermann_mux` 에 humble 브랜치가 없는 것은 **정상이다.** 단순한 메시지
> mux 라 distro 간 차이가 없어 `foxy-devel` 을 그대로 쓴다. 브랜치 이름만
> 보고 "잘못 받았나" 의심할 필요 없다.

확인:

```bash
ls vesc ackermann_mux teleop_tools    # 셋 다 비어 있지 않아야 한다
```

이 저장소도 워크스페이스에 넣는다:

```bash
cd ~
git clone git@github.com:jjujjujjuny/mlcs_mpc.git
ln -sfn ~/mlcs_mpc/src/mlcs_mpc ~/f1tenth_ws/src/mlcs_mpc
```

> 복사가 아니라 **심링크**다. repo 에서 고치면 바로 반영되고, 워크스페이스
> 쪽에서 고쳐도 그게 곧 repo 라 커밋으로 보존된다. 26-jetson 저장소가
> 쓰는 것과 같은 방식이다.

### ★ 라이다가 없다 — bringup 에서 라이다를 빼야 한다

`humble-devel` 의 `f1tenth_stack` 은 **라이다를 전제로** 만들어져 있다:

- `package.xml` 이 `urg_node` 와 `sick_scan_xd` 를 `<depend>` 로 건다
- `bringup_launch.py` 가 `urg_node` 를 **조건 없이** 띄운다

우리 차에는 라이다가 없으므로 그대로 두면 매번 라이다 노드가 연결 실패
에러를 뱉는다. 또 `sick_scan_xd` 는 apt 에 arm64 바이너리가 없을 수 있어
`rosdep install` 이 거기서 멈출 수 있다.

**해결 — `urg_node` 를 런치에서 뺀다:**

```bash
cd ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/launch
cp bringup_launch.py bringup_launch.py.orig
```

`bringup_launch.py` 에서 아래 줄을 찾아 주석 처리한다:

```python
# ld.add_action(urg_node)      # ← 라이다 없음. mocap 으로 위치추정한다
```

> `urg_node = Node(...)` 정의 자체는 남겨둬도 된다 — `add_action` 만 안 하면
> 노드가 안 뜬다. 나중에 라이다를 달면 주석만 풀면 된다.

`package.xml` 의 의존성은 아래 `--skip-keys` 로 우회한다.

빌드:

```bash
cd ~/f1tenth_ws

# ★ 없는 라이다 패키지는 건너뛴다
rosdep install --from-paths src -y --ignore-src \
    --skip-keys "sick_scan_xd urg_node"

colcon build --symlink-install
source install/setup.bash
```

> 처음이면 `sudo rosdep init && rosdep update` 를 먼저 한 번 실행한다.

#### 빌드 실패 — `asio_cmake_module` 을 못 찾는 경우

```
CMake Error ... By not providing "Findasio_cmake_module.cmake" ...
Failed   <<< vesc_driver
Aborted  <<< vesc_ackermann  ackermann_mux
```

`vesc_driver` → `serial_driver` → `io_context` → `asio_cmake_module` 로
이어지는 의존성인데, `--skip-keys` 를 쓰면 rosdep 이 여기까지 못 채우는
경우가 있다. 2절에 이미 넣어 뒀지만 빠졌다면:

```bash
sudo apt install -y ros-humble-asio-cmake-module ros-humble-io-context \
                    ros-humble-serial-driver libasio-dev

# ★ 실패한 빌드 잔여물을 지워야 한다 — 안 지우면 실패 상태가 캐시된다
cd ~/f1tenth_ws
rm -rf build/vesc_driver build/vesc_ackermann build/ackermann_mux
colcon build --symlink-install
```

> `vesc_driver` 가 죽으면 그 뒤 패키지가 통째로 중단되므로
> (`2 packages not processed`), 고치고 다시 빌드하면 나머지가 따라온다.

라이다를 뺐으므로 bringup 후 토픽이 이렇게 나와야 한다 (`/scan` **없음**):

```bash
ros2 topic list
#  /ackermann_cmd  /drive  /odom  /sensors/core  /sensors/imu  /teleop ...
#  ← /scan 이 없어야 정상이다
```

`--symlink-install` 이면 파이썬 코드 수정은 재빌드 없이 반영된다.
재빌드가 필요한 경우: 새 `.py`/launch 파일 추가, `setup.py`/`package.xml` 변경.

---

## 4. VESC 장치 권한 (udev)

```bash
ls -l /dev/ttyACM*        # VESC 연결 확인
sudo usermod -aG dialout $USER    # 재로그인 필요
```

포트 이름이 재부팅마다 바뀌는 것을 막으려면 udev 규칙을 만든다:

```bash
udevadm info -a -n /dev/ttyACM0 | grep -m1 serial   # 시리얼 번호 확인
```

```bash
sudo tee /etc/udev/rules.d/99-vesc.rules > /dev/null <<'EOF'
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="sensors/vesc"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/sensors/vesc
```

`f1tenth_stack/config/vesc.yaml` 의 `port` 가 `/dev/sensors/vesc` 를 가리키게 한다.

---

## 4-2. 조이스틱 (로지텍 F710)

### 연결 확인

수신기를 USB 에 꽂고:

```bash
ls /dev/input/js*              # js0 가 보여야 한다
sudo apt install -y joystick   # jstest 용
jstest /dev/input/js0          # 버튼·스틱을 움직여 값이 변하는지 확인
```

권한이 없으면:

```bash
sudo usermod -aG input $USER   # 재로그인 필요
```

### ★ 스위치 두 개를 확인하세요

| 위치 | 설정 | 왜 |
|---|---|---|
| 앞면 | **X** (D 아님) | D 모드는 축 배치가 완전히 다르다 |
| 뒷면 Mode | **OFF** (LED 꺼짐) | ON 이면 D-pad 와 L스틱이 서로 바뀐다 |

### 축·버튼 번호 확인

`config/joystick.yaml` 의 기본값은 F710 표준 배치지만, 드라이버 버전에 따라
다를 수 있다. 실제 값을 보려면:

```bash
ros2 run joy joy_node            # 터미널 1
ros2 topic echo /joy             # 터미널 2
```

- 버튼을 하나씩 누르며 `buttons` 배열의 몇 번째가 `1` 이 되는지
- 스틱·트리거를 움직이며 `axes` 배열의 몇 번째가 변하는지

다르면 `config/joystick.yaml` 을 고친다.

### 실행

```bash
# bringup 을 이미 띄웠으면 joy:=false  ← ★ 중요
ros2 launch mlcs_mpc joystick.launch.py joy:=false
```

> ⚠ `f1tenth_stack` 의 `bringup_launch.py` 가 **joy_node 를 이미 띄운다.**
> 여기서 또 띄우면 `/joy` 퍼블리셔가 둘이 되어 deadzone 설정이 뒤섞이고,
> 어느 쪽 메시지가 오는지에 따라 조종감이 달라진다.

### 첫 조종 — 순서를 지킬 것

1. **차를 거치대에 올리고** (바퀴가 땅에 안 닿게)
2. 실행하고 **RT 에서 손을 뗀 채** 로그에 `트리거 캘리 완료` 가 뜰 때까지 대기
3. RT 를 살짝 당겨 바퀴가 **전진** 방향으로 도는지 확인
   → 반대면 `config/joystick.yaml` 의 `speed_invert: true`
4. **전진이 맞는 것을 확인한 뒤에** L스틱을 좌로 밀어 앞바퀴가 좌로 도는지 확인
   → 반대면 `steer_invert: true`

> ★ **3번을 건너뛰고 4번을 판단하지 마세요.** 후진 중에는 스틱을 좌로 밀어도
> 앞머리가 우로 도는 것처럼 보입니다(후진 주차와 같은 감각). 26-jetson 2호차가
> 이 함정에 빠져 `steer_invert` 를 거꾸로 잡았고, 나중에 속도 부호를 고치자
> 이중 반전이 되어 좌우가 또 바뀌었습니다.

### 조이스틱을 뽑아보세요 (워치독 확인)

거치대 위에서 RT 를 당긴 채 **수신기를 뽑습니다.** 0.3초 안에 멈추고
로그에 `/joy 가 ...s 끊겼습니다 — ★ 속도 0 강제 발행` 이 떠야 합니다.

이게 동작하지 않으면 실주행을 하지 마세요.

---

## 5. 모션캡처 (OptiTrack NatNet) 연동

라이다가 없으므로 **위치추정 전체를 모션캡처가 담당한다.** 이 부분이
안 되면 아무것도 못 한다.

### 5-1. natnet 드라이버 설치

```bash
cd ~/f1tenth_ws/src
git clone https://github.com/L2S-lab/natnet_ros2.git
cd ~/f1tenth_ws && colcon build --packages-select natnet_ros2 --symlink-install
```

> 다른 구현체(`mocap4ros2_optitrack` 등)를 써도 된다. **PoseStamped 를
> 발행하기만 하면** `mocap_bridge` 는 동작한다. 토픽 이름만
> `config/mocap.yaml` 의 `mocap_topic` 에 맞추면 된다.

### 5-2. Motive PC 쪽 설정

- Motive → View → Data Streaming
- **Broadcast Frame Data** 체크
- Network Interface: 젯슨과 같은 서브넷의 주소
- Transmission Type: **Unicast** 권장 (무선에서 멀티캐스트는 자주 막힌다)
- 차량을 **RigidBody** 로 정의하고 이름을 적어둔다 (예: `car`)

> ★ RigidBody 의 원점과 축 방향을 차량 기준으로 맞춰 두면 나중이 편하다.
> 원점 = 뒤축 중심, x축 = 전진 방향. 안 맞으면 `config/mocap.yaml` 의
> `offset_x/offset_y/yaw_offset` 으로 보정한다.

### 5-3. 연결 확인

```bash
ros2 launch natnet_ros2 natnet_ros2.launch.py \
    server_address:=<Motive PC IP> client_address:=<젯슨 IP>

# 다른 터미널
ros2 topic list | grep natnet
ros2 topic hz /natnet_ros/car/pose     # 100Hz 근처가 나와야 한다
ros2 topic echo /natnet_ros/car/pose --once
```

토픽 이름을 `config/mocap.yaml` 의 `mocap_topic` 에 반영한다.

---

## 6. 네트워크 / DDS

Motive PC, 젯슨, (있다면) 노트북이 **같은 `ROS_DOMAIN_ID`** 를 써야 토픽이 보인다.

```bash
export ROS_DOMAIN_ID=42      # 모든 기기에서 같은 값
```

무선에서 DDS 가 불안정하면 (26-jetson 저장소가 실제로 겪은 문제):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

> ⚠ **와이파이를 바꾸면 스택을 재시작하세요.** CycloneDDS 는 기동 시점의
> IP 에 인터페이스를 고정하고, 그 뒤 IP 가 바뀌면 재바인딩하지 못한다.
> 노드는 살아 있는데 서로 못 듣는 상태가 되어 "서보 고장" 처럼 보인다.
> 26-jetson 저장소가 이 문제로 하루를 썼다.

---

## 7. 동작 확인 순서

**반드시 이 순서대로.** 각 단계가 통과해야 다음으로 간다.

### ① 시뮬에서 MPC 먼저 (차 없이)

```bash
# 터미널 1
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
# 터미널 2
ros2 launch mlcs_mpc sim_mpc.launch.py
```

RViz 에서 차가 트랙을 도는지 본다. 여기서 안 되면 실차로 가지 않는다.

### ② mocap 위치추정 확인 (차를 손으로 밀면서)

```bash
ros2 launch natnet_ros2 natnet_ros2.launch.py ...   # 터미널 1
ros2 run mlcs_mpc mocap_bridge --ros-args \
    --params-file ~/mlcs_mpc/src/mlcs_mpc/config/mocap.yaml   # 터미널 2

ros2 topic echo /mpc/state          # 터미널 3
```

차를 손으로 1m 밀고 `position.x/y` 가 실제와 맞는지, 속도가 튀지 않는지 본다.

### ③ 거치대 위 조향 확인 (바퀴가 땅에 안 닿게)

```bash
ros2 launch f1tenth_stack bringup_launch.py         # 터미널 1
ros2 launch mlcs_mpc car_mpc.launch.py \
    waypoints:=<경로.csv> bench:=true               # 터미널 2
ros2 topic pub --once /mpc/enabled std_msgs/msg/Bool "{data: true}"
```

`bench:=true` 는 속도를 항상 0 으로 묶는다. 조향만 움직이는지 확인한다.

### ④ 캘리브레이션

→ `docs/CALIBRATION.md`

### ④-2 RViz 로 주행 화면 보기

**젯슨에서** 실행하고 NoMachine 으로 봅니다:

```bash
MLCS_TRACK=<트랙폴더> ./drive.sh rviz
```

> ⚠ 메인 PC(20.04/ROS 1)에서는 안 됩니다 — rviz2 가 없고 ROS 1↔ROS 2 는
> 통신하지 않습니다. 젯슨 화면을 원격으로 가져오는 것이 가장 간단합니다.

`ros-humble-desktop` 을 설치했다면 rviz2 가 이미 있습니다. `ros-base` 만
깔았다면:

```bash
sudo apt install -y ros-humble-rviz2
```

NoMachine 이 느리면 RViz 좌측 Displays 에서 **Trail** 을 끄세요 (점이 계속
쌓입니다). 그래도 느리면 `track_viz` 의 `trail_len` 을 500 으로 줄입니다.

### ⑤ 저속 실주행

```bash
ros2 launch mlcs_mpc car_mpc.launch.py \
    waypoints:=<경로.csv> target_speed:=1.0
```

**킬스위치를 손에 들고** 시작한다.

---

## 8. acados (선택 — 나중에)

지금은 필요 없다. iLQR 백엔드가 의존성 없이 동작하고, Levine 트랙 시뮬에서
20Hz 를 여유 있게 만족한다 (평균 9.7ms).

동역학 모델 + 하드 제약이 필요해지면 그때 설치한다:

```bash
git clone https://github.com/acados/acados.git --recursive
cd acados && mkdir build && cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4
pip3 install -e ../interfaces/acados_template
export ACADOS_SOURCE_DIR=~/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/acados/lib
```

→ 로드맵은 `docs/ROADMAP.md`
