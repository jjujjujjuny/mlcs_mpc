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
아래 3단계에서 humble 브랜치를 새로 받는다.

| | 이 저장소 | 받아둔 참고본 |
|---|---|---|
| f1tenth_system | `humble` 브랜치를 새로 clone | `foxy-devel` (20.04용, 참고만) |
| f1tenth_gym_ros | `main` 그대로 사용 가능 | 받아둔 것 사용 |

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

## 3. 워크스페이스 + f1tenth_system (humble)

```bash
mkdir -p ~/f1tenth_ws/src && cd ~/f1tenth_ws/src

# ★ humble 브랜치 + 서브모듈(vesc, ackermann_mux, teleop_tools)까지
git clone -b humble --recurse-submodules \
    https://github.com/f1tenth/f1tenth_system.git
```

> 받아 둔 `f1tenth_system-foxy-devel` 의 서브모듈 폴더(`vesc/`,
> `ackermann_mux/`, `teleop_tools/`)는 **비어 있다.** zip 다운로드는
> 서브모듈을 포함하지 않기 때문이다. 그래서 반드시 `--recurse-submodules`
> 로 새로 clone 해야 한다.

이 저장소도 워크스페이스에 넣는다:

```bash
cd ~
git clone git@github.com:jjujjujjuny/mlcs_mpc.git
ln -sfn ~/mlcs_mpc/src/mlcs_mpc ~/f1tenth_ws/src/mlcs_mpc
```

> 복사가 아니라 **심링크**다. repo 에서 고치면 바로 반영되고, 워크스페이스
> 쪽에서 고쳐도 그게 곧 repo 라 커밋으로 보존된다. 26-jetson 저장소가
> 쓰는 것과 같은 방식이다.

빌드:

```bash
cd ~/f1tenth_ws
rosdep install --from-paths src -y --ignore-src   # 처음 한 번은 rosdep init/update 필요
colcon build --symlink-install
source install/setup.bash
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
