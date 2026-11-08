# 외부 패키지 패치

`f1tenth_ws/src` 에 클론해 쓰는 서드파티 저장소에 우리가 가한 수정입니다.
`git pull` 하면 날아가므로 여기에 보관하고, 다시 클론할 때 적용합니다.

## natnet_ros2-bitstream-version.patch

**대상** [L2S-lab/natnet_ros2](https://github.com/L2S-lab/natnet_ros2) — 기준 커밋 `f379296`

**증상** — 실행하면 노드가 SIGABRT(exit -6) 로 즉사한다. 토픽이 하나도 안 뜨고
`/natnet_ros/transition_event` 만 남는다.

    ClientCore.cpp:710: ErrorCode ClientCore::ValidateHostConnection():
        Assertion `mServerDescription.HostPresent' failed.

**원인** — 우리 Motive 는 **2.3.0.1**, 즉 **NatNet 3.1** 로 말한다. 그런데 이
드라이버가 쓰는 NatNet SDK 는 **4.4** 이고, `BitstreamVersion` 을 비워두면
4.x 형식 응답을 기대한다. 3.1 서버의 짧은 응답을 파싱하지 못해
`HostPresent` 가 채워지지 않고 assert 로 죽는다.

Motive 쪽은 멀쩡했다. 같은 젯슨에서 SDK 동봉 파이썬 클라이언트
(`samples/PythonClient/PythonSample.py`) 로 확인했더니 스스로
`resetting requested version to 3 1 0 0` 하고 100Hz 로 프레임을 잘 받았다.
C++ SDK 만 버전 협상을 안 한다.

**수정** — `bitstream_version` 파라미터(기본 `"3.1.0.0"`)를 추가해
`sNatNetClientConnectParams::BitstreamVersion` 을 명시적으로 채운다.

    "3.1.0.0"   Motive 2.x   ← 현재 장비
    "4.0.0.0"   Motive 3.x
    ""          지정 안 함 (SDK 기본). Motive 2.x 에서는 위 assert 로 죽는다.

**Motive 를 3.x 로 올리면** 이 패치는 필요 없다. 다만 지워도 되고,
런치에 `bitstream_version:=4.0.0.0` 을 넘겨도 된다.

### 적용

    cd ~/f1tenth_ws/src
    git clone https://github.com/L2S-lab/natnet_ros2.git
    cd natnet_ros2
    git apply ~/mlcs_mpc/patches/natnet_ros2-bitstream-version.patch
    cd ~/f1tenth_ws
    colcon build --packages-select natnet_ros2 --symlink-install

`git apply` 가 거부하면 업스트림이 바뀐 것이다. 위 "원인" 을 읽고
`set_conn_params()` 에 직접 네 줄만 넣으면 된다.

### 실행

    ros2 launch natnet_ros2 natnet_ros2.launch.py \
        serverIP:=192.168.1.3 clientIP:=<젯슨 IP> serverType:=unicast \
        pub_rigid_body:=true activate:=true

★ `pub_rigid_body` 와 `activate` 는 **둘 다 기본값이 false** 다. 빼먹으면
연결은 되는데 토픽이 안 나온다 (라이프사이클 노드가 활성화되지 않는다).

토픽 이름은 Motive 의 강체 이름이 그대로 붙어 `/<강체이름>/pose` 가 된다.
ROS1 판(`natnet_ros_cpp`)의 `/natnet_ros/` 접두어는 **붙지 않는다.**


## f1tenth_system-no-joy-nodes.patch

**대상** [f1tenth/f1tenth_system](https://github.com/f1tenth/f1tenth_system) — 브랜치 `humble-devel`

remote 가 **우리 fork 가 아니라 upstream 원본**이다. push 할 수 없고,
`git pull` 하면 이 수정이 날아간다.

### ① 조이스틱 두 노드를 bringup 에서 뺀다

`bringup_launch.py` 는 `joy_node` 와 f1tenth 자체 `joy_teleop` 을 같이 띄운다.
그런데 그 `joy_teleop` 은 우리 `joystick_teleop` 과 **같은 `/teleop` 토픽**에
발행하고, 설정이 이렇다 (`f1tenth_stack/config/joy_teleop.yaml`):

    human_control:
      deadman_buttons: [4]              ← LB. 우리가 모드 토글로 쓰는 버튼
      drive-speed: {axis: 1, scale: 5.0}

즉 LB 를 누르는 순간 두 노드가 동시에 반응하고, 저쪽은 X 모드에서
`axis 2`(= LT 트리거)를 조향으로 읽으면서 **5 m/s** 를 명령한다.
`/teleop` 퍼블리셔가 둘이 되는 것 자체가 위험하다.

조이스틱은 `mlcs_mpc/joystick.launch.py` 가 띄운다. 터미널 두 개로:

    ros2 launch f1tenth_stack bringup_launch.py     # 터미널 A
    ros2 launch mlcs_mpc joystick.launch.py         # 터미널 B

### ② urg_node 를 뺀다

라이다가 없다. 위치추정은 mocap 으로 한다.

## vesc-stop-brake.patch + f1tenth_system-stop-brake.patch

**대상** `f1tenth_system/vesc` (vesc_ackermann) 와 `f1tenth_stack/config/vesc.yaml`.
yaml 패치는 servo-offset 패치를 적용한 뒤에 들어간다 (apply.sh 순서).

**증상** (2026-11-08) — VESC 속도 PID 제동(`s_pid_allow_braking`)을 켠 뒤, 멈출 때
"감속 → 앞으로 한 번 튐 → 감속". 그 전에는 정지하면 뒤로 굴러가기도 했다.

**원인** — 센서리스 모터라 0 근처에서 VESC 가 속도를 못 본다. 속도 0 을 목표로 잡으면
목표 램프(`s_pid_ramp_erpms_s`)와 강한 제동, 개루프 전환이 얽혀 PID 가 다시 밀거나
제동 토크가 후진 토크가 된다.

**수정** — `ackermann_to_vesc` 가 정지 명령(`|speed| < brake_speed_threshold`, 0.05 m/s)을 받으면
`sensors/core` 의 실제 바퀴 속도를 본다.

- 바퀴가 `brake_handoff_speed` (0.5 m/s) 보다 빠르면 지금처럼 속도 0 → VESC PID 의 강한 반대 전류 제동
- 그 아래로 내려오면 `commands/motor/brake` 로 `brake_current` (A). 출발 명령이 올 때까지 유지
- VESC 상태가 0.2 s 넘게 안 오면 바로 제동 전류 (안전한 쪽)

제동 전류 모드는 세우기만 하고 구동하지 않는다. VESC 설정은 건드리지 않는다.
처음에는 항상 제동 전류였는데 "트리거를 놓으면 딱 잡던" 강한 제동이 없어져서 나눴다.

    brake_current: 10.0          # 0 이면 끔 (원래 동작)
    brake_speed_threshold: 0.05
    brake_handoff_speed: 0.5     # 0 이면 처음부터 제동 전류

조이스틱 정지·워치독 0, hmpc 정지·차단 모두 이 경로로 선다. 0 이 아닌 감속 명령은 그대로 속도 PID.
