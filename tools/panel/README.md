# 노트북 제어판 — `./car.sh panel`

젯슨 차의 상태를 보고, 실행·정지·설정·맵 저장을 버튼으로 한다. 레포 맨 위에서 `./car.sh panel`.

```
┌ 상태 칩: SSH · 토픽 · 배터리 · 라이다 · 위치 · 속도 · 솔버 p95/최대 · hmpc ┐
├ 실행 (젯슨)                                   ├ 설정 (젯슨 race.yaml)          ┤
│  젯슨: bench 실행 중                           │  맵 / 트랙 / 최고 속도          │
│  ● 드라이버 ● 라이다 ● Cartographer ● hmpc …   │  장애물 / 기록 태그             │
│  [bench 시작] [주행 시작] [맵핑 시작]           │  [적용] [되돌리기]              │
│  [정지 (Ctrl+C)] [강제 종료] [출발 (B)]         │  [목록 새로고침]                │
│  [RViz 열기] [젯슨 터미널]                      │                                │
├ 맵 만들기: 이름 [맵 저장] [트랙 만들기]         │                                │
└ 로그 탭: 화면 · hmpc · 솔버 · Cartographer · 라이다 · 드라이버 · 브리지 · 위치 다시잡기 · 맵 저장·트랙 · 제어판 ┘
```

## 원칙 (2026-11-10 사용자 결정)

- **노트북 → 젯슨 실시간 제어는 없다.** 정지는 젯슨에 직결된 조이스틱(LB 수동 전환, START 비상 정지)이 한다. 노트북 연결이 끊겨도 주행은 계속된다.
- 노트북에서 젯슨으로 가는 것: 실행/정지/설정 명령(SSH), `출발`(= 조이스틱 B, `/mpc/go`), RViz `2D Pose Estimate`(`/initialpose`). 이게 전부다.
- **젯슨 연산량을 늘리면 안 된다.** 솔버 시간이 늘 바에는 안 하는 게 낫다.

## 구조

| 하는 일 | 통로 | 젯슨 쪽 비용 |
|---|---|---|
| 상태 칩 (배터리·라이다·위치·속도·솔버·hmpc) | Zenoh 브리지 토픽 (이미 쓰던 것 + `/sensors/core` 2 Hz) | 브리지 CPU ≈0 % |
| 실행 상태, 설정 읽기 | SSH `remote.sh status` 5 초마다 | 1 회 0.055 s CPU (코어 0-3, nice 19) → 코어 하나의 약 1 % |
| 로그 탭 | SSH `remote.sh logs` = `tail -F` 하나를 계속 | inotify 대기, 거의 0 |
| 화면 탭 | `tmux capture-pane`, **그 탭을 보고 있을 때만** 2 초마다 | 1 회 0.02 s |
| 실행/정지/설정/맵 저장 | SSH `remote.sh start/int/stop/set/savemap/maptrack` (누를 때만) | – |

- SSH 는 연결 하나를 열어 두고(ControlMaster) 그 위에서 명령을 보낸다. 매번 새로 접속하지 않는다.
- 조회 명령(status/lists/screen/logs)은 젯슨 코어 0-3·nice 19 로 돈다. 젯슨에 isolcpus 가 없어서 그냥 두면 솔버 코어 5 에 올라갈 수 있다.
- ★ **실행 명령(start)에는 nice·taskset 을 물려주지 않는다.** tmux 서버가 nice 19 로 뜨면 hmpc·솔버가 전부 최저 우선순위가 되고, root 없이는 되돌릴 수 없다. 제어판으로 시작한 race.sh 가 nice 0, 코어 0-5 로 도는 것을 확인했다.
- `status` 는 처음엔 0.28 s CPU 였다 (pgrep 7 번 = /proc 7 번). 프로세스 목록 한 번, yaml 은 awk 한 번, 서브셸 없이 → 0.055 s.
- SSH 비대화형 실행에서 젯슨 `~/.bashrc` 는 첫 줄(`case $- in *i*`)에서 돌아가므로 ROS source 비용이 없다.

## 파일

| 파일 | 어디서 |
|---|---|
| `car_panel.py` | 노트북. PyQt5 + rclpy. 노트북 Zenoh 브리지를 직접 띄운다 (이미 떠 있으면 그대로) |
| `remote.sh` | 젯슨. 제어판이 SSH 로 부르는 창구 |
| `msgs/vesc_msgs` | 노트북에서 배터리(`/sensors/core`)를 읽기 위한 메시지 정의. 젯슨 `f1tenth_ws` 에서 복사. `./car.sh panel` 이 처음 한 번 빌드 (젯슨에서 시험 15 초) |

## 노트북에서 처음 (2026-11-10 요청)

```bash
cd ~/mlcs_mpc && git pull
python3 -c 'import PyQt5' || sudo apt install -y python3-pyqt5
./car.sh panel
```

확인할 것:

1. 창이 뜨고 `SSH ✓` 칩이 초록인가 (`./car.sh key` 를 했어야 한다 — BatchMode 라 비밀번호를 묻지 않는다)
2. 설정 칸에 젯슨 race.yaml 값(맵 1109_4 등)이 채워지는가, 맵 목록이 나오는가
3. `bench 시작` → 상태가 "bench 실행 중", ● 드라이버·라이다·Cartographer·hmpc·브리지가 차례로 초록, `화면` 탭에 race.sh 출력
4. 칩: `토픽 ✓`, 배터리 %, 라이다 ≈10 Hz, 위치 좌표, 솔버 p95 (bench 에서도 풀이는 돈다)
5. `RViz 열기` → 2D Pose Estimate → `위치 다시잡기` 탭에 로그
6. `정지 (Ctrl+C)` → 상태가 "끝남", 다시 `bench 시작` 이 되는가
7. 설정에서 최고 속도만 바꿔 `적용` → 젯슨 race.yaml 이 바뀌는가 (확인 후 되돌리기)
8. 제어판을 닫아도 젯슨에서 돌던 것이 그대로인가 (`./car.sh term` 으로 확인)

결과와 에러 원문은 이 README 아래 "노트북 시험 결과" 절에 적어서 push.

## 젯슨에서 미리 확인한 것 (2026-11-10)

- `remote.sh`: status/lists/screen/set, start→int→ended→재시작→stop 흐름을 가짜 race.sh 로. `set` 은 형식 밖 값(`a;rm`)을 거부하고 race.yaml 의 주석을 보존한다
- 제어판 ROS 부분(Qt 없이): 가짜 토픽으로 라이다 10 Hz, 위치, 속도, 솔버 p95, 배터리 11.6 V, hmpc 상태 수신
- vesc_msgs 사본 빌드·import (colcon 15 초)
- **못 한 것: 화면(Qt) 자체.** 젯슨에 PyQt5 가 없다. 노트북이 첫 실행이다

## 알려진 한계

- `/hmpc/status` 는 바뀔 때만 발행된다. 제어판을 주행 도중에 열면 다음 변화까지 `hmpc —` 로 보인다
- 라이다 칩은 브리지가 10 Hz 로 솎은 뒤의 값이다 (젯슨 원래 40 Hz). 0 이면 끊긴 것
- 배터리 % 는 쉬고 있을 때 기준이라 주행 중엔 낮게 나온다
