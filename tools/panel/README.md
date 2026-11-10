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

## 노트북 시험 결과 (2026-11-10 13:04~13:07, hj-gram)

- 실행 ✓: `./car.sh panel` → vesc_msgs 자동 빌드 후 창 뜸 (PyQt5 는 이미 있었음). 시작 출력은 Qt 의 wayland 경고 한 줄뿐
- 1·2·3·4·6·7: 사용자 보고로 됨
- 5 ✗ **2D Pose Estimate — RViz 에 찍어도 변화 없음.** 전달은 된다: 노트북 RViz `/initialpose` → 노트북 브리지 → 젯슨 브리지 → `/relocalize` 구독 (양쪽 브리지 로그 확인). 젯슨이 맞춤 점수 미달로 5번 모두 거부:
  ```
  [relocalize]: RViz — 맞춤 3.1 s, 점수 0.01 → 0.24, 이동 72.5 cm, yaw -17.2° — 점수가 0.35 미만이라 적용하지 않습니다. 다시 찍으세요.
  ```
  (나머지 4번: 0.00→0.23, 0.07→0.33, 0.00→0.19, 0.00→0.24. 로그 `~/mlcs_data/logs/20261110_130537_relocalize.log`)
  맞춤 전 점수가 0.00~0.07 이라 찍은 자리가 많이 틀렸거나, 차가 맵 1109_4 위에 있지 않았을 수 있다. **거부됐다는 것이 노트북에 안 보인다** — RViz 도, 사용자도 "아무 변화 없음" 으로만 봤다. 제어판에 거부 사유(점수)를 띄워 주면 좋겠다.
- 8: 확인 못 함 — 제어판을 닫을 때 bench 는 이미 끝나 있었다
- 닫을 때 출력: `QProcess: Destroyed while process ("ssh") is still running.`
- bench 정지 뒤 race.sh 채점이 실패 (제어판·SSH 로 띄우면 젯슨에 DISPLAY 가 없다):
  ```
  ✗ DISPLAY 가 없습니다. 이 도구는 창을 띄웁니다 — NoMachine 으로 젯슨 화면에 접속
  한 뒤 실행하세요.
  채점을 못 했습니다 — 위 메시지를 보세요
  ```

## 젯슨 답 (2026-11-10) — 노트북 시험 결과 반영

1. **위치 다시잡기 결과를 제어판 위쪽 배너로** — `위치 다시잡기` 로그 탭의 relocalize 줄을 읽어서
   - 거부: 빨강 `✗ 13:06:17 위치 다시잡기 거부 — 맞춤 점수 0.01 → 0.24 (기준 0.35 이상), 찍은 곳에서 72.5 cm · -17.2° 떨어진 곳이 그나마 맞음 …` (찍은 자리 점수가 0.1 미만이면 "위치나 방향이 크게 틀린 것" 도 붙인다)
   - 성공: 초록 `✓ 위치 설정 (x, y, yaw) — 조이스틱 B 로 출발`
   - 시각은 로그의 ROS 시각이라, 재접속으로 옛 줄이 다시 와도 언제 일인지 맞게 보인다
   - 오늘 13:06 로그 5줄로 시험: 5번 모두 빨강 배너. 노트북만 바뀌고 젯슨 비용 없음
   - 이번 5번 거부 자체는 정상 동작으로 본다 — 찍은 자리 점수가 0.00~0.07 이면 그 자리·방향에 맵 벽이 거의 안 겹친 것. 차가 1109_4 맵 위 그 자리에 있었는지부터
2. **채점 실패 (DISPLAY 없음) 고침** — 원인은 race_report 가 아니라 `tools/draw_track.py`: import 되는 순간 DISPLAY 가 없으면 `sys.exit` 해서, 그걸 import 하는 도구가 전부 죽었다. `except Exception` 으로 안 잡힌다 (SystemExit).
   - **제어판 '트랙 만들기'(maptrack)도 같은 이유로 실패했을 것이다** — `track_from_map` 이 draw_track 을 import 한다. DISPLAY 없이 `./drive.sh maptrack 1109_4 zz_headless_test` 를 끝까지 돌려 확인함 (임시 트랙은 지움)
   - 이제 DISPLAY 가 없으면 Agg(그림 파일만). 창이 꼭 필요한 손그리기 편집기(`draw_track.py` 직접 실행)만 main() 에서 막는다. `test_draw_track.py`·`test_map_tools.py` DISPLAY 없이 통과
   - 참고: 채점(`racereport`)은 mocap 정답 기록이 있어야 한다. 대회장(mocap 없음)에서는 원래 못 한다
3. **닫을 때 `QProcess: Destroyed while process ("ssh") is still running.`** — 닫기 전에 돌고 있는 ssh(로그 tail 등)를 먼저 끝내고, 그 콜백(로그 재접속)이 돌지 않게 끊는다
4. 8번(닫아도 주행 유지)은 다음에 bench 를 켜 둔 채로 닫아서 확인 부탁

다시 확인할 것: `git pull` → `./car.sh panel` → RViz 에서 일부러 엉뚱한 곳을 찍어 빨강 배너, 제대로 찍어 초록 배너가 뜨는지. 닫을 때 경고가 없는지.

