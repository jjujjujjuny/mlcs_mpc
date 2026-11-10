# 새 노트북에서 젯슨 제어판 쓰기 — 설치 안내 (2026-11-10)

> **이 문서를 읽는 노트북의 Claude 에게**
> 이 레포(`mlcs_mpc`)는 F1TENTH 차(젯슨 Orin, 호스트명 `mlcs-car`)의 코드다. 지금 이 노트북은 레포를 clone 만 해 둔 상태다.
> 목표는 이 노트북에서 `./car.sh panel` 로 **젯슨 제어판**을 띄우는 것이다. 제어판은 차 상태 보기, 실행·정지, race.yaml 설정, 맵 저장, 로그를 한 창에서 한다.
> 아래 순서대로 하고, 각 단계 결과를 맨 아래 **'노트북 기록'** 절에 적어 push 해 달라. 젯슨 쪽 Claude 가 `git pull` 해서 읽고 답을 같은 절 아래에 적는다. 두 Claude 는 이렇게 레포로 대화한다.
> 대답은 한국어로.

## 지켜야 할 것

1. **실제 주행을 띄우지 않는다.** `./car.sh race`, 제어판의 `주행`, 젯슨의 `./race.sh` 는 사람이 차 옆에서 킬스위치를 들고 직접 한다. 시험은 **bench**(속도 0, 조향만)까지만, 그것도 사람이 "해도 된다" 고 한 뒤에.
2. **sudo 는 사람이 터미널에서 친다.** Claude 의 셸에는 비밀번호를 넣을 수 없다. `./car.sh setup` 이 멈추며 알려 주는 apt 명령을 사람에게 보여 주고 쳐 달라고 한다.
3. **비밀키(`~/.ssh/id_ed25519`)는 절대 커밋하지 않는다.** 올리는 것은 `.pub` 하나뿐.
4. **커밋은 내가 만든 파일만.** `git add -A` 금지. 이 문서와 `tools/laptops/keys/<내 키>.pub` 처럼 파일을 지정해서 add 한다. (젯슨에는 아직 커밋하지 않은 `race.yaml` 등이 있다. 노트북에서 그것들을 바꾸거나 올리지 말 것)
5. **젯슨 tmux 안에서 `./car.sh` 나 `tmux` 를 또 치지 않는다.** `./car.sh term` 으로 젯슨에 들어가 있으면 이미 tmux 안이다. 거기서 또 열면 겹쳐서 `Ctrl+b` 가 엉킨다 (2026-11-09 실제로 겪음).
6. 젯슨 부하를 늘리는 일(젯슨에서 파이썬·ros2 CLI 를 주기적으로 돌리기, 화면 공유 등)은 하지 않는다. 솔버 시간이 늘면 안 된다.

## 구조 (알고 시작하면 쉽다)

```
젯슨 노드들 ─(localhost DDS)─ 젯슨 Zenoh 브리지(7447) ══ 와이파이 ══ 노트북 브리지 ─ 노트북 RViz·제어판
                                    └── SSH 연결 하나 ── tools/panel/remote.sh (실행·정지·설정)
```

- **보기**는 Zenoh 브리지로 받은 토픽이다. `/scan`, `/tracked_pose`, `/mpc/*`, 배터리 등.
- **명령**은 SSH 위에서 `tools/panel/remote.sh` 를 부른다. 주행 중 노트북에서 젯슨으로 가는 것은 `출발`(조이스틱 B 와 같음)과 RViz `2D Pose Estimate` 뿐이다.
- **정지는 차에 붙은 조이스틱**으로 한다 (LB·START). 노트북이 끊겨도 주행은 계속된다 (일부러 그렇게 함).
- 젯슨과 노트북 모두 `ROS_LOCALHOST_ONLY=1` 이다. 밖으로 나가는 길은 브리지 하나뿐이다.
- **노트북 여러 대가 동시에 붙어도 된다.** 젯슨 브리지는 router 모드라 여러 노트북을 받는다. 다만 **실행·정지·설정은 한 사람만** 한다. 두 번째 실행은 젯슨이 "이미 실행 중" 으로 거부하지만, 설정을 둘이 동시에 바꾸는 것은 막지 않는다.

자세한 설계는 [../panel/README.md](../panel/README.md), 브리지는 [../zenoh/README.md](../zenoh/README.md) 에 있다. 이 문서만으로도 설치는 끝난다.

## 0. 확인 — 이 노트북이 되는가

```bash
lsb_release -rs        # 22.04 여야 한다 (젯슨·hj-gram 노트북 모두 22.04)
ls /opt/ros/humble     # ROS 2 Humble (ros-humble-desktop) — 없으면 먼저 설치
nproc                  # 브리지 빌드 시간 가늠 (8코어 ≈ 3분)
```

- 22.04 + Humble 이 아니면 **멈추고** 기록에 적는다. 메시지 형식이 젯슨과 달라지거나 브리지 빌드 방법이 달라진다.
- Humble 이 없으면 사람에게 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 의 `ros-humble-desktop` 설치를 부탁한다 (sudo, 시간이 걸림).

## 1. 노트북 준비 — `./car.sh setup`

```bash
cd <레포 위치>      # 보통 ~/mlcs_mpc
git pull
./car.sh setup
```

이미 된 단계는 건너뛰므로 여러 번 돌려도 된다. 하는 일은 [setup.sh](setup.sh) 맨 위 주석에 있다.

1. Ubuntu·ROS 를 확인한다.
2. apt 패키지 중 없는 것을 찾는다. **없으면 sudo 명령을 보여 주고 멈춘다.** 사람이 그 명령을 터미널에서 치고 나면 `./car.sh setup` 을 다시 돌린다.
3. `~/.bashrc` 에 `export ROS_LOCALHOST_ONLY=1` 을 넣는다. **새 터미널부터** 적용된다.
4. Rust 를 설치한다 (rustup, 인터넷 필요).
5. **Zenoh 브리지 1.10.2 를 소스에서 빌드**해 `tools/zenoh/x86_64/` 에 넣는다 (`~/zenoh-plugin-ros2dds` 에 소스를 받는다).
   - 미리 빌드된 x86_64 판은 22.04 에서 뜨지 않는다. gnu 판은 glibc 2.38 을 요구하고, musl 판은 musl 동적 링크다 (2026-11-09 실측). 그래서 직접 빌드한다.
   - 실행 파일은 레포에 넣지 않는다 (26MB, `.gitignore`).
   - 실패하면 마지막 30줄을 기록에 그대로 붙인다. 흔한 원인은 `stddef.h not found`(→ clang)와 libclang 못 찾음(→ libclang-dev).
6. SSH 키(`~/.ssh/id_ed25519`)가 없으면 만든다.

끝나면 `tools/zenoh/x86_64/zenoh-bridge-ros2dds --version` 이 `v1.10.2` 를 보여야 한다.

## 2. 젯슨 주소

```bash
./car.sh ip 192.168.1.7      # 연구실 공유기 mlcs-robotics5G 에서 젯슨 주소
```

- 노트북도 **같은 공유기(`mlcs-robotics5G`)** 에 붙어 있어야 한다. "닿습니다 ✓" 가 떠야 한다.
- 주소는 `tools/zenoh/.car_host` 에 저장된다 (이 노트북에만 있고 git 에는 안 올라감).

## 3. 젯슨 SSH 등록 — 둘 중 하나

젯슨 계정은 `mlcs` 이다. 제어판은 비밀번호를 묻지 않는 키 접속만 쓴다 (BatchMode).

**A. 젯슨 비밀번호를 알면** (사람에게 물어 직접 치게 한다):

```bash
./car.sh key          # 비밀번호 한 번 → "이제 비밀번호를 묻지 않습니다 ✓"
```

**B. 모르면** 공개키를 레포로 넘긴다:

```bash
./car.sh pubkey       # ~/.ssh/id_ed25519.pub → tools/laptops/keys/<사용자@호스트>.pub
git add tools/laptops/keys/<그 파일>.pub
git commit -m "노트북 키: <사용자@호스트>"
git push
```

그리고 기록 절에 "키 올림, 젯슨 등록 부탁" 을 적는다. 젯슨에서는 `git pull && tools/laptops/add_keys.sh` 로 등록한다. 등록됐는지는 아래로 확인한다.

```bash
ssh -o BatchMode=yes mlcs@192.168.1.7 'hostname; uptime'     # mlcs-car 가 나오면 됨
```

처음 접속하면 호스트 키를 믿을지 묻는다. 그때만 `yes`.

## 4. 제어판

```bash
./car.sh panel
```

- 처음 한 번은 배터리 메시지(`vesc_msgs`)를 colcon 으로 빌드한다 (약 30초).
- 젯슨 브리지는 젯슨에서 무언가 돌 때(bench·주행·맵핑)만 떠 있다. **아무것도 안 돌 때는 토픽 칩이 안 들어오는 게 정상**이다. SSH 칩과 설정 칸은 바로 채워진다.

**확인할 것** (bench 를 띄우는 4번은 사람이 "해도 된다" 고 할 때만):

1. 창이 뜨고 `SSH ✓` 칩이 초록인가
2. 설정 칸에 젯슨 race.yaml 값(맵·트랙·최고 속도)이 채워지고, 맵 목록이 나오는가
3. `명령어` 탭에 drive.sh·race.sh·bat.sh 사용법이 나오는가
4. (허락 후) `bench` → 상태 "bench 실행 중", 노드 점들이 차례로 초록, `토픽 ✓`, 라이다 ≈10 Hz, 배터리 % → `RViz` 버튼으로 맵·스캔·차 위치 → `정지 (Ctrl+C)` → 상태 "끝남"
5. 창을 닫았을 때 터미널에 Traceback·경고가 없는가

다른 노트북(hj-gram)에서 제어판이나 RViz 가 동시에 떠 있어도 된다. 둘 다 보이는지가 이번 시험의 핵심이니 같이 떠 있었는지도 적어 달라.

## 자주 쓰는 명령

`./car.sh` 만 치면 전체 목록이 나온다.

| 명령 | 하는 일 |
|---|---|
| `./car.sh panel` | 제어판 |
| `./car.sh rviz` | RViz 만 (브리지 포함) |
| `./car.sh check` | `/scan` `/tracked_pose` 주파수 (`rviz` 나 `panel` 이 떠 있을 때) |
| `./car.sh term` | 젯슨 tmux `race` 세션에 붙기. 빠질 때는 `Ctrl+b` 뒤 `d` (끄지 않음) |
| `./car.sh bat` | 배터리 |
| `./car.sh stop` | 젯슨 전부 정지·종료 |

## 남은 일 (두 노트북 공통)

- [ ] **대회 공유기 `yax_5G` 에서 아직 아무도 안 해 봤다.** 젯슨에 와이파이 프로필이 아직 없다 (비밀번호 필요). 대회장에서 젯슨을 `yax_5G` 에 붙인 뒤 각 노트북에서:
  `./car.sh ip <대회장 젯슨 IP>` → `./car.sh panel` → bench 로 확인.
  젯슨 IP 는 젯슨에서 `hostname -I` 로 보거나, `./drive.sh bridge` 가 화면에 보여준다.
- [ ] 젯슨 `sudo systemctl restart avahi-daemon` 을 아직 안 했다. 하고 나면 IP 대신 `mlcs-car.local` 로 접속할 수 있다. 그 전에는 IP 로.

## 노트북 기록

> 형식: `### <노트북 이름> 시험 N (날짜 시각, 커밋)`. 단계마다 ✓/✗, ✗ 는 에러 원문.
> 젯슨 답은 `### 젯슨 답 N` 으로 그 아래에.

(아직 없음)
