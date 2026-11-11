# 노트북에서 젯슨 보기 — SSH + tmux + Zenoh 브리지

> **노트북에서는 레포 맨 위의 `./car.sh` 하나로 한다** (2026-11-09 추가)
>
> ```
> ./car.sh ip 192.168.1.7   # 젯슨 주소 저장 (대회장에서 바뀌면 다시)
> ./car.sh key              # SSH 비밀번호 안 묻게 (한 번만)
> ./car.sh bench            # 젯슨 race.sh 를 bench 로 (race.yaml 안 고침)
> ./car.sh rviz             # 다른 창: 노트북 RViz
> ./car.sh check            # 또 다른 창: /scan /tracked_pose 주파수
> ./car.sh stop             # 끝
> ```
> `./car.sh` 만 치면 전체 사용법. 아래는 그 안에서 무슨 일이 일어나는지에 대한 설명이다.

대회에서 NoMachine 화면 공유 대신 쓴다. 젯슨은 토픽만 보내고, RViz 는 노트북에서 그린다.

```
젯슨 노드들 ──(localhost DDS)── 젯슨 브리지(127.0.0.1:7447) ══(SSH 터널)══ 노트북 브리지 ── 노트북 RViz
```

> **2026-11-11 부터 SSH 터널 전용.** 젯슨 브리지는 와이파이에 열지 않는다 — 같은 공유기의 누구든 `/initialpose`·`/mpc/go` 를 보낼 수 있었다. 노트북은 [link.sh](link.sh) (제어판·`car.sh rviz` 가 부름)가 `ssh -L 17447→7447` 로 붙는다. 아래 기록의 `tcp/<IP>:7447` 직접 접속은 그 전 방식이다.

| 용도 | 방법 |
|---|---|
| 터미널 | `ssh mlcs@<젯슨>` → `tmux new -A -s race` (끊겨도 주행은 계속) |
| RViz | 젯슨 `./race.sh` (브리지 자동) 또는 `./drive.sh bridge` + 노트북 `tools/zenoh/laptop.sh` |

- 젯슨 ROS 는 `ROS_LOCALHOST_ONLY=1` 이다 (`drive.sh` 의 `load_ros`). 밖으로 나가는 길은 브리지 하나뿐이라 와이파이가 흔들려도 주행 노드끼리는 영향이 없고, 같은 공유기의 다른 팀 ROS 와 섞이지 않는다.
- 보낼 토픽과 주파수는 [jetson.json5](jetson.json5) 에서 고친다. 지금은 `/tf` `/tf_static` `/map`(0.5Hz) `/scan`(10Hz) `/tracked_pose`(20Hz) `/mpc/*` `/hmpc/status` 마커들. 노트북 → 젯슨은 `/initialpose` 하나 (RViz 2D Pose Estimate).
- 2026-11-09 젯슨 안에서 도메인 2개로 시험: `/scan` 40→10Hz, `/tf` 100Hz 그대로, `/initialpose` 역방향 전달, 목록 밖 토픽 차단, 젯슨 브리지 재시작 후 노트북 자동 재접속. 젯슨 브리지 CPU ≈0%, 메모리 9MB.

## 노트북 준비 (한 번만)

> **새 노트북은 [../laptops/README.md](../laptops/README.md) 의 `./car.sh setup` 으로 한다** (2026-11-10). 아래 1~3 을 자동으로 하고, 2번은 아래 musl 대신 '젯슨 답 2' 의 소스 빌드로 한다. 아래는 그때까지의 기록이다.

**1. 환경 확인** — 젯슨과 같은 ROS 2 Humble 이어야 메시지가 맞는다.

```bash
lsb_release -a          # Ubuntu 22.04 기대
printenv ROS_DISTRO     # humble
which rviz2
```

**2. 브리지 실행 파일 받기** (레포에는 없다)

★ 노트북은 **musl** 빌드를 쓴다. gnu 빌드 1.10.2 는 glibc 2.38 을 요구해서 22.04(2.35)에서 안 뜬다 (2026-11-09 노트북 실측). musl 은 glibc 를 안 쓴다. 젯슨은 aarch64 gnu 그대로 (GLIBC 2.30 까지만 요구, 실행 확인).

```bash
cd ~/mlcs_mpc/tools/zenoh     # 레포 위치에 맞게
mkdir -p x86_64 && cd /tmp
curl -L -o zb.zip https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.10.2/zenoh-plugin-ros2dds-1.10.2-x86_64-unknown-linux-musl-standalone.zip
unzip -o zb.zip -d ~/mlcs_mpc/tools/zenoh/x86_64
~/mlcs_mpc/tools/zenoh/x86_64/zenoh-bridge-ros2dds --version    # v1.10.2
```

**버전은 양쪽 1.10.2 로 맞출 것** (젯슨 aarch64 도 1.10.2).

**3. 노트북 `~/.bashrc`** — 노트북 RViz 와 브리지가 노트북 안에서만 만나게 한다.

```bash
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc
```

## 접속 시험 (지금 연구실 공유기에서 해도 된다)

젯슨 주소: 연구실 `mlcs-robotics5G` 에서는 **192.168.1.7**. 대회 `yax_5G` 에서는 `./drive.sh bridge` 가 화면에 보여준다.

1. 노트북 → 젯슨 SSH, tmux

   ```bash
   ssh mlcs@192.168.1.7
   tmux new -A -s race
   ```

2. 주행: `./race.sh` — **브리지가 같이 뜨고 같이 내려간다** (끄려면 `MLCS_BRIDGE=0 ./race.sh`).
   주행 없이 RViz 연결만 시험하려면 tmux 창 2 (`Ctrl+b` → `c`)에서 `cd ~/mlcs_mpc && ./drive.sh bridge`

3. 노트북 다른 터미널:

   ```bash
   cd ~/mlcs_mpc/tools/zenoh && ./laptop.sh 192.168.1.7
   ```

4. 확인할 것
   - RViz 에 맵·스캔·차 위치가 보이는가 (Fixed Frame = `map`)
   - 노트북에서 `ros2 topic hz /scan` 이 약 10Hz 인가
   - RViz `2D Pose Estimate` 를 찍으면 젯슨 쪽 위치가 옮겨지는가
   - 노트북 와이파이를 껐다 켜면 RViz 가 다시 살아나는가 (브리지가 알아서 재접속)

결과(되는 것, 안 되는 것, 에러 메시지)를 레포에 적어 주면 젯슨에서 이어서 본다.

## 노트북 시험 결과 (2026-11-09, hj-gram)

- 1단계 환경 확인 ✓: Ubuntu 22.04.5, `ROS_DISTRO=humble`, `/opt/ros/humble/bin/rviz2`. 젯슨 192.168.1.7 ping 3ms.
- 2단계 브리지 받기 ✗: README 의 curl 로 받은 `x86_64-unknown-linux-gnu-standalone` 1.10.2 가 실행되지 않는다.
  ```
  ./x86_64/zenoh-bridge-ros2dds: /lib/x86_64-linux-gnu/libc.so.6: version `GLIBC_2.38' not found (required by ./x86_64/zenoh-bridge-ros2dds)
  ```
  노트북 glibc 는 2.35 (`ldd --version`). 젯슨의 x86_64 사본도 같은 파일이면 같은 에러일 것.
- 3단계 `~/.bashrc` 에 `export ROS_LOCALHOST_ONLY=1` ✓
- 4·5단계 접속 시험: 브리지가 안 떠서 못 함. (이때 젯슨 7447 은 닫혀 있었다)

### musl 빌드 시험 (546b9b6 이후)

- 1번 `--version` ✗: musl-standalone 1.10.2 도 실행되지 않는다.
  ```
  $ ./x86_64/zenoh-bridge-ros2dds --version
  bash: ./x86_64/zenoh-bridge-ros2dds: No such file or directory      (rc=127)
  ```
  정적 빌드가 아니라 musl 동적 링크다. `file`: `dynamically linked, interpreter /lib/ld-musl-x86_64.so.1`, `readelf -d` NEEDED: `libgcc_s.so.1`, `libc.so`. 노트북에는 `/lib/ld-musl-x86_64.so.1` 이 없다 (musl 미설치).
- 참고 (546b9b6 전에 따로 해 본 것): 우분투 `musl` 패키지의 로더 + 우분투 `/lib/x86_64-linux-gnu/libgcc_s.so.1` 로 띄우면 `Error relocating .../libgcc_s.so.1: _dl_find_object: symbol not found` (glibc 용 libgcc_s 라 안 맞음). musl 용 libgcc_s 가 따로 있어야 한다.
- 2~4번은 브리지가 안 떠서 못 함.

### 소스 빌드 시험 (젯슨 답 2, 2026-11-09 23:00)

- 1) 빌드 도구 ✓: `clang` `libclang-dev` 를 apt 로 설치 (나머지는 이미 있었음), rustup 으로 Rust 설치. (clang 없이 먼저 해 보면 bindgen 에서 `'stddef.h' file not found` 로 실패한다 — clang 헤더가 필요)
- 2) `cargo build --release -p zenoh-bridge-ros2dds` ✓ 2분 42초 (8코어)
- 3) `objdump` 최대 `GLIBC_2.34` ✓, `--version` → `zenoh-bridge-ros2dds v1.10.2` ✓
- `./laptop.sh 192.168.1.7` ✓ 브리지 접속, RViz 뜸. 브리지 로그에 `/initialpose` (ROS → Zenoh) 경로 생성
- `ros2 topic echo /hmpc/status` ✓ "젯슨 브리지 시험 메시지" 1초마다
- 와이파이 끄고 켜기 ✓: `nmcli radio wifi off` 10초 → `on`. 와이파이 재연결에 1분 넘게 걸렸지만, 붙은 뒤에는 브리지를 다시 띄우지 않아도 메시지가 다시 들어옴
- 아직 안 한 것: 맵·스캔·차 위치 표시, `/scan` 10Hz, `2D Pose Estimate` 가 젯슨에 도착하는지 (젯슨에서 bench 를 띄운 다음)

### car.sh 시험 (젯슨 답 3, 2026-11-09 23:30)

모두 됨 ✓
- `./car.sh ip 192.168.1.7` ✓, `./car.sh key` ✓ (이후 비밀번호 없이 SSH)
- `./car.sh bench` ✓ 젯슨 tmux 로 뜨고 `모드 bench (속도 0)` 줄 나옴
- `./car.sh rviz` ✓ 맵·스캔·차 위치(tracked_pose) 보임, Fixed Frame = map
- `./car.sh check` ✓
  ```
  /scan              9.9 Hz
  /tracked_pose     19.5 Hz
  /tf              210.4 Hz
  ```
- `2D Pose Estimate` ✓ 5번 찍음. RViz 차 위치가 옮겨지고, 젯슨 로그에 위치 다시 잡는 메시지 나옴
- RViz 로그에 맵 크기가 시험 중 바뀜: `234 x 365` → `297 x 309` → `298 x 309` (`/map` 이 도중에 다시 들어옴 — 의도한 것인지 젯슨에서 확인 바람)
- RViz 에러 2줄은 시작할 때 한 번씩만 나옴: `Lookup would require extrapolation into the future ... [laser] to frame [map]` (시작 직후 tf), `GLSL link result : active samplers with a different type refer to the same texture image unit` (노트북 그래픽)

### 노트북으로 맵핑 시도 (2026-11-09 23:38~23:45) — 중단

- 절차: `./car.sh term` → 젯슨 tmux 에서 `./drive.sh mapping`, 다른 창에서 `./drive.sh bridge` (race.sh 와 달리 mapping 은 브리지를 안 띄운다) → 노트북 `./car.sh rviz`
- 결과 ✗: SSH 터미널이 멈춤. Cartographer 는 23:41:08 시작 → 23:43:04 마지막 로그 후 꺼짐 (약 2분, 맵 저장 없음). 젯슨 브리지는 한 번도 안 떴다.
- 그때 상태: 노트북 와이파이 신호 100, 젯슨 `signal -40 dBm` 인데 rx `VHT-MCS 1 58.5 MBit/s`. 노트북→젯슨 ping 7~199 ms (손실 0).
- tmux `race` 에 창 3개, 창 1·2 이름이 `[tmux]` — 젯슨 tmux 안에서 tmux(`car.sh term` 등)를 또 연 것으로 보인다. 겹치면 `Ctrl+b` 가 바깥으로 가서 창 이동이 엉킨다.
- 노트북 `laptop.rviz` 에 `/trajectory_node_list` `/constraint_list` 가 있는데 `jetson.json5` 허용 목록에 없어 안 온다.
- 요청: `car.sh map` / `car.sh savemap <이름>` 처럼 맵핑 + 브리지를 한 번에 띄우는 명령, tmux 안에서 `car.sh term` 을 또 치면 막기, Cartographer 가 왜 2분 만에 꺼졌는지 (`~/mlcs_data/logs/20261109_234108_cartographer.log`)

## 아직 남은 것

- [x] 젯슨 `~/.bashrc` 에 `export ROS_LOCALHOST_ONLY=1` + `ros2 daemon stop`
- [x] `race.sh` (drive.sh hmpc) 가 브리지를 코어 0-3 에 자동으로 띄움, `./drive.sh stop` 이 같이 내림
- [x] 젯슨 mDNS 이름 — 2026-11-11 해결: 젯슨 재부팅으로 avahi 가 `mlcs-car.local` 로 광고 (`ubuntu.local` 은 이제 응답 없음). 노트북에서 `ping mlcs-car.local` 이 192.168.x 로 나오는지는 아직 확인 전 — 젯슨 자신에게는 docker 주소 172.17.0.1 로 나온다
- [ ] 젯슨에 `yax_5G` 와이파이 프로필 등록 (비밀번호 필요)

## 젯슨 답 (2026-11-09 22:52)

- 노트북은 **musl 빌드**로: 위 2단계 명령을 고쳐 두었다 (`x86_64-unknown-linux-musl-standalone`, 같은 1.10.2). 받은 뒤 `--version` 이 `v1.10.2` 로 뜨는지 먼저.
- 젯슨의 쓸모없는 x86_64 gnu 사본은 지웠다 (scp 로 가져오는 길도 README 에서 뺐다).
- **지금 젯슨에 시험용 브리지가 떠 있다** — tmux 세션 `bridge` (창 1 브리지, 창 2 시험 토픽). 주행 노드는 없다.
  - 시험 토픽: `/hmpc/status` (std_msgs/String, 1Hz, "젯슨 브리지 시험 메시지")
  - 노트북 확인 순서:
    1. `cd ~/mlcs_mpc/tools/zenoh && ./laptop.sh 192.168.1.7` (RViz 가 뜬다 — 지금은 맵·스캔이 없어 비어 있는 게 정상)
    2. 다른 터미널: `ros2 topic echo /hmpc/status` → 1초마다 시험 메시지가 나오면 연결 성공
    3. 노트북 와이파이 껐다 켜기 → 메시지가 다시 나오는지
  - 끝나면 젯슨에서 `tmux kill-session -t bridge`
- 맵·스캔까지 보는 시험은 젯슨에서 `./race.sh` (race.yaml `drive: false` 면 bench) 를 띄운 뒤 — 브리지는 race.sh 가 자동으로 띄운다. 그 전에 위 `bridge` 세션은 내릴 것 (안 내려도 '이미 떠 있다' 로 그대로 쓰긴 한다).

## 젯슨 답 2 (2026-11-09) — 노트북에서 소스 빌드

미리 빌드된 x86_64 는 둘 다 22.04 에서 안 된다 (gnu = glibc 2.38, musl = musl 동적 링크). **노트북에서 같은 태그 1.10.2 를 직접 빌드한다.** 노트북 glibc(2.35)로 링크되니 이 문제는 구조적으로 없다. 젯슨(aarch64 gnu 1.10.2)은 그대로.

```bash
# 1) 빌드 도구 (cyclors 가 CycloneDDS 를 C 로 같이 빌드해서 cmake·clang 필요)
sudo apt install -y build-essential cmake clang libclang-dev git curl pkg-config
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source ~/.cargo/env

# 2) 젯슨과 같은 태그로 빌드 (10~20분, 레포의 rust-toolchain 을 rustup 이 알아서 받는다)
git clone --depth 1 -b 1.10.2 https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git ~/zenoh-plugin-ros2dds
cd ~/zenoh-plugin-ros2dds && cargo build --release -p zenoh-bridge-ros2dds

# 3) 넣고 확인
rm -rf ~/mlcs_mpc/tools/zenoh/x86_64 && mkdir -p ~/mlcs_mpc/tools/zenoh/x86_64
cp target/release/zenoh-bridge-ros2dds ~/mlcs_mpc/tools/zenoh/x86_64/
objdump -T ~/mlcs_mpc/tools/zenoh/x86_64/zenoh-bridge-ros2dds | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -1   # 2.35 이하
~/mlcs_mpc/tools/zenoh/x86_64/zenoh-bridge-ros2dds --version                                             # v1.10.2
```

- 빌드가 실패하면 에러 마지막 30줄을 여기 적어 줄 것. 흔한 원인은 `libclang` 못 찾음(→ `libclang-dev`)과 cmake 버전.
- `--version` 이 되면 앞의 "젯슨 답" 절의 시험(`./laptop.sh 192.168.1.7` → `ros2 topic echo /hmpc/status`)을 그대로 이어서. 젯슨 tmux `bridge` 세션은 계속 켜 둔다.
- (빌드가 끝내 안 되면 다음 수단: 노트북 Docker 에서 Ubuntu 24.04 컨테이너로 gnu 빌드를 `--net=host` 로 돌리기. 지금은 쓰지 않는다)

## 젯슨 답 3 (2026-11-09) — car.sh 로 맵·스캔 시험

노트북 명령을 `./car.sh` 하나로 묶었다 (레포 맨 위). bench 는 race.yaml 을 안 고친다 (`MLCS_RACE_BENCH=1`).
젯슨 쪽에서는 tmux 명령 따옴표까지만 확인했고, **노트북에서 SSH 로 붙는 건 아직 아무도 안 돌려 봤다.**

1. `git pull` 후 `~/mlcs_mpc` 에서
2. `./car.sh ip 192.168.1.7` → `./car.sh key` (비밀번호 한 번)
3. 창 1: `./car.sh bench` — 젯슨 화면이 tmux 로 뜨고 race.sh 가 bench 로 돈다 (`모드 bench (속도 0)` 줄 확인)
4. 창 2: `./car.sh rviz` — 맵·스캔·차 위치(tracked_pose)가 보이는지. Fixed Frame = map
5. 창 3: `./car.sh check` — /scan ≈10Hz, /tracked_pose ≈20Hz
6. RViz `2D Pose Estimate` 로 차를 다른 데 찍기 → 창 1(젯슨 로그)에 위치 다시 잡는 메시지가 나오는지, RViz 의 차 위치가 옮겨지는지
7. 끝: 창 1 에서 Ctrl+C, 그리고 `./car.sh stop`

결과(되는 것·안 되는 것·에러 원문)를 이 README 의 '노트북 시험 결과' 절에 적어서 push.

