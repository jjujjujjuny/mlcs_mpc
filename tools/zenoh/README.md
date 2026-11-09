# 노트북에서 젯슨 보기 — SSH + tmux + Zenoh 브리지

대회에서 NoMachine 화면 공유 대신 쓴다. 젯슨은 토픽만 보내고, RViz 는 노트북에서 그린다.

```
젯슨 노드들 ──(localhost DDS)── 젯슨 브리지 ══(와이파이 TCP 7447 하나)══ 노트북 브리지 ── 노트북 RViz
```

| 용도 | 방법 |
|---|---|
| 터미널 | `ssh mlcs@<젯슨>` → `tmux new -A -s race` (끊겨도 주행은 계속) |
| RViz | 젯슨 `./drive.sh bridge` + 노트북 `tools/zenoh/laptop.sh` |

- 젯슨 ROS 는 `ROS_LOCALHOST_ONLY=1` 이다 (`drive.sh` 의 `load_ros`). 밖으로 나가는 길은 브리지 하나뿐이라 와이파이가 흔들려도 주행 노드끼리는 영향이 없고, 같은 공유기의 다른 팀 ROS 와 섞이지 않는다.
- 보낼 토픽과 주파수는 [jetson.json5](jetson.json5) 에서 고친다. 지금은 `/tf` `/tf_static` `/map`(0.5Hz) `/scan`(10Hz) `/tracked_pose`(20Hz) `/mpc/*` `/hmpc/status` 마커들. 노트북 → 젯슨은 `/initialpose` 하나 (RViz 2D Pose Estimate).
- 2026-11-09 젯슨 안에서 도메인 2개로 시험: `/scan` 40→10Hz, `/tf` 100Hz 그대로, `/initialpose` 역방향 전달, 목록 밖 토픽 차단, 젯슨 브리지 재시작 후 노트북 자동 재접속. 젯슨 브리지 CPU ≈0%, 메모리 9MB.

## 노트북 준비 (한 번만)

**1. 환경 확인** — 젯슨과 같은 ROS 2 Humble 이어야 메시지가 맞는다.

```bash
lsb_release -a          # Ubuntu 22.04 기대
printenv ROS_DISTRO     # humble
which rviz2
```

**2. 브리지 실행 파일 받기** (레포에는 없다, 26MB)

```bash
cd ~/mlcs_mpc/tools/zenoh     # 레포 위치에 맞게
mkdir -p x86_64 && cd /tmp
curl -L -o zb.zip https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.10.2/zenoh-plugin-ros2dds-1.10.2-x86_64-unknown-linux-gnu-standalone.zip
unzip -o zb.zip -d ~/mlcs_mpc/tools/zenoh/x86_64
~/mlcs_mpc/tools/zenoh/x86_64/zenoh-bridge-ros2dds --version    # v1.10.2
```

인터넷이 없으면 젯슨에 이미 받아 둔 것을 복사한다: `scp -r mlcs@<젯슨IP>:~/mlcs_mpc/tools/zenoh/x86_64 ~/mlcs_mpc/tools/zenoh/`
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

2. tmux 창 1: 주행 (`./race.sh` 또는 시험이면 `./drive.sh bench`)
   tmux 창 2 (`Ctrl+b` → `c`): `cd ~/mlcs_mpc && ./drive.sh bridge`

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

## 아직 남은 것

- [ ] 젯슨 `~/.bashrc` 에 `export ROS_LOCALHOST_ONLY=1` + `ros2 daemon stop` (젯슨 사용자가 직접)
- [ ] 젯슨 mDNS 이름: 호스트명은 `mlcs-car` 로 바꿨지만 avahi 가 아직 `ubuntu.local` 로 광고한다 → `sudo systemctl restart avahi-daemon` 하면 `mlcs-car.local` 로 접속 가능. 그 전에는 IP 로
- [ ] 젯슨에 `yax_5G` 와이파이 프로필 등록 (비밀번호 필요)
- [ ] 노트북 시험이 되면 `race.sh` 가 브리지를 자동으로 함께 띄우게
