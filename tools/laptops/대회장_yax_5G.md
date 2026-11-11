# 대회장 `yax_5G` 에서 젯슨·노트북 연결하기 (인터넷 없음)

> 대회 공유기 `yax_5G` 에는 **인터넷이 없다.** 그래서 대회장에서는 다음이 안 된다.
> - Claude Code (노트북·젯슨 모두 인터넷이 있어야 돈다)
> - `git pull`·`push`, `apt install`, Rust·브리지 빌드
> - NTP 시계 맞추기
>
> **설치·빌드·업데이트는 전부 연구실(인터넷 있음)에서 끝내고 간다.** 대회장에서는 이 문서를 보고 사람이 직접 한다. 출력해 가거나 폰에 띄워 둘 것.
>
> 2026-11-11 작성. 아직 아무도 `yax_5G` 에서 해 보지 않았다. 해 보면 맨 아래 '대회장 기록' 에 적는다.

---

## A. 연구실에서 미리 (인터넷 있을 때)

### A-1. 젯슨에 `yax_5G` 프로필 넣기 — 한 번

공유기가 눈앞에 없어도 프로필은 미리 만들 수 있다. 비밀번호는 대회 측에서 받는다.

```bash
# 젯슨에서 (ssh mlcs@192.168.1.7 로 들어가서). <비밀번호> 만 바꾼다
sudo nmcli connection add type wifi ifname wlP1p1s0 con-name yax_5G ssid yax_5G \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk '<비밀번호>' \
    connection.autoconnect yes connection.autoconnect-priority 20
nmcli -t -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show | grep yax_5G   # yax_5G:yes:20
```

- 젯슨 와이파이 장치는 `wlP1p1s0` 이다. 연구실 프로필 `mlcs-robotics5G`·`choilab5G` 는 우선순위 0 이다.
  - 대회장에는 연구실 공유기가 없으니 `yax_5G` 로 붙는다.
  - 둘 다 보이는 곳이면 우선순위가 높은 `yax_5G` 로 간다.
- 주소는 DHCP(자동) 그대로 둔다. 고정 IP 는 다른 팀 기기와 겹칠 수 있다. 주소는 아래 `./car.sh find` 가 찾는다.
- 대회 공유기가 WPA2-PSK 가 아니면(WPA3, 숨긴 SSID 등) 위 명령을 바꿔야 한다. 그때는 대회장에서 B-2 를 따른다.

### A-2. 모든 노트북 — 출발 전날 확인

각 노트북에서 연구실 공유기(`mlcs-robotics5G`)에 붙어서:

```bash
cd ~/mlcs_mpc && git pull       # 젯슨과 같은 커밋으로
./car.sh setup                  # 전부 '✓ 이미 있음' 이어야 한다 (브리지 v1.10.2, SSH 키)
./car.sh ip 192.168.1.7
./car.sh panel                  # SSH ✓ 가 초록인지 확인하고 닫는다
```

- **`./car.sh panel` 을 한 번 띄우는 게 중요하다.**
  - 이때 젯슨 호스트 키를 `mlcs-car` 라는 이름으로 기억한다. 그래야 대회장에서 IP 가 바뀌어도 SSH 가 막히지 않는다.
  - 처음 띄울 때 배터리 메시지 빌드(vesc_msgs)도 끝난다.
- 결과는 `[car] 젯슨 호스트 키 기록 (mlcs-car)` 한 줄이 나오거나, 이미 기록돼 있어 아무 말이 없거나 둘 중 하나다. 확인: `ssh-keygen -F mlcs-car` 가 한 줄 이상.
- 젯슨에서도 `git pull` 로 레포를 맞춘다. 대회장에서는 git 을 못 한다.

### A-3. 가져갈 것

- 노트북 충전기
- 젯슨 배터리 여분
- 조이스틱 (차 정지는 조이스틱이 한다 — 노트북 아님)
- 젯슨 모니터·키보드 대용 수단 — 젯슨이 공유기에 안 붙을 때 직접 봐야 한다 (B-2)
- 이 문서

---

## B. 대회장에서

### B-1. 순서 (보통 이것으로 끝)

1. **젯슨 켜기.** 1~2분 기다리면 `yax_5G` 에 스스로 붙는다.
2. **노트북을 `yax_5G` 에 연결.** 우분투 오른쪽 위 메뉴 → Wi-Fi → `yax_5G`.
   - "인터넷 없음" 표시가 떠도 괜찮다.
   - ★ 폰 핫스팟 같은 다른 와이파이로 자동 전환되지 않게, 대회장에서는 다른 와이파이를 꺼 두거나 잊어버린다.
3. **젯슨 찾기:**
   ```bash
   cd ~/mlcs_mpc
   ./car.sh find
   ```
   - 먼저 `mlcs-car.local` 이름으로 찾는다.
   - 안 되면 같은 공유기(`/24`)에서 SSH 가 열린 기기들에 hostname 을 물어 `mlcs-car` 를 찾는다. 약 3초, 인터넷도 nmap 도 필요 없다.
   - 찾으면 주소를 저장하고 시계 차이까지 보여 준다.
   ```
   [car] 젯슨: 192.168.x.x 저장
     시계 차이 (젯슨 − 노트북): +0.004 s  (왕복 9 ms)
   ```
4. **제어판:** `./car.sh panel` → `SSH ✓` 초록 확인.
5. **bench 로 확인** (속도 0, 조향만): `bench` → 노드 점이 모두 초록, `토픽 ✓`, 라이다 ≈10 Hz → `RViz` 에서 맵·스캔·차 위치 → `정지 (Ctrl+C)`.
6. 노트북 두 대 모두 3~5 를 한다. 둘이 동시에 봐도 되지만 **실행·정지·설정은 한 사람만**.

### B-2. 안 될 때

| 증상 | 볼 것 |
|---|---|
| `./car.sh find` → `이 노트북에 IP 가 없습니다` | 노트북이 `yax_5G` 에 붙었는지 |
| `find` → `못 찾음 — SSH 22 열린 곳: (빈칸)` | 젯슨이 공유기에 안 붙음 → 아래 '젯슨 직접 보기'. 또는 공유기 AP 격리(기기끼리 통신 막음) — 노트북 두 대끼리 `ping` 이 되는지로 확인. 안 되면 대회 측에 문의 |
| `find` → `못 찾음 — SSH 22 열린 곳: 192.168.x.a 192.168.x.b …` | 다른 팀 기기만 보인 것. 젯슨이 아직 안 붙었거나 이 노트북 키가 젯슨에 없음 (`./car.sh key`) |
| 제어판 `SSH ✗` 인데 `ping` 은 됨 | 터미널에서 `ssh -o HostKeyAlias=mlcs-car mlcs@<IP> hostname` 를 쳐서 메시지를 본다 |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | **그 주소에 있는 건 우리 젯슨이 아니다.** 주소를 다시 확인(`./car.sh find`). 젯슨을 재설치한 경우에만 `ssh-keygen -R mlcs-car` 후 다시 |
| RViz 가 비어 있음, 제어판 `토픽` 회색 | 젯슨에서 아무것도 안 돌면 정상 (브리지는 bench·주행·맵핑 때만 뜬다). 돌고 있는데도면 `./car.sh check` |

**젯슨 직접 보기** (공유기에 안 붙을 때) — 젯슨에 모니터·키보드를 꽂고 터미널에서:

```bash
nmcli -t -f DEVICE,STATE,CONNECTION device | grep wlP1p1s0   # wlP1p1s0:connected:yax_5G 여야 한다
nmcli device wifi list | grep yax_5G                        # 공유기가 보이는지, 신호 세기
sudo nmcli connection up yax_5G                             # 수동으로 붙이기 (에러 원문 확인)
hostname -I                                                 # 첫 주소가 대회장 IP 다 (192.168.0.15 는 라이다 유선)
```

프로필을 아예 다시 만들려면 `sudo nmcli connection delete yax_5G` 후 A-1.

### B-3. 시계

대회장에는 NTP 가 없다. 젯슨(RTC)과 노트북이 각자 시계로 간다. 하루 안에 크게 어긋나지는 않지만, `./car.sh find`·`./car.sh clock` 이 0.2 s 넘게 나오면 RViz 에 `extrapolation into the future` 같은 tf 경고가 나고 화면이 끊길 수 있다.

- **주행에는 영향 없다.** 젯슨 노드끼리는 모두 젯슨 시계를 쓴다. 노트북 화면 문제일 뿐이다.
- 맞추려면 **젯슨에서 아무것도 안 돌 때만** 젯슨 시계를 노트북 쪽에 맞춘다 (돌고 있을 때 시계를 바꾸면 ROS 시간이 튄다):
  ```bash
  ./car.sh stop
  ssh -t -o HostKeyAlias=mlcs-car mlcs@$(cat tools/zenoh/.car_host) "sudo date -s @$(date +%s.%N)"   # 젯슨 비밀번호
  ./car.sh clock      # ±0.05 s 안쪽이면 됨
  ```

### B-4. 대회장에서 하지 말 것

- `git pull` 시도 — 안 된다. 코드를 바꿔야 하면 젯슨에서 직접 고치고, 연구실에 돌아와서 커밋한다.
- 젯슨 tmux 안(`./car.sh term`)에서 `./car.sh` 나 `tmux` 를 또 치기.
- 노트북에서 실제 주행 시작은 킬스위치(조이스틱)를 든 사람이 차 옆에 있을 때만.

---

## C. 알아 둘 위험 — 같은 공유기에 다른 팀이 있다

`yax_5G` 는 모든 팀이 같이 쓴다. 지금 젯슨에서 밖으로 열린 문은 두 개다.

| 문 | 지금 | 위험 |
|---|---|---|
| SSH 22 | 키 + **비밀번호 로그인도 허용** | 젯슨 `mlcs` 비밀번호가 약하면 남이 들어올 수 있다 |
| Zenoh 7447 (bench·주행·맵핑 중에만) | **인증 없음** | 같은 공유기의 누구든 붙어서 토픽을 보고, `/initialpose`(위치 덮어쓰기)·`/mpc/go`(출발)를 보낼 수 있다 |

ROS 자체(DDS)는 `ROS_LOCALHOST_ONLY=1` 이라 밖에서 안 보인다. 문제는 위 두 문이다. 막는 방법은 정해서 연구실에서 해 둔다 (아직 안 함, 결정 필요):

1. ~~SSH 비밀번호 로그인 끄기~~ — **안 함 (2026-11-11 사용자 결정: 젯슨 비밀번호가 강하다).** 비밀번호 로그인은 그대로 둔다 (`./car.sh key` 가 쓴다).
2. Zenoh 를 SSH 터널 안으로. 젯슨 브리지는 `127.0.0.1:7447` 에서만 듣고, 노트북 브리지는 `ssh -L` 로 붙는다.
   - 인증은 SSH 키가 맡는다.
   - 젯슨 CPU 는 SSH 암호화만큼 조금 는다. 토픽량이 작아 1 % 안쪽일 것으로 본다 (측정 필요).

---

## 대회장 기록

> 형식: `### <날짜> <노트북> — 단계별 ✓/✗, 에러 원문, 젯슨 IP, 시계 차이, ping`
> 대회장에서는 push 를 못 하니 메모해 두었다가 연구실에서 적는다.

(아직 없음)
