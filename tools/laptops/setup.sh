#!/usr/bin/env bash
# ============================================================================
#  setup.sh — ★ 노트북에서 쓰는 것. 새 노트북을 젯슨 제어판용으로 준비 (한 번)
#
#     ./car.sh setup          (= tools/laptops/setup.sh)
#
#  된 단계는 건너뛴다. 몇 번을 다시 돌려도 된다. 하는 일:
#     1. Ubuntu 22.04 + ROS 2 Humble 확인 (젯슨과 메시지가 맞아야 한다)
#     2. apt 패키지 — 없는 것만 sudo apt install (비밀번호를 묻는다)
#     3. ~/.bashrc 에 export ROS_LOCALHOST_ONLY=1
#     4. Rust (rustup) — 브리지 빌드용
#     5. Zenoh 브리지 1.10.2 소스 빌드 → tools/zenoh/x86_64/ (약 3~20분)
#        ★ 미리 빌드된 x86_64 는 22.04 에서 안 뜬다 (gnu = glibc 2.38, musl = 동적 링크,
#          2026-11-09 hj-gram 실측). 그래서 직접 빌드한다.
#     6. SSH 키 (~/.ssh/id_ed25519) 만들기 — 젯슨 등록은 ./car.sh key 또는 ./car.sh pubkey
#
#  sudo 가 필요한 2번은 사람이 터미널에서 돌려야 한다 (비밀번호 입력).
# ============================================================================
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
ZB_DIR="$ROOT/tools/zenoh/x86_64"
ZB="$ZB_DIR/zenoh-bridge-ros2dds"
ZV=1.10.2
SRC="$HOME/zenoh-plugin-ros2dds"

RED=$'\033[31m'; GRN=$'\033[32m'; YLW=$'\033[33m'; BLD=$'\033[1m'; RST=$'\033[0m'
step() { echo; echo "${BLD}[setup $1]${RST} $2"; }
ok()   { echo "  ${GRN}✓${RST} $*"; }
warn() { echo "  ${YLW}⚠${RST} $*"; }
die()  { echo "  ${RED}✗ $*${RST}" >&2; exit 1; }

# ── 1 ──────────────────────────────────────────────────────────────────
step 1 "Ubuntu·ROS 확인"
. /etc/os-release 2>/dev/null
[ "${VERSION_ID:-}" = 22.04 ] && ok "Ubuntu $VERSION_ID" \
    || warn "Ubuntu ${VERSION_ID:-?} — 젯슨·hj-gram 은 22.04. 다르면 아래 빌드·메시지가 안 맞을 수 있다"
[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble 이 없습니다 (/opt/ros/humble). 먼저 설치: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html (ros-humble-desktop)"
ok "ROS 2 Humble"
[ -x /opt/ros/humble/bin/rviz2 ] && ok "rviz2" || die "rviz2 가 없습니다: sudo apt install -y ros-humble-desktop"

# ── 2 ──────────────────────────────────────────────────────────────────
step 2 "apt 패키지"
PKGS="python3-pyqt5 python3-colcon-common-extensions build-essential cmake clang libclang-dev git curl pkg-config iputils-ping openssh-client"
miss=""
for p in $PKGS; do dpkg -s "$p" >/dev/null 2>&1 || miss="$miss $p"; done
if [ -z "$miss" ]; then
    ok "모두 있음"
else
    echo "  없는 것:$miss"
    if [ -t 0 ]; then
        sudo apt install -y $miss || die "apt 설치 실패 — 위 에러 확인"
        ok "설치함"
    else
        die "터미널에서 직접:  sudo apt install -y$miss   (비밀번호가 필요해 여기서는 못 함) — 그 뒤 ./car.sh setup 다시"
    fi
fi

# ── 3 ──────────────────────────────────────────────────────────────────
step 3 "~/.bashrc 의 ROS_LOCALHOST_ONLY=1"
if grep -q '^export ROS_LOCALHOST_ONLY=1' ~/.bashrc 2>/dev/null; then
    ok "이미 있음"
else
    echo 'export ROS_LOCALHOST_ONLY=1   # 젯슨 제어판: 노트북 ROS 는 노트북 안에서만 (mlcs_mpc tools/laptops)' >> ~/.bashrc
    ok "덧붙임 (새 터미널부터 적용)"
fi

# ── 4 ──────────────────────────────────────────────────────────────────
step 4 "Rust"
[ -f "$HOME/.cargo/env" ] && . "$HOME/.cargo/env"
if command -v cargo >/dev/null; then
    ok "$(cargo --version)"
elif "$ZB" --version 2>/dev/null | grep -q "v$ZV"; then
    ok "브리지가 이미 있어 필요 없음"
else
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal \
        || die "rustup 설치 실패 (인터넷 확인)"
    . "$HOME/.cargo/env"; ok "$(cargo --version)"
fi

# ── 5 ──────────────────────────────────────────────────────────────────
step 5 "Zenoh 브리지 $ZV (젯슨과 같은 버전)"
if "$ZB" --version 2>/dev/null | grep -q "v$ZV"; then
    ok "이미 있음: $("$ZB" --version 2>/dev/null | head -1)"
else
    if [ ! -d "$SRC/.git" ]; then
        git clone --depth 1 -b "$ZV" https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git "$SRC" \
            || die "소스 받기 실패"
    fi
    echo "  빌드 중 (hj-gram 8코어 2분 42초, 느린 노트북은 20분 가까이). 로그: ${TMPDIR:-/tmp}/mlcs_zenoh_build.log"
    ( cd "$SRC" && cargo build --release -p zenoh-bridge-ros2dds ) > "${TMPDIR:-/tmp}/mlcs_zenoh_build.log" 2>&1 \
        || { tail -30 "${TMPDIR:-/tmp}/mlcs_zenoh_build.log"; die "빌드 실패 — 위 30줄. 흔한 원인: libclang 못 찾음(libclang-dev), 'stddef.h' not found(clang)"; }
    mkdir -p "$ZB_DIR" && cp "$SRC/target/release/zenoh-bridge-ros2dds" "$ZB"
    ok "$("$ZB" --version 2>/dev/null | head -1)"
    g="$(objdump -T "$ZB" 2>/dev/null | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -1)"
    [ -n "$g" ] && ok "요구 glibc 최대 $g"
fi

# ── 6 ──────────────────────────────────────────────────────────────────
step 6 "SSH 키"
if [ -f ~/.ssh/id_ed25519.pub ]; then
    ok "있음: ~/.ssh/id_ed25519.pub ($(awk '{print $3}' ~/.ssh/id_ed25519.pub))"
else
    mkdir -p ~/.ssh && chmod 700 ~/.ssh
    ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519 -C "$(whoami)@$(hostname)" >/dev/null && ok "만듦"
fi

echo
echo "${GRN}${BLD}준비 끝.${RST} 다음:"
echo "  1) ./car.sh ip 192.168.1.7      (연구실 mlcs-robotics5G. 대회장에서는 그때 IP)"
echo "  2) 젯슨 SSH 등록 — 둘 중 하나:"
echo "       ./car.sh key               젯슨 비밀번호를 알면 (한 번 입력)"
echo "       ./car.sh pubkey            모르면: 공개키를 레포에 넣고 push → 젯슨에서 tools/laptops/add_keys.sh"
echo "  3) ./car.sh panel"
