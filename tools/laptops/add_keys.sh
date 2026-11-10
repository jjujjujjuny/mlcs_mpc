#!/usr/bin/env bash
# ============================================================================
#  add_keys.sh — ★ 젯슨에서 쓰는 것. 레포에 올라온 노트북 공개키를 젯슨 SSH 에 등록
#
#     cd ~/mlcs_mpc && git pull && tools/laptops/add_keys.sh
#
#  노트북이 젯슨 비밀번호를 모를 때의 길이다 (비밀번호를 알면 노트북에서
#  ./car.sh key 한 번이면 끝). 노트북에서 ./car.sh pubkey → commit·push →
#  젯슨에서 이 스크립트.
#
#  · tools/laptops/keys/*.pub 중 ~/.ssh/authorized_keys 에 없는 것만 덧붙인다.
#    지우지는 않는다 (빼려면 authorized_keys 에서 그 줄을 직접 지운다).
#  · 공개키라 레포에 있어도 된다. 다만 이 레포에 push 할 수 있는 사람은
#    젯슨에 들어올 수 있게 되는 것이니, 키 파일은 아는 사람 것만.
# ============================================================================
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AK="$HOME/.ssh/authorized_keys"
mkdir -p "$HOME/.ssh" && chmod 700 "$HOME/.ssh"
touch "$AK" && chmod 600 "$AK"

added=0
shopt -s nullglob
for f in "$HERE"/keys/*.pub; do
    n="$(basename "$f" .pub)"
    line="$(grep -v '^[[:space:]]*$' "$f" | head -1)"
    # 한 줄, 알려진 종류, 옵션(command= 등) 없이 — 이상한 줄은 넣지 않는다
    if ! [[ "$line" =~ ^(ssh-ed25519|ssh-rsa|ecdsa-sha2-nistp(256|384|521))\ [A-Za-z0-9+/=]+(\ [^[:cntrl:]]*)?$ ]] \
       || [ "$(grep -cv '^[[:space:]]*$' "$f")" != 1 ]; then
        echo "✗ $n — 공개키 형식이 아니라 건너뜀"; continue
    fi
    key="$(echo "$line" | awk '{print $2}')"
    if grep -qF "$key" "$AK"; then
        echo "  $n — 이미 있음"
    else
        echo "$line" >> "$AK"; echo "✓ $n — 등록"; added=$((added+1))
    fi
done
echo "새로 등록 $added 개 (전체 $(grep -c . "$AK") 줄: $AK)"
