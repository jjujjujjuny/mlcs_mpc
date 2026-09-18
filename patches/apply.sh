#!/usr/bin/env bash
# 외부 패키지 패치를 일괄 적용한다. 이미 적용돼 있으면 건너뛴다.
set -u
WS="${MLCS_WS:-$HOME/f1tenth_ws}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rc=0

apply_one () {
    local repo="$1" patch="$2"
    local dir="$WS/src/$repo"
    if [ ! -d "$dir" ]; then
        echo "  건너뜀: $repo 가 없습니다 ($dir)"
        return
    fi
    if git -C "$dir" apply --reverse --check "$patch" 2>/dev/null; then
        echo "  이미 적용됨: $repo"
        return
    fi
    if git -C "$dir" apply --check "$patch" 2>/dev/null; then
        git -C "$dir" apply "$patch" && echo "  적용: $repo" || { echo "  ✗ 실패: $repo"; rc=1; }
    else
        echo "  ✗ 적용 불가: $repo — 업스트림이 바뀌었을 수 있습니다."
        echo "    $HERE/README.md 를 읽고 수동으로 넣으세요."
        rc=1
    fi
}

apply_one natnet_ros2 "$HERE/natnet_ros2-bitstream-version.patch"
exit $rc
