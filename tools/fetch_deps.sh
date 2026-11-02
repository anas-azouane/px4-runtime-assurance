#!/bin/sh
# Pinned so a MAVLink update never changes the firmware behind a PR's back.
set -eu
MAVLINK_REV=a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29
dir="$(dirname "$0")/../third_party/c_library_v2"

if [ "$(git -C "$dir" rev-parse HEAD 2>/dev/null)" != "$MAVLINK_REV" ]; then
    rm -rf "$dir"
    mkdir -p "$dir"
    git -C "$dir" init -q
    git -C "$dir" remote add origin https://github.com/mavlink/c_library_v2.git
    git -C "$dir" fetch -q --depth 1 origin "$MAVLINK_REV"
    git -C "$dir" checkout -q FETCH_HEAD
fi
