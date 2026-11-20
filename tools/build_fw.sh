#!/bin/sh
# Fetch a minimal Zephyr 4.4.2 workspace and build the M4 image. CI runs this;
# with an existing workspace, west build -b stm32mp157c_dk2 monitor is enough.
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
ws=${ZEPHYR_WS:-$HOME/zephyr-ws}

[ -d "$ws/.west" ] || west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.4.2 "$ws"
cd "$ws"
west update --narrow -o=--depth=1 cmsis_6 hal_stm32 open-amp libmetal
pip install -q -r zephyr/scripts/requirements-base.txt
west sdk install -t arm-zephyr-eabi
west build -p always -b stm32mp157c_dk2 -d "$here/build-fw" "$here/monitor"
