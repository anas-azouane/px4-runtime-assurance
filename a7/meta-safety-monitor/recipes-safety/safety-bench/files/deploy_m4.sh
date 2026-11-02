#!/bin/sh
# Load the monitor on the M4 and wait for its rpmsg tty. Runs on the A7.
set -eu

FW="${1:?usage: deploy_m4.sh <zephyr.elf>}"
RPROC=/sys/class/remoteproc/remoteproc0
NAME=$(basename "$FW")

[ -e "$RPROC" ] || { echo "remoteproc0 not found (device tree?)" >&2; exit 1; }

if [ "$(cat $RPROC/state)" = "running" ]; then
    echo stop > $RPROC/state
    sleep 0.5
fi

cp "$FW" /lib/firmware/"$NAME"
echo "$NAME" > $RPROC/firmware
echo start  > $RPROC/state

for _ in $(seq 1 50); do
    [ -e /dev/ttyRPMSG0 ] && break
    sleep 0.1
done
[ -e /dev/ttyRPMSG0 ] || { echo "M4 started but /dev/ttyRPMSG0 never appeared" >&2; exit 1; }
echo "M4 running: $NAME, link: /dev/ttyRPMSG0"
