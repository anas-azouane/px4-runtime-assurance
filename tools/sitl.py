"""Start and stop a clean PX4 + Gazebo instance in Docker, headless unless asked."""
import os
import subprocess
import time
from pathlib import Path

IMAGE = os.environ.get("PX4_SITL_IMAGE", "px4-sitl:v1.16.2")
PX4_DIR = Path(os.environ.get("PX4_DIR", Path(__file__).parents[2] / "PX4-Autopilot")).resolve()
CONTAINER = "px4-safety-sitl"
BIN = "/px4/build/px4_sitl_default/bin"

# Run px4 directly rather than through `make px4_sitl gz_x500`: its -d flag
# detaches from stdin, and a stale parameter file would leak state between runs.
#
# Gazebo is started here rather than by PX4, which gives it only 30 s. A CI
# runner with no GPU renders in software and can take longer; PX4 then finds
# the running world and uses it.
START = """
cd /px4/build/px4_sitl_default/rootfs
rm -f parameters*.bson eeprom/* dataman
. ./gz_env.sh
gz sim --verbose=1 -r -s "$PX4_GZ_WORLDS/default.sdf" &
[ -n "$HEADLESS" ] || gz sim -g > /dev/null 2>&1 &
for i in $(seq 1 180); do
    if gz service -i --service /world/default/scene/info 2>&1 | grep -q "Service providers"; then
        echo "INFO  [sitl] Gazebo world up after $i s"
        exec ../bin/px4 -d
    fi
    sleep 1
done
echo "ERROR [sitl] Gazebo world not up after 180 s"
exit 1
"""


def _docker_base():
    return [
        "--network", "host",
        "--user", f"{os.getuid()}:{os.getgid()}",
        "-e", "HOME=/tmp",
        "-v", f"{PX4_DIR}:/px4",
        "-w", "/px4",
    ]


def build():
    subprocess.run(
        ["docker", "run", "--rm", *_docker_base(), IMAGE, "make", "px4_sitl_default"],
        check=True,
    )


def _display():
    """Docker flags that put the Gazebo window on this machine's X display."""
    xauth = os.environ.get("XAUTHORITY", str(Path.home() / ".Xauthority"))
    render_gid = os.stat("/dev/dri/renderD128").st_gid
    return [
        "-e", f"DISPLAY={os.environ['DISPLAY']}",
        "-e", "XAUTHORITY=/tmp/.Xauthority",
        "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
        "-v", f"{xauth}:/tmp/.Xauthority:ro",
        "--device", "/dev/dri",
        "--group-add", str(render_gid),
        "--group-add", "video",
    ]


def start(gui=False):
    stop()
    view = _display() if gui else ["-e", "HEADLESS=1"]
    subprocess.run(
        # --init: as PID 1, px4 would ignore the SIGSTOP that hang() sends it.
        ["docker", "run", "-d", "--init", "--name", CONTAINER, *_docker_base(), *view,
         "-e", "PX4_SYS_AUTOSTART=4001",
         "-e", "PX4_SIM_MODEL=gz_x500",
         "-e", "GZ_IP=127.0.0.1",
         IMAGE, "bash", "-c", START],
        check=True, stdout=subprocess.DEVNULL,
    )


def stop():
    subprocess.run(["docker", "rm", "-f", CONTAINER],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def logs():
    return subprocess.run(["docker", "logs", CONTAINER], capture_output=True, text=True).stdout


def px4_cmd(*args, check=True):
    cmd = ["docker", "exec", CONTAINER, f"{BIN}/px4-{args[0]}", *args[1:]]
    return subprocess.run(cmd, check=check, capture_output=True, text=True)


def hang():
    subprocess.run(["docker", "exec", CONTAINER, "pkill", "-STOP", "-x", "px4"], check=True)


def wait_ready(timeout_s=300):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        out = logs()
        if "Startup script returned successfully" in out or "Ready for takeoff" in out:
            return
        if "ERROR [sitl]" in out:
            break
        time.sleep(1)
    raise TimeoutError("PX4 did not start:\n" + logs()[-2000:])


if __name__ == "__main__":
    build()
