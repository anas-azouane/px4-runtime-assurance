"""Start and stop a clean headless PX4 + Gazebo instance in Docker."""
import os
import subprocess
import time
from pathlib import Path

IMAGE = os.environ.get("PX4_SITL_IMAGE", "px4-sitl:v1.16.2")
PX4_DIR = Path(os.environ.get("PX4_DIR", Path(__file__).resolve().parents[2] / "PX4-Autopilot"))
CONTAINER = "px4-safety-sitl"
BIN = "/px4/build/px4_sitl_default/bin"

# Run px4 directly rather than through `make px4_sitl gz_x500`: its -d flag
# detaches from stdin, and a stale parameter file would leak state between runs.
START = """
cd /px4/build/px4_sitl_default/rootfs
rm -f parameters*.bson eeprom/* dataman
exec ../bin/px4 -d
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


def start(log_path=None):
    stop()
    subprocess.run(
        # --init: as PID 1, px4 would ignore the SIGSTOP that hang() sends it.
        ["docker", "run", "-d", "--init", "--name", CONTAINER, *_docker_base(),
         "-e", "HEADLESS=1",
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
    """Run a PX4 shell command in the running instance, e.g. px4_cmd('param', 'set', ...)."""
    cmd = ["docker", "exec", CONTAINER, f"{BIN}/px4-{args[0]}", *args[1:]]
    return subprocess.run(cmd, check=check, capture_output=True, text=True)


def hang():
    subprocess.run(["docker", "exec", CONTAINER, "pkill", "-STOP", "-x", "px4"], check=True)


def wait_ready(timeout_s=120):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if "Startup script returned successfully" in logs() or "Ready for takeoff" in logs():
            return
        time.sleep(1)
    raise TimeoutError("PX4 did not start:\n" + logs()[-2000:])


if __name__ == "__main__":
    build()
