# PX4 runtime safety monitor

A Simplex-style runtime monitor for a PX4 quadcopter, written for the Cortex-M4 of
an STM32MP157. PX4 does the flying and nobody verifies it. The monitor is small and
deterministic: it watches telemetry, and when a rule breaks it commands RTL, then
LAND, then flight termination. Scripted fault-injection flights check its reaction
on every pull request.

The process is DO-178C-inspired: requirements-based tests, traceability and
structural coverage, applied to a hobby project. It is not DO-178C compliant.

**Status:** the rules, the arbiter and the full fault campaign run against PX4 SITL
with the monitor built as a Linux process. The M4 firmware cross-compiles but has
not yet run on a board, so nothing here is an M4 timing result. See
[What has not been run](#what-has-not-been-run).

```
 PX4 SITL + Gazebo x500  <--UDP-->  bench proxy  <-->  monitor
                                    fault injection     rules -> arbiter -> command
                                    latency clock
```

The monitor is one set of sources with two transports: rpmsg-tty on the M4
(`src/main_zephyr.c`, `src/transport_rpmsg.c`) and UDP on the PC (`host/`). See
[docs/architecture.md](docs/architecture.md).

## Results

PX4 v1.16.2, monitor built for the PC, real-time factor between 0.999 and 1.0.
Latency runs from fault onset to the command reaching the proxy.

| Scenario | Fault | Commands | Latency | Limit |
|---|---|---|---|---|
| SC-00 | none | none | | |
| SC-01 | mission leaves the fence | RTL | 30.0 ms | 200 ms |
| SC-02 | attitude corrupted to 69° | LAND, TERMINATE | 40.9 ms | 200 ms |
| SC-03 | battery reported at 2 % | RTL | 12.0 ms | 500 ms |
| SC-04 | PX4 frozen with SIGSTOP | TERMINATE | 308.8 ms | 500 ms |
| SC-05 | telemetry dropped | TERMINATE | 309.8 ms | 500 ms |
| SC-06 | position jumps 100 m | TERMINATE | 25.6 ms | 200 ms |
| SC-07 | position stops updating | LAND | 991.8 ms | 1200 ms |
| SC-08 | PX4 never receives commands | RTL, LAND, TERMINATE | 1238.8 ms | 1400 ms |
| SC-09 | attitude is NaN | LAND, TERMINATE | 47.3 ms | 200 ms |
| SC-10 | GPS lost in Gazebo | none, PX4 lands itself | | |
| SC-11 | motor 2 cut in Gazebo | LAND, TERMINATE | 348.5 ms | |

The latencies come from one campaign on a desktop, so read them as a sanity check
and not as a bound. SC-11 has no limit because its onset is the moment the parameter
command returned, not the moment the airframe started to tumble.

Rules and arbiter have 100 % statement and decision coverage from the host unit
tests. MC/DC is not done. `docs/results/` has the raw numbers.

## Limitations

**Every override goes through PX4.** If PX4 is hung, a MAVLink command cannot save
the vehicle. SC-04 proves the monitor notices and sends the command, not that the
motors stop. A real Simplex design needs an independent path: the monitor cuts ESC
power through a relay, or switches a PWM multiplexer to its own simple controller.

**Losing the link looks like losing PX4.** In SC-05 PX4 is healthy, and the monitor
terminates it anyway because SR-04 cannot tell the two cases apart. On a real vehicle
that drops a working drone, so that requirement needs a second opinion first.

**Corrupt attitude terminates near the ground.** In SC-02 and SC-09 PX4 lands
normally, the bad data continues, and the envelope rule terminates at 5 m. That
follows SR-02 as written, but it is a 5 m drop caused by a sensor fault alone.

**An accepted command is trusted.** If PX4 acknowledges RTL and then does not
return, nothing escalates until the 30 m hard margin is crossed.

**The fence uses PX4's estimate.** After GPS loss PX4 keeps publishing a position, so
the fence is checked against dead reckoning (SC-10).

**Battery numbers are placeholders.** The speeds and drain rate behind SR-03 are
assumed for a simulated x500, and SC-03 fakes the reported level.

## What has not been run

- The firmware on a board. It builds for `stm32mp157c_dk2` at 39.7 KB of 64 KB RETRAM,
  but the rpmsg transport and `--board` mode in `run_scenario.py` have never executed.
- The GitHub Actions workflow. The `build-fw` west project list is untested too.
- The Yocto layer past a dry run. Both recipes parse, resolve against scarthgap and
  fetch their sources, but neither has been compiled or installed.
- Flying the fence breach by hand from QGroundControl. SC-01 uses a mission waypoint.

## Running it

Needs Docker, about 7 GB for the image and the PX4 build, and Python 3.

```sh
git clone --branch v1.16.2 --depth 1 --recursive --shallow-submodules \
    https://github.com/PX4/PX4-Autopilot.git ../PX4-Autopilot
# PX4's version script needs a NuttX tag, which a shallow clone leaves out.
git -C ../PX4-Autopilot/platforms/nuttx/NuttX/nuttx fetch --depth 1 origin tag nuttx-12.12.0

docker build -t px4-sitl:v1.16.2 -f tools/sitl/Dockerfile ../PX4-Autopilot/Tools/setup
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt matplotlib
tools/fetch_deps.sh
.venv/bin/python tools/sitl.py            # builds PX4

make -C monitor/tests test
make -C monitor/tests coverage GCOVR=$PWD/.venv/bin/gcovr
make -C monitor/host
.venv/bin/python tools/fly_mission.py
.venv/bin/python tools/run_scenario.py scenarios/SC-*.yaml --out results
```

Add `--gui` to `run_scenario.py` or `fly_mission.py` to watch in the Gazebo window. Keep
it off in CI: rendering adds load, and a run fails if the real-time factor drifts.

To see the PX4 to monitor exchange of one run, with message rates, commands, acks and
the monitor's state on one time axis:

```sh
.venv/bin/python tools/plot_timeline.py results/SC-04.jsonl sc04.png --window 2
```

The Docker image exists because the machine this was built on runs Debian, and PX4's
setup script wants Ubuntu. A full campaign takes about 12 minutes. Keep the machine
quiet while it runs, since load shows up as simulation stalls.

Firmware, with a Zephyr 4.4.2 workspace:

```sh
west build -b stm32mp157c_dk2 -d build-fw monitor
```

`stm32mp157c_dk2` is the only MP157 board Zephyr has. The DK1 is expected to work with
the same image.

The CI `sitl` and `hil` jobs expect a self-hosted PC runner with the image built and
PX4 built at `../PX4-Autopilot`, or at `PX4_DIR`.

## Layout

```
monitor/        rules, arbiter, Zephyr app, host/ build, tests/
a7/             Yocto layer: mavlink-router, pymavlink, bench proxy
tools/          mission flying, bench proxy, scenario runner, traceability check
scenarios/      one YAML per fault
docs/           requirements, traceability, architecture, results
```
