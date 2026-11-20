#!/usr/bin/env python3
"""
Fly scenarios from YAML files and judge the monitor's reaction.

    run_scenario.py scenarios/SC-*.yaml --out docs/results

Exits non-zero if any scenario fails. Writes junit.xml and latency.csv.
"""
import argparse
import csv
import json
import math
import shlex
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

import missions
import sitl

M = missions.M
ROOT = Path(__file__).resolve().parents[1]
ACTIONS = {
    "RTL": M.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    "LAND": M.MAV_CMD_NAV_LAND,
    "TERMINATE": M.MAV_CMD_COMPONENT_ARM_DISARM,
}
ACTION_NAMES = {v: k for k, v in ACTIONS.items()}
PROXY_FAULTS = {"corrupt_attitude", "nan_attitude", "low_battery", "freeze_position",
                "position_jump", "drop_telemetry", "ignore_commands"}

# Same fence as monitor/src/params.c, restated so the check is independent of
# the code it is judging.
FENCE_NORTH_M = 60.0
FENCE_LAT0_E7 = 473979711
MAX_RTF_DRIFT = 0.15
MIN_PEAK_ALT_M = 5.0

PX4_PARAMS = [
    ("COM_RC_IN_MODE", "4"),
    ("NAV_RCL_ACT", "0"),
    ("NAV_DLL_ACT", "0"),
    ("RTL_RETURN_ALT", "15"),
    ("RTL_DESCEND_ALT", "10"),
]


class Bench:
    """The proxy and the monitor, local or over SSH on the board."""

    def __init__(self, board, log_path):
        self.events = []
        self.log = open(log_path, "w")
        self.monitor = None
        proxy = ROOT / "tools" / "bench_proxy.py"
        if board:
            cmd = ["ssh", board, "python3 /opt/safety/bench_proxy.py "
                   f"--px4 {shlex.quote(board_px4_addr())}:14581 "
                   "--monitor serial:/dev/ttyRPMSG0"]
        else:
            binary = ROOT / "monitor" / "host" / "monitor_host"
            self.monitor = subprocess.Popen([str(binary), "14560"])
            cmd = [sys.executable, str(proxy)]
        self.proxy = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proxy.stdout:
            self.log.write(line)
            self.events.append(json.loads(line))

    def _request(self, **fields):
        self.proxy.stdin.write(json.dumps(fields) + "\n")
        self.proxy.stdin.flush()

    def fault(self, name):
        self._request(fault=name)

    def mark_injected(self):
        self._request(mark="inject")

    def close(self):
        for p in (self.proxy, self.monitor):
            if p is not None:
                p.terminate()
                p.wait()
        self.log.close()


def board_px4_addr():
    return subprocess.run(["hostname", "-I"], capture_output=True, text=True).stdout.split()[0]


def inject(fault, bench):
    if fault in PROXY_FAULTS:
        bench.fault(fault)
    elif fault == "px4_hang":
        sitl.hang()
    # `failure gps|motor off` do not reach the physics under Gazebo in PX4 v1.16:
    # gps is only handled by SimulatorMavlink, motor only fakes the ESC status.
    elif fault == "gps_off":
        sitl.px4_cmd("param", "set", "SIM_GPS_USED", "0")
    elif fault == "motor_off":
        sitl.px4_cmd("param", "set", "SIM_GZ_EC_MIN2", "0")
        sitl.px4_cmd("param", "set", "SIM_GZ_EC_MAX2", "0")
    else:
        raise ValueError(f"unknown fault {fault}")


def fault_onset(events, fault, t_inject):
    """When the condition the monitor has to catch actually began, on the proxy clock."""
    if fault in PROXY_FAULTS - {"ignore_commands"}:
        onset = [e["t"] for e in events if e["k"] == "onset"]
        if onset:
            return onset[0]
    if fault == "px4_hang":
        # Attitude keeps arriving until the SIGSTOP lands, so the last one is the start.
        first_cmd = next((e["t"] for e in events if e["k"] == "cmd"), math.inf)
        att = [e["t"] for e in events if e["k"] == "att" and e["t"] < first_cmd]
        if att:
            return att[-1]
    return t_inject


def fence_onset(events):
    for e in events:
        if e["k"] == "pos" and (e["lat"] - FENCE_LAT0_E7) * 0.011131949 > FENCE_NORTH_M:
            return e["t"]
    return None


def real_time_factor(events, until):
    att = [e for e in events if e["k"] == "att" and e["t"] < until]
    if len(att) < 100:
        return None
    return (att[-1]["boot_ms"] - att[0]["boot_ms"]) / 1000 / (att[-1]["t"] - att[0]["t"])


def judge(scenario, events, flight_error):
    """Returns a result dict; 'failure' is None on a pass."""
    # The proxy's clock, not ours: with --board the two are different machines.
    t_inject = next((e["t"] for e in events if e["k"] == "mark"), None)
    sid = scenario["id"]
    expect = scenario["expect"]
    fault = scenario.get("inject", {}).get("fault")
    wanted = expect["action"]
    cmds = [e for e in events if e["k"] == "cmd"]
    seen = [ACTION_NAMES[e["command"]] for e in cmds if e["confirmation"] == 0]
    result = dict(id=sid, requirement=scenario["requirement"], fault=fault or "none",
                  expected=wanted, limit_ms=expect.get("within_ms", ""), actions=" > ".join(seen),
                  latency_ms="", rtf="", ack="", failure=None)

    end = t_inject or (events[-1]["t"] if events else 0)
    rtf = real_time_factor(events, end)
    if rtf is not None:
        result["rtf"] = round(rtf, 3)
    if flight_error:
        result["failure"] = f"{sid} failed: flight error: {flight_error}"
        return result
    if rtf is None or abs(rtf - 1) > MAX_RTF_DRIFT:
        result["failure"] = f"{sid} failed: real-time factor {rtf} outside 1 +/- {MAX_RTF_DRIFT}"
        return result

    # A scenario that never left the ground proves nothing, least of all "no override".
    peak = max((e["alt_mm"] for e in events if e["k"] == "pos"), default=0) / 1000
    if peak < MIN_PEAK_ALT_M:
        result["failure"] = f"{sid} failed: vehicle only reached {peak:.1f} m"
        return result

    if wanted == "NONE":
        if cmds:
            result["failure"] = f"{sid} failed: false positive, monitor sent {seen}"
        return result

    if fault == "px4_hang" and any(e["k"] == "att" and e["t"] > t_inject + 0.2 for e in events):
        result["failure"] = f"{sid} failed: PX4 kept publishing after the hang was injected"
        return result

    match = next((e for e in cmds if e["command"] == ACTIONS[wanted]), None)
    if match is None or (wanted == "TERMINATE" and match["param2"] != 21196):
        limit = expect.get("within_ms")
        where = f" within {limit} ms" if limit else ""
        result["failure"] = f"{sid} failed: no {wanted}{where} (saw: {seen or 'nothing'})"
        return result

    if expect.get("from") == "fence_exit":
        onset = fence_onset(events)
    else:
        onset = fault_onset(events, fault, t_inject)
    if onset is None:
        result["failure"] = f"{sid} failed: fault onset not observed"
        return result

    latency = (match["t"] - onset) * 1000
    result["latency_ms"] = round(latency, 1)
    limit = expect.get("within_ms")
    if limit is not None and latency > limit:
        result["failure"] = f"{sid} failed: no {wanted} within {limit} ms (took {latency:.0f} ms)"
    if latency < 0:
        result["failure"] = f"{sid} failed: {wanted} sent {-latency:.0f} ms before the fault"
    acks = [e for e in events if e["k"] == "ack" and e["command"] == ACTIONS[wanted]]
    result["ack"] = acks[0]["result"] if acks else "none"
    if expect.get("acked") and result["ack"] != M.MAV_RESULT_ACCEPTED and not result["failure"]:
        result["failure"] = f"{sid} failed: PX4 did not accept {wanted} (ack {result['ack']})"
    return result


def run_one(scenario, out_dir, board, gui=False):
    sid = scenario["id"]
    inj = scenario.get("inject")
    sitl.start(gui=gui)
    bench = Bench(board, out_dir / f"{sid}.jsonl")
    t_inject = None
    flight_error = None
    try:
        sitl.wait_ready()
        if board:
            sitl.px4_cmd("mavlink", "start", "-x", "-u", "14581", "-r", "4000000", "-f",
                         "-m", "onboard", "-t", board.split("@")[-1], "-o", "14540")
        for name, value in PX4_PARAMS + list(scenario.get("params", {}).items()):
            sitl.px4_cmd("param", "set", name, str(value))

        link = missions.connect()
        home = missions.wait_home(link)
        missions.upload(link, home, missions.MISSIONS[scenario["mission"]])
        missions.arm_and_start(link)
        started = time.monotonic()

        def tick():
            nonlocal t_inject
            if inj and t_inject is None and time.monotonic() - started >= inj["at_s"]:
                inject(inj["fault"], bench)
                t_inject = time.monotonic()
                bench.mark_injected()

        def settled():
            if t_inject is None or inj["fault"] not in ("px4_hang", "drop_telemetry"):
                return False
            # PX4 or the link is gone by design, so disarm will never be seen.
            return time.monotonic() - t_inject > 3

        deadline = started + scenario.get("timeout_s", 150)
        while time.monotonic() < deadline and not settled():
            if missions.wait_disarmed(link, 0.5, tick, min_flight_s=0) and \
                    time.monotonic() - started > 8:
                break
        else:
            if not settled():
                flight_error = f"not finished after {scenario.get('timeout_s', 150)} s"
        time.sleep(0.5)
    except subprocess.CalledProcessError as exc:
        flight_error = f"{' '.join(exc.cmd[3:])}: {exc.stdout}{exc.stderr}"
    except (missions.FlightError, TimeoutError) as exc:
        flight_error = str(exc)
    finally:
        bench.close()
        (out_dir / f"{sid}.px4.log").write_text(sitl.logs())
        sitl.stop()

    return judge(scenario, bench.events, flight_error)


def write_reports(results, out_dir):
    fields = ["id", "requirement", "fault", "expected", "actions", "latency_ms", "limit_ms",
              "rtf", "ack"]
    with open(out_dir / "latency.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(results)

    suite = ET.Element("testsuite", name="fault-injection", tests=str(len(results)),
                       failures=str(sum(bool(r["failure"]) for r in results)))
    for r in results:
        case = ET.SubElement(suite, "testcase", classname=r["requirement"], name=r["id"])
        if r["failure"]:
            ET.SubElement(case, "failure", message=r["failure"]).text = r["failure"]
    ET.ElementTree(suite).write(out_dir / "junit.xml", encoding="unicode")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("scenarios", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=ROOT / "results")
    ap.add_argument("--board", help="user@ip of the MP157; default is the host build of the monitor")
    ap.add_argument("--gui", action="store_true",
                    help="show Gazebo; rendering adds load, so keep CI runs headless")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    results = []
    for path in sorted(args.scenarios):
        scenario = yaml.safe_load(path.read_text())
        result = run_one(scenario, args.out, args.board, args.gui)
        results.append(result)
        status = result["failure"] or (
            f"{result['id']} ok: {result['actions'] or 'no override'}"
            + (f" in {result['latency_ms']} ms" if result["latency_ms"] != "" else ""))
        print(status, flush=True)
        write_reports(results, args.out)

    return 1 if any(r["failure"] for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
