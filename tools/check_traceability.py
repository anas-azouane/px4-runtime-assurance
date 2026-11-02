#!/usr/bin/env python3
"""
Fail if the requirements, tests and scenarios have drifted apart. Every
requirement needs code and at least one test, every test and scenario named
must exist, and every scenario and SR comment must point at a known requirement.
"""
import csv
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
TEST_SRC = ROOT / "monitor/tests/test_monitor.c"


def main():
    errors = []
    reqs = set(re.findall(r"^\| (SR-\d+) \|", (ROOT / "docs/requirements.md").read_text(), re.M))
    test_src = TEST_SRC.read_text()
    tests = set(re.findall(r"^static void (test_\w+)\(void\)", test_src, re.M))
    scenarios = {yaml.safe_load(p.read_text())["id"]: yaml.safe_load(p.read_text())
                 for p in (ROOT / "scenarios").glob("SC-*.yaml")}

    traced_tests, traced_scenarios, traced_reqs = set(), set(), set()
    with open(ROOT / "docs/traceability.csv", newline="") as f:
        for row in csv.DictReader(f):
            req = row["requirement"]
            traced_reqs.add(req)
            if req not in reqs:
                errors.append(f"traceability.csv: {req} is not in requirements.md")
            for ref in row["source"].split():
                path, _, func = ref.partition(":")
                source = ROOT / path
                if not source.exists():
                    errors.append(f"{req}: no such source file {path}")
                elif func and func not in source.read_text():
                    errors.append(f"{req}: {func} not found in {path}")
            row_tests = row["unit_tests"].split()
            row_scenarios = row["scenarios"].split()
            if not row_tests and not row_scenarios:
                errors.append(f"{req}: has no test")
            for test in row_tests:
                traced_tests.add(test)
                if test not in tests:
                    errors.append(f"{req}: no such unit test {test}")
            for sid in row_scenarios:
                traced_scenarios.add(sid)
                if sid not in scenarios:
                    errors.append(f"{req}: no such scenario {sid}")
                elif scenarios[sid]["requirement"] != req:
                    errors.append(f"{sid}: says {scenarios[sid]['requirement']}, trace says {req}")

    for req in sorted(reqs - traced_reqs):
        errors.append(f"{req}: not traced")
    for sid in sorted(set(scenarios) - traced_scenarios):
        errors.append(f"{sid}: scenario points to no requirement in the trace")
    for test in sorted(tests - traced_tests):
        errors.append(f"{test}: unit test points to no requirement")

    for e in errors:
        print(e, file=sys.stderr)
    print(f"{len(reqs)} requirements, {len(tests)} unit tests, {len(scenarios)} scenarios: "
          f"{len(errors)} problems")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
