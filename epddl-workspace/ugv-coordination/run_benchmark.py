#!/usr/bin/env python3
"""Ground every UGV-coordination instance with plank and solve it with aletheia.

Writes one row per instance to results.csv: instance parameters, the size of
the grounded task, the heuristic and strategy the planner selected, the search
effort, and wall-clock times.
"""

import csv
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
INST = os.path.join(HERE, "instances")
EXPORT = os.path.join(HERE, "export_out")
PLANS = os.path.join(HERE, "plans")

PLANK = os.path.expanduser("~/plank/build/plank")
LIBRARY = os.path.expanduser("~/plank/benchmarks/libraries/intermediate.epddl")
PLANNER = os.path.expanduser("~/Projects/aletheia/build/epistemic_planner")

GROUND_TIMEOUT = 300
PLAN_TIMEOUT = 60

FIELDS = [
    "instance", "domain", "agents", "bays", "candidates", "items", "placement", "goal",
    "atoms", "worlds", "designated", "ground_actions",
    "heuristic", "strategy", "depth", "expanded", "generated", "leaves", "branches",
    "ground_s", "plan_s", "status",
]

# Instance names are <domain>-<key><n>...-<placement>-<goal>, where each
# <key><n> token carries one integer parameter.
PARAM_KEYS = {"a": "agents", "b": "bays", "u": "candidates", "i": "items"}

RE_LOADED = re.compile(
    r"Loaded: (\d+) atoms, (\d+) agents, (\d+) worlds \((\d+) designated\), (\d+) actions")
RE_HEUR = re.compile(r"Heuristic: (\S+)")
RE_STRAT = re.compile(r"Strategy: (\S+)")
RE_SOL = re.compile(r"Solution found at depth (\d+)\s+Expanded=(\d+)\s+Generated=(\d+)")
RE_VALID = re.compile(r"validator\] OK — (\d+) leaves, (\d+) branches")
RE_GBFS = re.compile(r"Plan found.*length (\d+).*Expanded=(\d+)\s+Generated=(\d+)")


def parse_name(name):
    tok = name.split("-")
    out = {"domain": tok[0], "placement": tok[-2], "goal": tok[-1]}
    for t in tok[1:-2]:
        key = PARAM_KEYS.get(t[0])
        if key and t[1:].isdigit():
            out[key] = int(t[1:])
    return out


def run(cmd, timeout):
    t0 = time.perf_counter()
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout + p.stderr, time.perf_counter() - t0
    except subprocess.TimeoutExpired:
        return None, "", time.perf_counter() - t0


def load_existing():
    """Previous rows, so a partial re-run updates results.csv instead of truncating it."""
    path = os.path.join(HERE, "results.csv")
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        return {r["instance"]: {k: r.get(k, "") for k in FIELDS}
                for r in csv.DictReader(fh)}


def main():
    os.makedirs(EXPORT, exist_ok=True)
    os.makedirs(PLANS, exist_ok=True)

    # Optional prefix filter: `python3 run_benchmark.py directed facility`
    prefixes = tuple(sys.argv[1:])
    names = sorted(f[:-6] for f in os.listdir(INST) if f.endswith(".epddl"))
    if prefixes:
        names = [n for n in names if n.startswith(prefixes)]
    merged = load_existing()
    rows = []

    for k, name in enumerate(names, 1):
        row = dict.fromkeys(FIELDS, "")
        row["instance"] = name
        row.update(parse_name(name))

        domain = os.path.join(HERE, "domains", f"ugv-{row['domain']}.epddl")
        rc, _, gt = run([PLANK, "export", "-d", domain, "-p",
                         os.path.join(INST, f"{name}.epddl"),
                         "-l", LIBRARY, "-o", EXPORT], GROUND_TIMEOUT)
        row["ground_s"] = f"{gt:.3f}"
        if rc is None:
            row["status"] = "ground-timeout"
            rows.append(row)
            print(f"[{k}/{len(names)}] {name}: ground-timeout")
            continue
        if rc != 0:
            row["status"] = "ground-error"
            rows.append(row)
            print(f"[{k}/{len(names)}] {name}: ground-error")
            continue

        rc, out, pt = run([PLANNER,
                           "--task", os.path.join(EXPORT, f"{name}.json"),
                           "--plan", os.path.join(PLANS, f"{name}.json"),
                           "--explain"], PLAN_TIMEOUT)
        row["plan_s"] = f"{pt:.3f}"

        if m := RE_LOADED.search(out):
            (row["atoms"], _agents, row["worlds"],
             row["designated"], row["ground_actions"]) = m.groups()
        if m := RE_HEUR.search(out):
            row["heuristic"] = m.group(1)
        if m := RE_STRAT.search(out):
            row["strategy"] = m.group(1)
        if m := RE_SOL.search(out):
            row["depth"], row["expanded"], row["generated"] = m.groups()
        elif m := RE_GBFS.search(out):
            row["depth"], row["expanded"], row["generated"] = m.groups()
        if m := RE_VALID.search(out):
            row["leaves"], row["branches"] = m.groups()

        if rc is None:
            row["status"] = "plan-timeout"
        elif rc != 0:
            row["status"] = "plan-error"
        elif "validator] OK" in out:
            row["status"] = "solved"
        elif row["depth"]:
            row["status"] = "solved-unvalidated"
        else:
            row["status"] = "unsolved"

        rows.append(row)
        print(f"[{k}/{len(names)}] {name}: {row['status']} "
              f"depth={row['depth']} exp={row['expanded']} t={row['plan_s']}s")

    for r in rows:
        merged[r["instance"]] = r
    with open(os.path.join(HERE, "results.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(merged[k] for k in sorted(merged))
    print(f"ran {len(rows)}; results.csv now holds {len(merged)} rows")


if __name__ == "__main__":
    main()
