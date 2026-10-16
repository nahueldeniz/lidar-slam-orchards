#!/usr/bin/env python3
"""Run schedules for Plan A (decided 12 October 2026; brief Sections 5.0, 6).

  rep1         every test configuration (configs/<method>/configs.yaml 'test')
               on every enabled test sequence, repetition 1
  sensitivity  every one-at-a-time configuration once on its sensitivity
               sequence (cf_5), plus D three times on cf_5 (run-to-run spread)
  rep2, rep3   repetitions 2 and 3 of the test configurations
  pullally     D once on every enabled Pullally sequence

Runs are shuffled within each block with a fixed seed. Batch 1 is rep1; after
its results are checked (brief Section 6, 'Order'), batch 2 runs sensitivity,
rep2, rep3 and pullally. Each output line is: method seq config rep
"""
import argparse
import random
from pathlib import Path

import yaml

R = Path(__file__).resolve().parents[1]
SEED = 20261012
METHODS = ["hdl", "lego", "aloam", "dlo", "kiss", "mesh"]


def block(kind, seqs):
    cfg = {m: yaml.safe_load((R / "configs" / m / "configs.yaml").read_text()) for m in METHODS}
    test = [s for s, v in seqs.items() if v["role"] == "test" and v.get("enabled", True)
            and v["dataset"] == "citrusfarm"]
    if kind.startswith("rep"):
        r = int(kind[3:])
        return [(m, s, c, r) for m in METHODS for c in cfg[m]["test"] for s in test]
    if kind == "sensitivity":
        runs = []
        for m in METHODS:
            seq = yaml.safe_load((R / "configs" / m / "sensitivity.yaml").read_text())["sequence"]
            runs += [(m, seq, c, 1) for c in cfg[m]["oat"]]
            runs += [(m, seq, "D", r) for r in (1, 2, 3)]
        return runs
    if kind == "baselines":   # short power baselines (5 min), one per enabled sequence
        en = [s for s, v in seqs.items() if v.get("enabled", True)]
        return [("baseline", s, k, 1) for s in en for k in ("idle", "infra")]
    if kind == "pullally":
        pl = [s for s, v in seqs.items() if v["dataset"] == "pullally" and v.get("enabled", True)]
        return [(m, s, "D", 1) for m in METHODS for s in pl]
    raise ValueError(kind)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("batch", choices=["batch1", "batch2"])
    ap.add_argument("--redo-file", help="batch 2: 'method seq config rep' lines to redo from the internal copy")
    args = ap.parse_args()
    redo = []
    if args.redo_file and Path(args.redo_file).exists():
        redo = [l.split() for l in Path(args.redo_file).read_text().splitlines() if l.strip()]
    seqs = yaml.safe_load((R / "datasets" / "sequences.yaml").read_text())["sequences"]
    out = R / "schedules" / f"{args.batch}.txt"
    out.parent.mkdir(exist_ok=True)
    if args.batch == "batch1":
        runs = block("rep1", seqs)
        random.Random(SEED).shuffle(runs)
        out.write_text("".join(f"{m} {s} {c} {r}\n" for m, s, c, r in runs))
        print(f"{out.relative_to(R)}: {len(runs)} runs")
        return
    # Batch 2 (13 October). Every sequence is copied once to the internal disk
    # (the USB drive reset its link mid-run), used for every run that needs it,
    # and deleted: power baselines and repetitions 2 and 3 of every test
    # configuration for each test sequence; baselines and all sensitivity runs
    # for cf_5; baselines and the D runs for Pullally (last, so there is time to
    # free disk space for it). Runs are shuffled within each repetition of each
    # sequence; the order of the test sequences is shuffled.
    text, n = [], 0

    def group(seq, run_blocks, seed):
        nonlocal n
        text.extend([f"stage {seq}", f"baseline {seq} idle 1", f"baseline {seq} infra 1"])
        n += 2
        for m, s_, c, r in [x for x in redo if x[1] == seq]:
            text.append(f"redo {m} {s_} {c} {r}")
            n += 1
        for j, runs in enumerate(run_blocks):
            runs = list(runs)
            random.Random(seed + j).shuffle(runs)
            text.extend(f"{m} {s} {c} {r}" for m, s, c, r in runs)
            n += len(runs)
        text.append(f"unstage {seq}")

    rep2, rep3 = block("rep2", seqs), block("rep3", seqs)
    order = sorted({s for _, s, _, _ in rep2})
    random.Random(SEED + 102).shuffle(order)
    for i, s in enumerate(order):
        group(s, [[x for x in rep2 if x[1] == s], [x for x in rep3 if x[1] == s]], SEED + 10 * i)
    group("cf_5", [block("sensitivity", seqs)], SEED + 200)
    pl = block("pullally", seqs)
    for s in sorted({x[1] for x in pl}):
        group(s, [[x for x in pl if x[1] == s]], SEED + 300)
    out.write_text("".join(x + "\n" for x in text))
    print(f"{out.relative_to(R)}: {n} runs and baselines, {sum(1 for x in text if x.startswith('stage'))} copy steps")


if __name__ == "__main__":
    main()
