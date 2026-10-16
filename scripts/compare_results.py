#!/usr/bin/env python3
"""Compare regenerated result CSVs with the reference CSVs shipped in the package.

  compare_results.py <regenerated_dir> <reference_dir> [--rtol 1e-6] [--atol 1e-6]
                     [--row-atol COLUMN=VALUE:ATOL ...] [--file-atol FILE=ATOL ...]

Every *.csv in the reference folder must exist in the regenerated folder with the
same columns and the same number of rows, in the same order. Text cells must be
identical. Numeric cells must agree within |a - b| <= atol + rtol * |b|; NaN equals
NaN. The default tolerance covers the text precision of the <output>.tum files
(positions written with 1 um resolution). --row-atol gives rows whose COLUMN equals
VALUE a larger absolute tolerance (e.g. alignment=B:2e-3) and --file-atol a whole
file (e.g. segments.csv=1e-5); see the Makefile target check-results for why.
Prints one line per file and, for files that differ, the worst columns. Exit status
0 when every file agrees.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def compare(a, b, rtol, atol, row_atol=()):
    """(ok, summary, details) for regenerated frame a against reference frame b.
    row_atol: (column, value, atol) rules that raise the tolerance of matching rows."""
    if list(a.columns) != list(b.columns):
        return False, "columns differ", [f"only regenerated: {sorted(set(a.columns) - set(b.columns))}",
                                         f"only reference: {sorted(set(b.columns) - set(a.columns))}"]
    if len(a) != len(b):
        return False, f"rows differ ({len(a)} regenerated, {len(b)} reference)", []
    details, worst_abs, worst_rel = [], 0.0, 0.0
    ok = True
    tol = np.full(len(b), float(atol))
    for col, value, a_tol in row_atol:
        if col in b.columns:
            m = (b[col].astype(str) == value).to_numpy()
            tol[m] = np.maximum(tol[m], a_tol)
    for c in b.columns:
        x, y = a[c], b[c]
        xn, yn = pd.to_numeric(x, errors="coerce"), pd.to_numeric(y, errors="coerce")
        numeric = (xn.notna() | x.isna()).all() and (yn.notna() | y.isna()).all()
        if numeric:
            xv, yv = xn.to_numpy(float), yn.to_numpy(float)
            both_nan = np.isnan(xv) & np.isnan(yv)
            d = np.where(both_nan, 0.0, np.abs(xv - yv))
            d = np.where(np.isnan(d), np.inf, d)          # NaN on one side only
            rel = d / np.maximum(np.abs(yv), 1e-300)
            bad = d > tol + rtol * np.abs(np.nan_to_num(yv))
            worst_abs = max(worst_abs, float(np.max(d)) if len(d) else 0.0)
            finite_rel = rel[np.isfinite(rel) & (d > 0)]
            worst_rel = max(worst_rel, float(np.max(finite_rel)) if len(finite_rel) else 0.0)
            if bad.any():
                ok = False
                i = int(np.argmax(np.where(bad, d, -1)))
                details.append(f"{c}: {int(bad.sum())} cell(s) outside tolerance, e.g. row {i}: "
                               f"{xv[i]!r} vs {yv[i]!r}")
        else:
            xs, ys = x.fillna("<NA>").astype(str), y.fillna("<NA>").astype(str)
            neq = (xs != ys).to_numpy()
            if neq.any():
                ok = False
                i = int(np.argmax(neq))
                details.append(f"{c}: {int(neq.sum())} text cell(s) differ, e.g. row {i}: {xs.iloc[i]!r} vs {ys.iloc[i]!r}")
    summary = (f"{len(b)} rows x {len(b.columns)} columns; max |diff| {worst_abs:.3g}, "
               f"max relative diff {worst_rel:.3g}")
    return ok, summary, details


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("regenerated")
    ap.add_argument("reference")
    ap.add_argument("--rtol", type=float, default=1e-6)
    ap.add_argument("--atol", type=float, default=1e-6)
    ap.add_argument("--row-atol", action="append", default=[], metavar="COLUMN=VALUE:ATOL")
    ap.add_argument("--file-atol", action="append", default=[], metavar="FILE=ATOL")
    args = ap.parse_args()
    row_atol = []
    for r in args.row_atol:
        cv, a_tol = r.rsplit(":", 1)
        col, value = cv.split("=", 1)
        row_atol.append((col, value, float(a_tol)))
    file_atol = {k: float(v) for k, v in (x.split("=", 1) for x in args.file_atol)}
    if row_atol or file_atol:
        print(f"tolerance: rtol {args.rtol}, atol {args.atol}; rows {row_atol}; files {file_atol}")
    reg, ref = Path(args.regenerated), Path(args.reference)
    all_ok = True
    for f in sorted(ref.glob("*.csv")):
        g = reg / f.name
        if not g.exists():
            print(f"MISSING  {f.name}")
            all_ok = False
            continue
        b = pd.read_csv(f, keep_default_na=True, low_memory=False)
        a = pd.read_csv(g, keep_default_na=True, low_memory=False)
        identical = f.read_bytes() == g.read_bytes()
        ok, summary, details = compare(a, b, args.rtol, file_atol.get(f.name, args.atol), row_atol)
        tag = "IDENTICAL" if identical else ("MATCH" if ok else "DIFFER")
        print(f"{tag:9s} {f.name}: {'byte-identical; ' if identical else ''}{summary}")
        for line in details[:10]:
            print(f"          {line}")
        all_ok &= ok
    extra = sorted({p.name for p in reg.glob("*.csv")} - {p.name for p in ref.glob("*.csv")})
    if extra:
        print(f"note: regenerated only: {extra}")
    print("ALL MATCH" if all_ok else "DIFFERENCES FOUND")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
