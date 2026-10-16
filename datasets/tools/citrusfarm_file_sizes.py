#!/usr/bin/env python3
"""Build datasets/citrusfarm_files.csv from the official dataset_file_list.yaml.

Usage: python -I citrusfarm_file_sizes.py <dataset_file_list.yaml> <out.csv>

Selects, for the six sequences used in the benchmark, every base_*.bag, the
ground-truth gt.bag/gt.csv and two calibration result files, then issues an
HTTP HEAD request to the official S3 bucket for each and records
Content-Length, ETag and Last-Modified. Nothing is downloaded.
"""
import csv
import sys
import urllib.request

import yaml

BASE_URL = "https://ucr-robotics.s3.us-west-2.amazonaws.com/citrus-farm-dataset"
SEQS = {  # paper id -> dataset folder (brief Section 4.1)
    "cf_2": "02_13B_Jackal",
    "cf_3": "03_13B_Jackal",
    "cf_4": "04_13D_Jackal",
    "cf_5": "05_13D_Jackal",
    "cf_6": "06_14B_Jackal",
    "cf_7": "07_14B_Jackal",
}
CALIB = ["04-lidar-gps-result.txt", "05-baselink-lidar-result.txt"]


def head(url):
    req = urllib.request.Request(url, method="HEAD")
    with urllib.request.urlopen(req, timeout=60) as r:
        return (int(r.headers["Content-Length"]), r.headers.get("ETag", "").strip('"'),
                r.headers.get("Last-Modified", ""))


def main(yaml_path, out_csv):
    with open(yaml_path) as f:
        tree = yaml.safe_load(f)["citrus-farm-dataset"]
    rows = []
    for sid, folder in SEQS.items():
        for name in sorted(tree[folder]):
            if name.startswith("base_") and name.endswith(".bag"):
                rows.append((sid, folder, name, tree[folder][name]["md5"]))
        gtf = f"ground_truth/{folder}"
        for name in ("gt.bag", "gt.csv"):
            rows.append((sid, gtf, name, tree[gtf][name]["md5"]))
    for name in CALIB:
        rows.append(("calib", "Calibration/results", name, tree["Calibration/results"][name]["md5"]))

    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "folder", "file", "url", "size_bytes", "md5_official", "s3_etag", "s3_last_modified"])
        for sid, folder, name, md5 in rows:
            url = f"{BASE_URL}/{folder}/{name}"
            size, etag, lm = head(url)
            w.writerow([sid, folder, name, url, size, md5, etag, lm])
            print(sid, folder, name, size, file=sys.stderr)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
