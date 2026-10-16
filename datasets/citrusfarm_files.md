# CitrusFarm: files used in the benchmark

Generated on 12 October 2026 from the official file list and HTTP `HEAD` requests to the official AWS S3 bucket. The machine-readable version is `datasets/citrusfarm_files.csv`, built by `datasets/tools/citrusfarm_file_sizes.py`.

## Sources

- **Official download script** (dataset Tools and Download pages): [`scripts/download_citrusfarm.py`](https://raw.githubusercontent.com/UCR-Robotics/Citrus-Farm-Dataset/main/scripts/download_citrusfarm.py), in [UCR-Robotics/Citrus-Farm-Dataset](https://github.com/UCR-Robotics/Citrus-Farm-Dataset).
  - Pinned copy: [commit `0e58efe7fe3359ccd8326dff6b2d20f21b644cf6`](https://github.com/UCR-Robotics/Citrus-Farm-Dataset/blob/0e58efe7fe3359ccd8326dff6b2d20f21b644cf6/scripts/download_citrusfarm.py), the head of `main` on 12 October 2026 (last commit 26 December 2023).
  - A local copy is in `datasets/citrusfarm/download_script/`, with `dataset_file_list.yaml` (official MD5s) and the website's Markdown pages at the same commit. It was read but not run.
- **What the official script does:** it downloads `https://ucr-robotics.s3.us-west-2.amazonaws.com/citrus-farm-dataset/<folder>/<file>` for every file in `dataset_file_list.yaml`, which it fetches from the `main` branch at run time, so it is not pinned. It filters files by modality prefix (`adk`, `base`, `blackfly`, `mapir`, `odom`, `zed`), skips files that already exist without checking their size, and then verifies MD5s. By default it fetches every sequence and modality (about 1.3 TB) into the current directory.
- **Our replacement:** `datasets/download_citrusfarm.sh` uses the same base URL and the official MD5s, but fetches only the files listed below.
- **Cross-check:** an S3 `ListObjectsV2` listing of each folder agrees with `dataset_file_list.yaml`. It has the same `base_*.bag` file names and the same sizes as the `HEAD` requests, and no other `base_*` files exist in these six folders.
- **Extra CSV columns:** besides the requested columns, the CSV also has `md5_official` (from the YAML), `s3_etag` and `s3_last_modified`. An S3 ETag with a `-N` suffix is a multipart upload, so it is not an MD5.

## Files

URL prefix `…` = `https://ucr-robotics.s3.us-west-2.amazonaws.com/citrus-farm-dataset/`. Sizes are in bytes, with decimal GB/MB in brackets.

| seq | folder | file | url | size_bytes |
|---|---|---|---|---:|
| cf_2 | 02_13B_Jackal | base_2023-07-18-14-51-21_0.bag | …02_13B_Jackal/base_2023-07-18-14-51-21_0.bag | 3,528,586,240 (3.529 GB) |
| cf_2 | ground_truth/02_13B_Jackal | gt.bag | …ground_truth/02_13B_Jackal/gt.bag | 3,445,209 (3.4 MB) |
| cf_2 | ground_truth/02_13B_Jackal | gt.csv | …ground_truth/02_13B_Jackal/gt.csv | 411,627 (0.4 MB) |
| cf_3 | 03_13B_Jackal | base_2023-07-16-19-17-20_0.bag | …03_13B_Jackal/base_2023-07-16-19-17-20_0.bag | 4,248,066,076 (4.248 GB) |
| cf_3 | 03_13B_Jackal | base_2023-07-16-19-26-21_1.bag | …03_13B_Jackal/base_2023-07-16-19-26-21_1.bag | 4,248,072,492 (4.248 GB) |
| cf_3 | 03_13B_Jackal | base_2023-07-16-19-35-22_2.bag | …03_13B_Jackal/base_2023-07-16-19-35-22_2.bag | 4,248,068,845 (4.248 GB) |
| cf_3 | 03_13B_Jackal | base_2023-07-16-19-44-22_3.bag | …03_13B_Jackal/base_2023-07-16-19-44-22_3.bag | 670,151,061 (0.670 GB) |
| cf_3 | ground_truth/03_13B_Jackal | gt.bag | …ground_truth/03_13B_Jackal/gt.bag | 13,199,753 (13.2 MB) |
| cf_3 | ground_truth/03_13B_Jackal | gt.csv | …ground_truth/03_13B_Jackal/gt.csv | 1,585,247 (1.6 MB) |
| cf_4 | 04_13D_Jackal | base_2023-07-18-14-25-34_0.bag | …04_13D_Jackal/base_2023-07-18-14-25-34_0.bag | 4,247,590,276 (4.248 GB) |
| cf_4 | 04_13D_Jackal | base_2023-07-18-14-34-05_1.bag | …04_13D_Jackal/base_2023-07-18-14-34-05_1.bag | 2,140,921,265 (2.141 GB) |
| cf_4 | ground_truth/04_13D_Jackal | gt.bag | …ground_truth/04_13D_Jackal/gt.bag | 5,952,553 (6.0 MB) |
| cf_4 | ground_truth/04_13D_Jackal | gt.csv | …ground_truth/04_13D_Jackal/gt.csv | 726,777 (0.7 MB) |
| cf_5 | 05_13D_Jackal | base_2023-07-16-17-05-13_0.bag | …05_13D_Jackal/base_2023-07-16-17-05-13_0.bag | 4,247,569,339 (4.248 GB) |
| cf_5 | 05_13D_Jackal | base_2023-07-16-17-13-40_1.bag | …05_13D_Jackal/base_2023-07-16-17-13-40_1.bag | 2,106,095,083 (2.106 GB) |
| cf_5 | ground_truth/05_13D_Jackal | gt.bag | …ground_truth/05_13D_Jackal/gt.bag | 5,874,581 (5.9 MB) |
| cf_5 | ground_truth/05_13D_Jackal | gt.csv | …ground_truth/05_13D_Jackal/gt.csv | 717,162 (0.7 MB) |
| cf_6 | 06_14B_Jackal | base_2023-07-18-14-29-35_0.bag | …06_14B_Jackal/base_2023-07-18-14-29-35_0.bag | 2,486,606,484 (2.487 GB) |
| cf_6 | ground_truth/06_14B_Jackal | gt.bag | …ground_truth/06_14B_Jackal/gt.bag | 2,346,213 (2.3 MB) |
| cf_6 | ground_truth/06_14B_Jackal | gt.csv | …ground_truth/06_14B_Jackal/gt.csv | 285,205 (0.3 MB) |
| cf_7 | 07_14B_Jackal | base_2023-07-16-18-31-20_0.bag | …07_14B_Jackal/base_2023-07-16-18-31-20_0.bag | 4,247,820,862 (4.248 GB) |
| cf_7 | 07_14B_Jackal | base_2023-07-16-18-39-54_1.bag | …07_14B_Jackal/base_2023-07-16-18-39-54_1.bag | 4,171,006,106 (4.171 GB) |
| cf_7 | ground_truth/07_14B_Jackal | gt.bag | …ground_truth/07_14B_Jackal/gt.bag | 7,840,533 (7.8 MB) |
| cf_7 | ground_truth/07_14B_Jackal | gt.csv | …ground_truth/07_14B_Jackal/gt.csv | 953,302 (1.0 MB) |
| calib | Calibration/results | 04-lidar-gps-result.txt | …Calibration/results/04-lidar-gps-result.txt | 421 |
| calib | Calibration/results | 05-baselink-lidar-result.txt | …Calibration/results/05-baselink-lidar-result.txt | 304 |

## Totals

| seq | role | base bags | base_*.bag (bytes) | gt.bag + gt.csv (bytes) | total (bytes) | total (GB) |
|---|---|---:|---:|---:|---:|---:|
| cf_2 | test | 1 | 3,528,586,240 | 3,856,836 | 3,532,443,076 | 3.532 |
| cf_3 | test | 4 | 13,414,358,474 | 14,785,000 | 13,429,143,474 | 13.429 |
| cf_4 | tuning | 2 | 6,388,511,541 | 6,679,330 | 6,395,190,871 | 6.395 |
| cf_5 | tuning | 2 | 6,353,664,422 | 6,591,743 | 6,360,256,165 | 6.360 |
| cf_6 | test | 1 | 2,486,606,484 | 2,631,418 | 2,489,237,902 | 2.489 |
| cf_7 | test | 2 | 8,418,826,968 | 8,793,835 | 8,427,620,803 | 8.428 |
| calib | – | – | – | – | 725 | 0.000 |
| **Tuning (cf_4 + cf_5)** | | 4 | 12,742,175,963 | 13,271,073 | 12,755,447,036 | 12.755 |
| **Test (cf_2, 3, 6, 7)** | | 8 | 27,848,378,166 | 29,067,089 | 27,877,445,255 | 27.877 |
| **Overall** | | **12** | **40,590,554,129** | **43,338,162** | **40,633,893,016** | **40.634** |

- `small` mode of `download_citrusfarm.sh` (ground truth + calibration): 43,338,887 bytes (43.3 MB). Downloaded on 12 October 2026; sizes and official MD5s match, and the SHA-256 sums are in `datasets/citrusfarm/SHA256SUMS`.
- `full` mode adds the 12 base bags: 40.59 GB (37.80 GiB). **Not downloaded:** free space was 12–14 GB (see `disk_budget.md`).
- The base-bag size of `cf_5` (6.35 GB) matches the "6.3 GB" that Nazate-Burgos et al. (2026, Table 5) give for CitrusFarm, which suggests their figure refers to the base bags of sequence 05 only.

## Open questions

1. The 2026 Pullally paper's CitrusFarm size (6.3 GB) appears to be the `cf_5` base bags only. This is an inference from the matching number, not a statement by the authors.
2. `download_citrusfarm.py` reads `dataset_file_list.yaml` from `main` at run time. If the upstream list changes, our CSV (pinned to commit `0e58efe`) may differ. Re-run `datasets/tools/citrusfarm_file_sizes.py` before a `full` download to confirm that nothing has changed.
