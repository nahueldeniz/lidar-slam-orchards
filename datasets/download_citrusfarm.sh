#!/usr/bin/env bash
# Download the CitrusFarm files used in the benchmark (brief Section 4.1).
#
# Usage: datasets/download_citrusfarm.sh small|full
#   small : ground truth (gt.bag, gt.csv) for the six sequences + two calibration files (~42 MB)
#   full  : small + every base_*.bag of the six sequences (~40.6 GB); restrict the base bags
#           with ONLY_SEQS="cf_2 cf_3 cf_5 cf_6 cf_7" (Plan A: 34.2 GB)
#
# The file list, URLs, sizes and official MD5s come from datasets/citrusfarm_files.csv,
# which datasets/tools/citrusfarm_file_sizes.py builds from the official
# dataset_file_list.yaml (UCR-Robotics/Citrus-Farm-Dataset @ 0e58efe) and S3 HEAD requests.
# This script replaces scripts/download_citrusfarm.py (copy kept in
# datasets/citrusfarm/download_script/) so that only the selected files are fetched.
#
# Layout:
#   ${SLAMBENCH_DATA:-datasets}/citrusfarm/raw/<folder>/base_*.bag
#   datasets/citrusfarm/ground_truth/<folder>/gt.bag, gt.csv
#   datasets/citrusfarm/calibration/04-lidar-gps-result.txt, 05-baselink-lidar-result.txt
# Set SLAMBENCH_DATA (as in datasets/sequences.yaml) to put the base bags on another
# drive; ground truth and calibration always stay in datasets/citrusfarm/.
#
# Behaviour:
#   - Resumable: each file is fetched to <name>.part with `curl -C -`, then renamed.
#   - Files already present with the expected size are not fetched again.
#   - Every file is checked against the CSV size and the official MD5.
#   - `full` refuses to start unless free space on the base-bag filesystem
#     >= bytes still to download + 15 GiB.
#   - Before each file, refuses to continue if the download would leave less than
#     MIN_FREE_GIB (12 GiB for small, 15 GiB for full) free on the target filesystem.
#   - SHA-256 of every file present is (re)written to datasets/citrusfarm/SHA256SUMS
#     (and to $SLAMBENCH_DATA/citrusfarm/SHA256SUMS for the base bags when it is set).
set -euo pipefail

MODE="${1:-}"
if [[ "$MODE" != "small" && "$MODE" != "full" ]]; then
  echo "Usage: $0 small|full" >&2
  exit 2
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CSV="$HERE/citrusfarm_files.csv"
OUT="$HERE/citrusfarm"
RAW_BASE="${SLAMBENCH_DATA:-$HERE}/citrusfarm"
GIB=$((1024 * 1024 * 1024))
if [[ "$MODE" == "full" ]]; then MIN_FREE_GIB=15; else MIN_FREE_GIB=12; fi

[[ -f "$CSV" ]] || { echo "Missing $CSV" >&2; exit 1; }
mkdir -p "$OUT"
if [[ "$MODE" == "full" ]]; then mkdir -p "$RAW_BASE"; RAW_BASE="$(cd "$RAW_BASE" && pwd)"; fi

# Map a CSV folder to the local destination directory.
dest_dir() {
  case "$1" in
    ground_truth/*)       echo "$OUT/$1" ;;
    Calibration/results)  echo "$OUT/calibration" ;;
    [0-9][0-9]_*)         echo "$RAW_BASE/raw/$1" ;;
    *) echo "Unknown folder '$1' in CSV" >&2; return 1 ;;
  esac
}

# Free bytes on the filesystem holding $1 (or its nearest existing parent).
free_bytes() { local p="$1"; while [[ ! -e "$p" ]]; do p="$(dirname "$p")"; done
               df -B1 --output=avail "$p" | tail -n 1 | tr -d ' '; }
local_size() { if [[ -f "$1" ]]; then stat -c %s "$1"; else echo 0; fi; }

# Read the selected rows (columns 1-6 contain no commas; later columns are ignored).
SEL=()
while IFS=, read -r seq folder file url size md5 _rest; do
  [[ "$seq" == "seq" ]] && continue
  if [[ "$MODE" == "small" && "$file" == base_*.bag ]]; then continue; fi
  # Optional filter for base bags, e.g. ONLY_SEQS="cf_2 cf_3 cf_5 cf_6 cf_7" (Plan A has no cf_4).
  if [[ -n "${ONLY_SEQS:-}" && "$file" == base_*.bag && " $ONLY_SEQS " != *" $seq "* ]]; then continue; fi
  SEL+=("$seq|$folder|$file|$url|$size|$md5")
done < "$CSV"

# Bytes still to download (complete files count as done; .part files count as partial).
total=0; remaining=0
for row in "${SEL[@]}"; do
  IFS='|' read -r seq folder file url size md5 <<< "$row"
  d="$(dest_dir "$folder")"
  total=$((total + size))
  have="$(local_size "$d/$file")"
  if [[ "$have" -eq "$size" ]]; then continue; fi
  part="$(local_size "$d/$file.part")"
  remaining=$((remaining + size - part))
done
if [[ "$MODE" == "full" ]]; then free="$(free_bytes "$RAW_BASE")"; else free="$(free_bytes "$OUT")"; fi
gb() { awk -v b="$1" 'BEGIN { printf "%.3f", b / 1e9 }'; }
echo "Mode $MODE: ${#SEL[@]} files, $(gb "$total") GB in total, $(gb "$remaining") GB still to download, $(gb "$free") GB free."

if [[ "$MODE" == "full" ]]; then
  need=$((remaining + 15 * GIB))
  if (( free < need )); then
    echo "Refusing to start: free space $(gb "$free") GB < still to download $(gb "$remaining") GB + 15 GiB." >&2
    exit 3
  fi
fi

for row in "${SEL[@]}"; do
  IFS='|' read -r seq folder file url size md5 <<< "$row"
  d="$(dest_dir "$folder")"
  mkdir -p "$d"
  f="$d/$file"
  have="$(local_size "$f")"
  if [[ "$have" -eq "$size" ]]; then
    echo "[skip] $folder/$file already complete ($size bytes)"
  else
    if [[ "$have" -ne 0 ]]; then
      echo "[error] $f exists with $have bytes, expected $size. Remove it and re-run." >&2
      exit 4
    fi
    part="$(local_size "$f.part")"
    if (( part > size )); then
      echo "[error] $f.part is larger than expected ($part > $size). Remove it and re-run." >&2
      exit 4
    fi
    free="$(free_bytes "$d")"
    if (( free - (size - part) < MIN_FREE_GIB * GIB )); then
      echo "[stop] Downloading $folder/$file would leave less than $MIN_FREE_GIB GiB free." >&2
      exit 3
    fi
    echo "[get ] $folder/$file ($size bytes)"
    curl --fail --location --progress-bar --retry 5 --retry-delay 5 -C - -o "$f.part" "$url"
    got="$(local_size "$f.part")"
    if [[ "$got" -ne "$size" ]]; then
      echo "[error] $f.part has $got bytes, expected $size. Re-run to resume." >&2
      exit 5
    fi
    mv "$f.part" "$f"
  fi
  sum="$(md5sum "$f" | cut -d' ' -f1)"
  if [[ "$sum" != "$md5" ]]; then
    echo "[error] MD5 mismatch for $f: $sum != $md5 (official). Delete it and re-run." >&2
    exit 6
  fi
  echo "[ ok ] size and official MD5 match"
done

# SHA-256 of everything downloaded so far (paths relative to the citrusfarm/ root).
write_sums() {  # root, subdirs...
  local root="$1"; shift; local dirs=()
  for x in "$@"; do [[ -d "$root/$x" ]] && dirs+=("$x"); done
  (( ${#dirs[@]} )) || return 0
  ( cd "$root" && find "${dirs[@]}" -type f ! -name '*.part' \
      | LC_ALL=C sort | xargs -r -d '\n' sha256sum > SHA256SUMS )
  echo "Wrote $root/SHA256SUMS ($(wc -l < "$root/SHA256SUMS") files)."
}
if [[ -d "$RAW_BASE" && "$(cd "$RAW_BASE" && pwd)" != "$(cd "$OUT" && pwd)" ]]; then
  write_sums "$OUT" ground_truth calibration
  write_sums "$RAW_BASE" raw
else
  write_sums "$OUT" raw ground_truth calibration
fi
df -h "$OUT" | tail -n 1
