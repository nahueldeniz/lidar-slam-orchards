#!/usr/bin/env bash
# One-command demo: the six LiDAR SLAM methods of the paper on CitrusFarm sequence 06
# (LiDAR only, 302 s, real-time playback), evaluated with the paper's code and settings.
#
#   ./demo/run_demo.sh                       all six methods, about 45 min
#   METHODS="lego dlo" ./demo/run_demo.sh    a subset (make demo METHODS="lego dlo")
#   ./demo/run_demo.sh --check               prerequisites, images and data only; no runs
#
# Steps: 1 prerequisites, 2 images (registry name, local build tag or pull), 3 demo bag
# (official CitrusFarm file, resumable download, MD5 check), 4 one run per method with
# scripts/run_one.py --demo, 5 evaluation with scripts/evaluate_runs.py, 6 report
# (demo/report.py): <OUT>/results.csv, results.md and trajectories_<SEQ>.png/.pdf.
# Settings: demo/config.sh (every value can be overridden from the environment).
set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$(pwd)"
unset PYTHONPATH          # a ROS 2 (or other) PYTHONPATH breaks the evaluation environment
# shellcheck source=demo/config.sh
source demo/config.sh

CHECK_ONLY=0
case "${1:-}" in
  --check) CHECK_ONLY=1 ;;
  -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
  "") ;;
  *) echo "unknown option: $1 (use --check or --help)" >&2; exit 2 ;;
esac

log()  { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
warn() { printf '[%s] WARNING: %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
die()  { printf '[%s] ERROR: %s\n' "$(date +%H:%M:%S)" "$*" >&2; exit 1; }
free_gb() {   # free space (GB) on the filesystem holding $1 (or its nearest existing parent)
  local p="$1"
  while [ ! -e "$p" ]; do p="$(dirname "$p")"; done
  df -Pk "$p" 2>/dev/null | awk 'NR == 2 { printf "%.1f", $4 / 1e6 }'
}
gb_lt() { awk -v a="$1" -v b="$2" 'BEGIN { exit !(a < b) }'; }
abspath() { case "$1" in /*) printf '%s\n' "$1" ;; *) printf '%s\n' "$REPO/$1" ;; esac; }

T0=$(date +%s)
read -r -a METHOD_LIST <<< "$METHODS"
[ "${#METHOD_LIST[@]}" -gt 0 ] || die "METHODS is empty"

# ---------------------------------------------------------------- 1. prerequisites
log "1/6 prerequisites"
[ "$(uname -s)" = Linux ] || warn "tested on Linux only (the containers use --network host)"
command -v docker >/dev/null || die "docker not found: install Docker Engine (https://docs.docker.com/engine/install/)"
docker info >/dev/null 2>&1 || die "cannot reach the Docker daemon as user $(id -un). Start Docker, and allow your user to run docker without sudo (https://docs.docker.com/engine/install/linux-postinstall/)."
if (exec 3<>/dev/tcp/127.0.0.1/11311) 2>/dev/null; then
  die "port 11311 is in use (a ROS 1 master is already running?). Stop it: the demo starts its own roscore."
fi
[ -f "gt/$SEQ/gt_tum.txt" ] || warn "no ground truth gt/$SEQ/gt_tum.txt: trajectories are written but not scored"
ncpu=$(nproc)
mem_gb=$(awk '/MemAvailable/ { printf "%.1f", $2 / 1e6 }' /proc/meminfo)
[ "$ncpu" -ge 4 ] || warn "only $ncpu logical CPUs: the methods may fall behind real time"
gb_lt "$mem_gb" 2 && warn "only $mem_gb GB of memory available (SLAMesh peaks at 1.3 GB): some methods may fail or fall behind"
[ -f /sys/fs/cgroup/cgroup.controllers ] || warn "no cgroup v2: CPU and memory of the method container are not recorded (results are unaffected)"
log "  Docker $(docker version --format '{{.Server.Version}}' 2>/dev/null), $ncpu logical CPUs, $mem_gb GB memory available"

declare -A REG LOCAL IMG
while read -r key reg local _; do
  [[ -z "${key:-}" || "$key" == \#* ]] && continue
  REG[$key]="$reg"; LOCAL[$key]="$local"
done < "$IMAGES_CONF"
for m in "${METHOD_LIST[@]}"; do
  [ -n "${REG[$m]:-}" ] || die "unknown method '$m' (known: hdl lego aloam dlo kiss mesh)"
done

if [ ! -x .venv/bin/python ]; then
  command -v uv >/dev/null || die "no .venv and no uv: install uv (https://docs.astral.sh/uv/) and run 'make venv'"
  log "  creating the evaluation environment (.venv) from env/requirements-eval.lock.txt"
  make venv
fi

# ---------------------------------------------------------------- 2. images
log "2/6 images ($IMAGES_CONF)"
to_pull=()
for key in infra "${METHOD_LIST[@]}"; do
  if docker image inspect "${REG[$key]}" >/dev/null 2>&1; then
    IMG[$key]="${REG[$key]}"
  elif docker image inspect "${LOCAL[$key]}" >/dev/null 2>&1; then
    IMG[$key]="${LOCAL[$key]}"          # built locally with make images
  else
    IMG[$key]="${REG[$key]}"; to_pull+=("$key")
  fi
done
if [ "${#to_pull[@]}" -gt 0 ]; then
  root="$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo /)"
  free="$(free_gb "$root")"; [ -n "$free" ] || free="$(free_gb /)"
  gb_lt "$free" "$MIN_FREE_GB_IMAGES" && die "only $free GB free for Docker images ($root); pulling needs about $MIN_FREE_GB_IMAGES GB (MIN_FREE_GB_IMAGES)"
  for key in "${to_pull[@]}"; do
    log "  pulling ${REG[$key]}"
    docker pull "${REG[$key]}" || die "could not pull ${REG[$key]}; build the images instead (README, 'Rebuild the images')"
  done
fi
for key in infra "${METHOD_LIST[@]}"; do
  log "  $key: ${IMG[$key]} ($(docker image inspect --format '{{.Id}}' "${IMG[$key]}" | cut -c8-19))"
done

# ---------------------------------------------------------------- 3. data
log "3/6 data"
if [ -n "$BAG" ]; then
  BAG_ABS="$(abspath "$BAG")"
  [ -f "$BAG_ABS" ] || die "BAG=$BAG not found"
  log "  your bag: $BAG_ABS (no checksum)"
else
  DATA_ABS="$(abspath "$DEMO_DATA_DIR")"
  BAG_ABS="$DATA_ABS/$(basename "$DEMO_BAG_URL")"
  if [ ! -f "$BAG_ABS" ]; then
    part="$BAG_ABS.part"
    have=0; [ -f "$part" ] && have=$(stat -c %s "$part")
    need_gb=$(awk -v n="$DEMO_BAG_SIZE" -v h="$have" 'BEGIN { printf "%.1f", (n - h) / 1e9 + 1 }')
    free="$(free_gb "$DATA_ABS")"
    gb_lt "$free" "$need_gb" && die "only $free GB free in $DATA_ABS; the demo bag needs $need_gb GB"
    mkdir -p "$DATA_ABS"
    if [ "$have" -ne "$DEMO_BAG_SIZE" ]; then
      log "  downloading the CitrusFarm bag ($(awk -v n="$DEMO_BAG_SIZE" 'BEGIN { printf "%.2f", n / 1e9 }') GB; resumable: run again to continue)"
      log "  from $DEMO_BAG_URL"
      curl -fL --retry 5 --retry-delay 10 -C - -o "$part" "$DEMO_BAG_URL" \
        || die "download failed or interrupted; run the demo again to resume ($part)"
    fi
    size=$(stat -c %s "$part")
    [ "$size" -eq "$DEMO_BAG_SIZE" ] || die "$part has $size bytes, expected $DEMO_BAG_SIZE; delete it and run again"
    log "  checking MD5"
    md5=$(md5sum "$part" | cut -d' ' -f1)
    [ "$md5" = "$DEMO_BAG_MD5" ] || die "MD5 of $part is $md5, expected $DEMO_BAG_MD5 (official CitrusFarm value); delete it and run again"
    mv "$part" "$BAG_ABS"
  else
    size=$(stat -c %s "$BAG_ABS")
    [ "$size" -eq "$DEMO_BAG_SIZE" ] || die "$BAG_ABS has $size bytes, expected $DEMO_BAG_SIZE"
    if [ -z "$SKIP_CHECKSUM" ]; then
      log "  checking MD5 of $BAG_ABS"
      md5=$(md5sum "$BAG_ABS" | cut -d' ' -f1)
      [ "$md5" = "$DEMO_BAG_MD5" ] || die "MD5 of $BAG_ABS is $md5, expected $DEMO_BAG_MD5 (official CitrusFarm value)"
    fi
  fi
  log "  demo bag: $BAG_ABS (CitrusFarm 06_14B_Jackal, CC BY-SA 4.0; MD5 ok)"
fi

if [ "$CHECK_ONLY" = 1 ]; then
  log "check finished: prerequisites, images and data are ready (no runs made)"
  exit 0
fi

# ---------------------------------------------------------------- 4. runs
OUT_ABS="$(abspath "$OUT")"
if [ -e "$OUT_ABS" ]; then
  old="$OUT_ABS.$(date +%Y%m%dT%H%M%S)"
  mv "$OUT_ABS" "$old"
  log "previous output moved to $old"
fi
mkdir -p "$OUT_ABS/logs"
free="$(free_gb "$OUT_ABS")"
gb_lt "$free" "$MIN_FREE_GB_OUT" && die "only $free GB free for the outputs in $OUT_ABS"
on_signal() {
  warn "interrupted; stopping the benchmark containers"
  docker rm -f sb_method sb_infra >/dev/null 2>&1 || true
  exit 130
}
trap on_signal INT TERM

n=${#METHOD_LIST[@]}
log "4/6 runs: ${METHOD_LIST[*]} on $SEQ (real-time playback${DURATION:+, first $DURATION s only}; about 6 min each)"
failed=()
i=0
for m in "${METHOD_LIST[@]}"; do
  i=$((i + 1))
  log "  [$i/$n] $m (${IMG[$m]}); log: $OUT/logs/run_$m.log"
  args=(--demo --method "$m" --seq "$SEQ" --config D --rep 1 --bag "$BAG_ABS"
        --image "${IMG[$m]}" --infra-image "${IMG[infra]}" --out-root "$OUT_ABS/runs")
  [ -n "$DURATION" ] && args+=(--dry-run-seconds "$DURATION")
  set +e
  .venv/bin/python scripts/run_one.py "${args[@]}" > "$OUT_ABS/logs/run_$m.log" 2>&1
  rc=$?
  set -e
  if [ "$rc" -eq 0 ]; then
    log "  [$i/$n] $m finished"
  else
    failed+=("$m")
    warn "$m: run_one.py exit code $rc; last lines of the log:"
    tail -n 5 "$OUT_ABS/logs/run_$m.log" >&2
  fi
done
trap - INT TERM

# ---------------------------------------------------------------- 5. evaluation
log "5/6 evaluation (eval/slameval, alignment A, lever arm compensated)"
jobs=$(( n < ncpu ? n : ncpu ))
.venv/bin/python scripts/evaluate_runs.py --runs "$OUT_ABS/runs" --out "$OUT_ABS/eval" \
  --jobs "$jobs" --n-cpu "$ncpu" 2>&1 | tee "$OUT_ABS/logs/evaluate.log"

# ---------------------------------------------------------------- 6. report
log "6/6 report"
elapsed=$(( $(date +%s) - T0 ))
.venv/bin/python demo/report.py --out "$OUT_ABS" --seq "$SEQ" --methods "${METHOD_LIST[*]}" \
  --elapsed-s "$elapsed" ${DURATION:+--duration "$DURATION"} 2>&1 | tee "$OUT_ABS/logs/report.log"
log "done in $((elapsed / 60)) min $((elapsed % 60)) s; results: $OUT/results.md"
[ "${#failed[@]}" -eq 0 ] || { warn "runs that failed: ${failed[*]} (see $OUT/logs/)"; exit 1; }
