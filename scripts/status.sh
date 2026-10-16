#!/usr/bin/env bash
# Campaign status at a glance (safe to run at any time; reads files only).
#   bash scripts/status.sh
cd "$(dirname "$0")/.."
echo "== $(date '+%F %T')"
if tmux has-session -t slambench 2>/dev/null; then echo "campaign: RUNNING (tmux session 'slambench')"; else echo "campaign: NOT RUNNING"; fi
tail -2 logs/campaign.log 2>/dev/null
L="$(ls -t logs/batch_*.log 2>/dev/null | head -1)"
[ -n "$L" ] && { echo "current batch log: $L"; grep -E "^\[20..-..-.. ..:..:..\] [0-9]+/[0-9]+" "$L" | tail -1; }
echo "runs finished: $(ls runs/*/*/*/rep[0-9]*/manifest.json 2>/dev/null | grep -v _failed_ | wc -l)"
for s in ok infra_fault failed; do
  n=$(grep -l "\"status\": \"$s\"" runs/*/*/*/rep[0-9]/manifest.json 2>/dev/null | wc -l); echo "  $s: $n"
done
echo "retried after a playback fault: $(ls -d runs/*/*/*/rep*_failed_* 2>/dev/null | wc -l)"
echo "containers: $(docker ps --format '{{.Names}}:{{.Image}}' | tr '\n' ' ')"
if [ -r /sys/class/powercap/intel-rapl:0/energy_uj ]; then echo "power (RAPL): readable"; else echo "power (RAPL): NOT readable - runs have no power data"; fi
