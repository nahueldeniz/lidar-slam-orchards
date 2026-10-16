#!/usr/bin/env bash
# Campaign watchdog: checks every 5 min (reads files only, lowest priority) and
# exits with a reason as soon as something needs attention, or after
# HEARTBEAT_S as a progress heartbeat. Usage: bash scripts/watchdog.sh [HEARTBEAT_S]
cd "$(dirname "$0")/.."
HEARTBEAT_S=${1:-14400}
t_start=$(date +%s)
# Runs already reviewed (e.g. genuine method failures) are listed in logs/watchdog_ack.txt.
seen_bad=logs/watchdog_ack.txt; touch "$seen_bad"
report() { echo "WATCHDOG $(date '+%F %T'): $*"; bash scripts/status.sh; exit 0; }
while true; do
  now=$(date +%s)
  tmux has-session -t slambench 2>/dev/null || report "campaign session ended ($(tail -1 logs/campaign.log))"
  # Only the current campaign (the log is appended to across restarts).
  awk '/campaign start/{b=""} {b=b $0 "\n"} END{printf "%s", b}' logs/campaign.log 2>/dev/null \
    | grep -q "STOPPED" && report "campaign STOPPED"
  # A run whose final status is not ok (the retry renames a faulted run within ~2 min).
  bad=$(find runs baselines -path '*/rep[0-9]/manifest.json' -mmin +10 2>/dev/null \
        | xargs -r grep -lE '"status": "(infra_fault|failed)"' 2>/dev/null | sort)
  newbad=$(comm -13 <(sort "$seen_bad") <(echo "$bad" | grep -v '^$'))
  [ -n "$newbad" ] && report "run(s) ended not ok: $newbad"
  L="$(ls -t logs/batch_*.log 2>/dev/null | head -1)"
  if [ -n "$L" ]; then
    age=$(( now - $(stat -c %Y "$L") ))
    [ $age -gt 5400 ] && report "no progress in $L for $((age/60)) min"
    n_wait=$(tail -4 "$L" | grep -c "not started (pre-check)")
    [ "$n_wait" -ge 3 ] && report "runs keep failing the pre-check: $(grep -h 'NOT STARTED' "$L" | tail -1)"
  fi
  link=$(env -u PYTHONPATH nice -n 19 .venv/bin/python -c "
import sys, yaml; sys.path.insert(0, 'scripts'); from run_one import data_link
l = data_link(yaml.safe_load(open('datasets/sequences.yaml'))); print(int(l.get('usb_speed_mbps') or 0) if l.get('mounted') else -1)" 2>/dev/null)
  [ "${link:-0}" -lt 5000 ] && report "data drive link problem (speed ${link} Mbit/s; -1 = not mounted)"
  free_gb=$(df -BG --output=avail / | tail -1 | tr -dc 0-9)
  [ "$free_gb" -lt 5 ] && report "internal disk nearly full: ${free_gb} GB"
  [ $(( now - t_start )) -ge $HEARTBEAT_S ] && report "heartbeat"
  sleep 300
done
