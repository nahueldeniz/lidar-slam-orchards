#!/usr/bin/env bash
# Plan A measured campaign, unattended: batch 1 (repetition 1), automatic check
# of repetition 1, then batch 2 (sensitivity, repetitions 2-3, Pullally) and the
# power baselines. Start it detached and leave the machine idle:
#   tmux new -d -s slambench scripts/launch_campaign.sh
# Progress: logs/campaign.log and logs/batch_*.log. Stops before batch 2 if the
# repetition-1 check finds a protocol fault (logs/check_rep1.md).
set -uo pipefail
cd "$(dirname "$0")/.."
PY="env -u PYTHONPATH .venv/bin/python"
mkdir -p logs
log() { echo "[$(date '+%F %T')] $*" | tee -a logs/campaign.log; }
log "campaign start"
$PY scripts/make_schedule.py batch1 >> logs/campaign.log 2>&1
$PY scripts/make_schedule.py batch2 >> logs/campaign.log 2>&1
log "batch 1"
$PY scripts/run_batch.py schedules/batch1.txt --phase test >> logs/campaign.log 2>&1
log "repetition-1 check"
if ! $PY scripts/check_rep1.py >> logs/campaign.log 2>&1; then
  log "STOPPED: protocol fault in repetition 1, see logs/check_rep1.md"
  exit 1
fi
log "batch 2"
$PY scripts/run_batch.py schedules/batch2.txt --phase test >> logs/campaign.log 2>&1
log "baselines"
$PY scripts/run_baselines.py >> logs/campaign.log 2>&1
log "campaign done"
