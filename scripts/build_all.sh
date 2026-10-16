#!/usr/bin/env bash
# Build every image in order: shared base, infrastructure, then the six methods
# (one image per configuration for the compile-time methods). Logs in env/build_logs/.
set -uo pipefail
cd "$(dirname "$0")/.."
PY="env -u PYTHONPATH .venv/bin/python"
echo "[$(date -u +%FT%TZ)] base"
docker build -t slambench/base:melodic env/docker/base > env/build_logs/base.log 2>&1 || { echo "base FAILED"; exit 1; }
echo "[$(date -u +%FT%TZ)] infra"
docker build -t slambench/infra:melodic -f env/docker/infra/Dockerfile env > env/build_logs/infra.log 2>&1 || echo "infra FAILED"
for m in dlo hdl mesh kiss aloam lego; do
  echo "[$(date -u +%FT%TZ)] $m"
  $PY scripts/build_images.py "$m" || echo "$m FAILED"
done
echo "[$(date -u +%FT%TZ)] done"
docker images --format '{{.Repository}}:{{.Tag}} {{.Size}}' | grep slambench
