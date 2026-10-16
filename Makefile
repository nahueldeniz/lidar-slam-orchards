# Single entry point (README.md). Run from this directory. Paths may contain
# spaces, so every path here is relative.
#
# Demo (any Linux machine with Docker; README, "Quickstart"):
#   make venv                      evaluation environment from env/requirements-eval.lock.txt
#   make demo-check                prerequisites, images and the demo bag; no runs
#   make demo                      six methods on CitrusFarm 06, evaluated as in the paper (~45 min)
#   make demo METHODS="lego dlo"   a subset; also BAG=, SEQ=, OUT=, DURATION=, DEMO_DATA_DIR=
#                                  (demo/config.sh)
#   make test                      unit and pipeline tests
#
# Benchmark campaign (as run for the paper; needs the full datasets and days of machine time):
#   make all        build -> data -> inventory/gt -> tests -> runs -> evaluate -> figures -> tables
#   make results    evaluate -> figures -> tables from the raw runs already on disk
#
# Regenerating the paper's results needs the run outputs, which are in the Zenodo
# package (DOI 10.5281/zenodo.23405930), not in this repository. In that package:
#   make venv                      evaluation environment from env/requirements-eval.lock.txt
#   make test                      unit and pipeline tests
#   make results OUT=/tmp/regen    regenerate eval/*.csv, figures and tables from the
#                                  included run outputs into OUT (default: in place)
#   make check-results OUT=/tmp/regen   compare OUT/eval/*.csv and OUT/tables/*.csv with
#                                  the shipped eval/*.csv and tables/*.csv
#
# Measured runs (tuning, test, baselines) take days. Start them detached with
# the *-detached targets and leave the machine idle (brief Section 0, rule 6).

SHELL := /bin/bash
PY    := env -u PYTHONPATH .venv/bin/python
METHODS := hdl lego aloam dlo kiss mesh
# JOBS: evaluation worker processes. OUT: output folder for make results
# (empty = in place: eval/, figs/, tables/).
JOBS    ?= 8
OUT     ?=

.PHONY: all results venv machine images data-small data-full inventory gt test \
        configs schedules batch1 batch2 baselines batch1-detached batch2-detached \
        evaluate figures tables check-results clean-eval demo demo-check

all: images data-full inventory gt test configs batch1 batch2 results

results: evaluate figures tables

# ----------------------------------------------------------------- environment
venv: .venv/bin/python
.venv/bin/python:
	uv venv --python 3.10 .venv
	uv pip install --python .venv/bin/python -r env/requirements-eval.lock.txt

machine:
	$(PY) env/collect_machine.py

images:
	docker build -t slambench/base:melodic env/docker/base 2>&1 | tee env/build_logs/base.log
	docker build -t slambench/infra:melodic -f env/docker/infra/Dockerfile env 2>&1 | tee env/build_logs/infra.log
	$(PY) scripts/build_images.py $(METHODS)

# ----------------------------------------------------------------- demo
# Variables are passed on explicitly; empty ones take the defaults of demo/config.sh.
DEMO_ENV = METHODS="$(METHODS)" BAG="$(BAG)" SEQ="$(SEQ)" OUT="$(OUT)" DURATION="$(DURATION)" \
           DEMO_DATA_DIR="$(DEMO_DATA_DIR)" SKIP_CHECKSUM="$(SKIP_CHECKSUM)"

demo: venv
	$(DEMO_ENV) ./demo/run_demo.sh

demo-check: venv
	$(DEMO_ENV) ./demo/run_demo.sh --check

# ----------------------------------------------------------------- data
data-small:
	bash datasets/download_citrusfarm.sh small

data-full:   # Plan A: no cf_4
	ONLY_SEQS="cf_2 cf_3 cf_5 cf_6 cf_7" bash datasets/download_citrusfarm.sh full

inventory:
	$(PY) scripts/inventory.py

gt:
	$(PY) scripts/process_gt.py

test:
	cd eval && env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../.venv/bin/python -m pytest -q

# ----------------------------------------------------------------- runs (Plan A)
# Plan A (12 October 2026): configuration D on the test sequences (3 repetitions),
# hdl floor ablation, one-at-a-time sensitivity on cf_5, Pullally D once.
configs:
	$(PY) scripts/make_configs.py $(METHODS)
	$(PY) scripts/freeze_configs.py

schedules:
	$(PY) scripts/make_schedule.py batch1
	$(PY) scripts/make_schedule.py batch2

batch1: schedules
	$(PY) scripts/run_batch.py schedules/batch1.txt --phase test

batch2: schedules
	$(PY) scripts/run_batch.py schedules/batch2.txt --phase test
	$(PY) scripts/run_baselines.py

baselines:
	$(PY) scripts/run_baselines.py

batch1-detached: schedules
	tmux new -d -s slambench '$(MAKE) batch1'
	@echo "Batch 1 started in tmux session 'slambench'. Close every other application now."

batch2-detached: schedules
	tmux new -d -s slambench '$(MAKE) batch2'
	@echo "Batch 2 started in tmux session 'slambench'. Close every other application now."

# ----------------------------------------------------------------- results
evaluate:
	$(PY) scripts/evaluate_runs.py --jobs $(JOBS) $(if $(OUT),--out "$(OUT)/eval")

figures:
	$(PY) scripts/make_figures.py $(if $(OUT),--results-dir "$(OUT)/eval" --out-dir "$(OUT)/figs")

tables:
	$(PY) scripts/make_tables.py $(if $(OUT),--results-dir "$(OUT)/eval" --out-dir "$(OUT)/tables")

# Tolerances for results regenerated from <output>.tum (the package has no outputs.bag):
# the TUM files store positions to 1 um. Everything must agree within 1e-6 except
# alignment-B values (fitted on the first 10 m only and applied to the whole run, which
# amplifies the rounding to at most 2 mm) and per-segment fits (at most 1e-5 m). Tables
# T1-T7 as printed (rounded CSV and LaTeX) come out byte-identical. With the raw
# recordings (evaluate_runs.py --source bag) every file is byte-identical.
CHECK_TOL := --row-atol alignment=B:2e-3 --file-atol segments.csv=1e-5

check-results:
	@test -n "$(OUT)" || { echo "usage: make check-results OUT=<folder used by make results>"; exit 2; }
	$(PY) scripts/compare_results.py "$(OUT)/eval" eval $(CHECK_TOL)
	$(PY) scripts/compare_results.py "$(OUT)/tables" tables $(CHECK_TOL)
	@for f in tables/*.tex; do cmp -s "$$f" "$(OUT)/$$f" && echo "IDENTICAL $$f" || { echo "DIFFER    $$f"; exit 1; }; done

clean-eval:
	rm -f eval/results_long.csv eval/segments.csv eval/heading.csv eval/run_diagnostics.csv eval/resources_summary.csv
	rm -rf figs/* tables/*
