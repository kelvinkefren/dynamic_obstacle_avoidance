#!/usr/bin/env bash
# run_experiment.sh
# Wrapper opcional para não ficar digitando roslaunch gigante.

PKG="${1:-dynamic_obstacle_avoidance}"
SCN="${2:-gazebo_scenario_A1_single_static.py}"
METHOD="${3:-METHOD}"
OUT="${4:-$HOME/mestrado_runs}"
TIMEOUT="${5:-300}"

set -e

roslaunch "$PKG" run_scenario_with_eval.launch \
  scenario_script:="$SCN" \
  method_tag:="$METHOD" \
  output_dir:="$OUT" \
  timeout_s:="$TIMEOUT"
