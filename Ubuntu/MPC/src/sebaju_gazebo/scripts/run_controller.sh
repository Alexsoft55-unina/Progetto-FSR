#!/usr/bin/env bash
# Run wbr_controller by hand against an already-running Gazebo instance
# (started separately with start_controller:=false). ros2 run's -p can't take the
# multi-line robot_description XML, so this generates a --params-file instead.
#
# Usage: run_controller.sh [-p name:=value ...]
#   Extra -p overrides are forwarded to ros2 run, after the generated params file,
#   so they take precedence over the defaults below.
#
# Example:
#   run_controller.sh -p planar_enable:=true

set -euo pipefail

SHARE=$(ros2 pkg prefix sebaju_gazebo)/share/sebaju_gazebo
PARAMS_FILE=$(mktemp /tmp/sebaju_controller_params.XXXXXX.yaml)
trap 'rm -f "$PARAMS_FILE"' EXIT

ROBOT_DESCRIPTION=$(xacro "$SHARE/urdf/sebaju.urdf.xacro" controllers_file:="$SHARE/config/sebaju_controllers.yaml")

python3 -c "
import json, sys
desc = sys.stdin.read()
params = {'/**': {'ros__parameters': {
    'robot_description': desc,
    'use_sim_time': True,
}}}
json.dump(params, open('$PARAMS_FILE', 'w'))
" <<< "$ROBOT_DESCRIPTION"

exec ros2 run sebaju_gazebo wbr_controller --ros-args --params-file "$PARAMS_FILE" "$@"
