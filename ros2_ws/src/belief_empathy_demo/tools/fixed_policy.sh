#!/bin/bash
# Stands in for the planner command of ePlanSys's Aletheia plugin, for a policy
# computed beforehand: it takes the plugin's arguments and writes the policy
# named by --fixed where the plugin reads the plan. The plugin validates it
# against the task as it does any planner's.
#
#     fixed_policy.sh --task T --plan P [--timeout N] [--consistent-beliefs] --fixed POLICY
set -e
while [ $# -gt 0 ]; do
  case "$1" in
    --task) TASK=$2; shift 2 ;;
    --plan) PLAN=$2; shift 2 ;;
    --fixed) FIXED=$2; shift 2 ;;
    --timeout) shift 2 ;;
    *) shift ;;
  esac
done
[ -f "$FIXED" ] || { echo "no policy at $FIXED" >&2; exit 1; }
cp "$FIXED" "$PLAN"
echo "Solution found: the policy in $FIXED, for $TASK"
