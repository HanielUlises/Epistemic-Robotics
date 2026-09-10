#!/bin/bash
# Copyright 2026 Haniel Ulises
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Builds the four worlds the multi-site survey runs on.
#
# They differ by where one pallet stands and by nothing else: same hall, same
# 153 obstacles, same nine actors, same camera. That is the whole point --- a
# sensing action reporting the same value whatever it is pointed at is not
# sensing, and the way to show it is not is to move one object and hold
# everything else fixed.
#
#   warehouse_xl_a17.world    the pallet in a17, which the relay scans
#   warehouse_xl_a31.world    the pallet in a31, which the scout scans
#   warehouse_xl_a06.world    the pallet in a06, which nobody goes to
#   warehouse_xl_none.world   no pallet anywhere: the control
#
# The last is a control and not a fourth case of the domain. The domain says
# exactly one site is contaminated, so a world with no pallet contradicts the
# problem the planner was given; running it shows what the fleet does when the
# world and the model disagree, which is worth knowing and is not a
# demonstration of anything working.
#
# The pallet coordinates come from tools/check_sites.py, which is also what
# refuses the placements that would put a prop across a lane.
#
#     make_site_worlds.sh [output directory, default /tmp]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-/tmp}"

# The floor is GPL-2.0 and is fetched, not vendored, so it sits in the source
# workspace and not in the installed share directory. This script is run from
# both, and the path relative to its own location is therefore correct in one
# case only. Candidates are tried in order and WORLD overrides all of them.
REL=third_party/dynamic_logistics_warehouse/worlds/warehouse.world
WORLD="${WORLD:-}"
for candidate in "$WORLD" \
                 "$HERE/../../../$REL" \
                 "$HOME/Projects/Epistemic-Robotics/ros2_ws/$REL"; do
  [ -n "$candidate" ] && [ -f "$candidate" ] && WORLD="$candidate" && break
done

if [ -z "$WORLD" ] || [ ! -f "$WORLD" ]; then
  echo "no world found; tried the path relative to $HERE and the source workspace" >&2
  echo "the floor is GPL-2.0 and fetched rather than vendored; see the README" >&2
  exit 1
fi

# A metre and a half north of a17, at half the height of the robot's mast,
# looking south down the aisle. The scout enters from dock17, which is the only
# lane into a17 and lies to the south, so it drives towards the camera and
# grows in frame.
#
# The distance is set by the size the robot has to occupy and not by what makes
# a pleasing composition. A TurtleBot3 Waffle is 0.28 m across. Measured off
# the capture, it spans some forty-eight pixels of a 1640-pixel canvas at
# 3.15 m and some twenty at 4.65 m, and at either distance the crop that makes
# it legible contains nothing but floor. At 1.5 m it spans about a hundred, and
# the aisle either side of it survives the crop.
#
# The height is 0.50 m rather than 2.60 m for a separate reason: from above, a
# Waffle is a disc. The earlier recording showed a dark circle on a floor and
# nothing that reads as a vehicle. Near the height of its own mast the chassis,
# the mast and the laser return are all distinguishable.
POSE="1.49 17.25 0.50 0 0.229 -1.5708"

place() {
  local name="$1" pallet="$2"
  local args=(--world "$WORLD" --pose "$POSE" --out "$OUT/warehouse_xl_${name}.world")
  # --pallet=X,Y,YAW and not --pallet X,Y,YAW. Two of the three sites are at
  # negative x, so the value begins with a minus; argparse treats a following
  # token that starts with "-" as an option unless it parses as a plain
  # negative number, and "-4.15,2.0,0" does not. Separated, it fails with
  # "expected one argument" and names the option rather than the value, which
  # sends you looking in the wrong place.
  [ -n "$pallet" ] && args+=("--pallet=$pallet")
  python3 "$HERE/with_camera.py" "${args[@]}"
}

place a17  "0.69,15.75,0"
place a31  "14.44,37.0,0"
place a06  "31.04,-1.75,0"
place none ""

echo
echo "run one with:"
echo "  ros2 launch warehouse_xl_rmf_demo survey_sites_launch.py \\"
echo "      contaminated:=a06 world:=$OUT/warehouse_xl_a06.world"
