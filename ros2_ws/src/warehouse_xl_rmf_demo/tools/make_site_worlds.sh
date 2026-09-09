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
WORLD="${WORLD:-$HERE/../../../third_party/dynamic_logistics_warehouse/worlds/warehouse.world}"

if [ ! -f "$WORLD" ]; then
  echo "no world at $WORLD" >&2
  echo "the floor is GPL-2.0 and fetched rather than vendored; see the README" >&2
  exit 1
fi

# Looking across a17 from the north, which is the view the single-site
# recording used: the robot approaches from the south, so it drives towards the
# camera and grows in frame.
POSE="1.40 20.4 2.6 0 0.454 -1.6239"

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
