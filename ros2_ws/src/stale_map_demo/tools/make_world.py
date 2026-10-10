#!/usr/bin/env python3
# Copyright 2026 Haniel Vásquez Morales
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

"""
Writes the stale-maps world: the pass-through floor as the shift found it.

    make_world.py --out /tmp/stale_maps.world

The hall, the racking and the clutter are pass_through_demo's, written by its
make_world.py's own functions. The loads are the shift map's, one in each bay
it shows blocked, each named `load_<bay>` so that the forklift can take it out
by name. The load the forklift stages is not in the world: it is spawned when
the forklift stages it. The robots are not in the world either; the launch
spawns them.
"""

import argparse
import importlib.util
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402


def _pass_through_world():
    """pass_through_demo's make_world.py, for its pieces."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.path.join(here, '..', '..', 'pass_through_demo', 'tools')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(get_package_share_directory('pass_through_demo'), 'tools'))
    except Exception:   # noqa: BLE001  outside a sourced workspace
        pass
    for path in candidates:
        source = os.path.join(path, 'make_world.py')
        if os.path.exists(source):
            sys.path.insert(0, path)
            spec = importlib.util.spec_from_file_location('pass_through_world', source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            sys.path.pop(0)
            return module
    raise RuntimeError('pass_through_demo/tools/make_world.py not found')


W = _pass_through_world()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', default=','.join(str(v) for v in L.OPENING_SHOT),
                    help='X,Y,Z,PITCH,YAW the Gazebo view opens on')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    P = L.P
    parts = [W.HEADER.replace('<world name="pass_through">', '<world name="stale_maps">')]
    shelves = P.block_shelves() + P.storage_shelves()
    parts.append(f'\n    <!-- Racking: {len(P.block_shelves())} units in the block, '
                 f'{len(P.storage_shelves())} in storage. -->\n')
    for i, (x, y) in enumerate(shelves):
        parts.append(W.include(P.SHELF_MODELS[i % 2], f'shelf_{i:03d}', x, y, P.SHELF_YAW))
    parts.append('\n    <!-- Dispatch and storage clutter. -->\n')
    for i, (model, x, y, yaw) in enumerate(P.CLUTTER):
        parts.append(W.include(model, f'clutter_{i:02d}', x, y, yaw))

    parts.append(f'\n    <!-- The shift map\'s loads: {", ".join(L.SHIFT_BLOCKED)}. The forklift '
                 'changes them during the shift. -->\n')
    for t in L.SHIFT_BLOCKED:
        x, y = L.bay_centre(t)
        parts.append(W.include(L.LOAD_MODEL, f'load_{t}', x, y, 0.0))

    parts.append(W.camera([float(v) for v in args.camera.split(',')]))
    parts.append(W.FOOTER)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as fh:
        fh.write(''.join(parts))
    print(f'{args.out}: {len(shelves)} shelves, loads in {" ".join(L.SHIFT_BLOCKED)}')


if __name__ == '__main__':
    main()
