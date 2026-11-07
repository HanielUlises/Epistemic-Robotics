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

"""
The worlds the runs are driven in.

A run is an act, a formulation and a seed. The seed picks the night in turn:
seed 1 the first of the act's worlds (tools/ranks.py), seed 2 the second, and
on round again, so that every night is run as often as every other; the pose
jitter of tools/make_world.py also follows the seed.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ranks as R  # noqa: E402

ROBOTS = {1: ['c'], 2: ['c', 'd'], 3: ['c', 'd']}
FORMULATIONS = R.FORMULATIONS


def world(act, seed):
    ws = R.worlds(act)
    return ws[(seed - 1) % len(ws)]


def tag(w):
    return R.name(w)
