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
The nights the runs are driven on.

A run is an act, a formulation and a seed. The seed picks the night in turn
from the act's nights (tools/ranks.py): the porter's round, and the scouts
whose laser is short-sighted. The pose jitter of tools/make_world.py also
follows the seed.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ranks as R  # noqa: E402

FORMULATIONS = R.FORMULATIONS


def robots(act):
    return R.robots(act)


def night(act, seed):
    ns = R.nights(act)
    return ns[(seed - 1) % len(ns)]


def seed_of(act, tag):
    return [R.tag(n) for n in R.nights(act)].index(tag) + 1


def tag(n):
    return R.tag(n)


if __name__ == '__main__':
    for act in sorted(R.FLEETS):
        print(act, ' '.join(f'{k + 1}:{R.tag(n)}' for k, n in enumerate(R.nights(act))))
