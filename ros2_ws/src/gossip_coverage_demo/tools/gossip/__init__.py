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
Bramblett and Bezzo's framework for coverage and task allocation out of
contact (Frontiers in Robotics and AI 10:1149439, 2023), in one library that
the kinematic simulator and the Gazebo nodes share.

  env       the area, its obstacles and tasks, and what a range sensor sees
  nav       grid search, path smoothing and the potential field
  belief    the common belief set: one particle per robot, propagated by the
            coverage every robot computes alike from what they last shared
  alloc     the task bundling, makespan minimisation and gossip auctions of
            Section 5.4
"""
