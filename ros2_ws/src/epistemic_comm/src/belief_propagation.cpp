// Copyright 2026 Haniel Vásquez Morales
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "epistemic_comm/belief_propagation.hpp"

#include <algorithm>
#include <cmath>

namespace epistemic_comm
{

namespace
{
/// Below this angular rate the arc and the straight line differ by less than
/// the numerical noise of the closed form, and the closed form divides by w.
constexpr double kStraightLine = 1e-6;

/// Indices of the x, y and yaw variances in a row-major 6x6 ROS covariance.
constexpr std::size_t kXX = 0;
constexpr std::size_t kYY = 7;
constexpr std::size_t kYawYaw = 35;
}  // namespace

Pose2D integrate(const Pose2D & from, const Twist2D & u, double dt)
{
  Pose2D to;
  if (std::abs(u.w) < kStraightLine) {
    to.x = from.x + u.v * dt * std::cos(from.yaw);
    to.y = from.y + u.v * dt * std::sin(from.yaw);
    to.yaw = from.yaw;
    return to;
  }

  // Constant twist: the body traces an arc of radius v/w. Integrating the
  // differential-drive equations over dt with v and w held gives this in
  // closed form.
  const double radius = u.v / u.w;
  const double turned = u.w * dt;
  to.x = from.x + radius * (std::sin(from.yaw + turned) - std::sin(from.yaw));
  to.y = from.y - radius * (std::cos(from.yaw + turned) - std::cos(from.yaw));
  to.yaw = from.yaw + turned;

  // Wrapped, so that a long outage does not hand the consumer a heading of
  // forty radians that is arithmetically right and useless to read.
  while (to.yaw > M_PI) {to.yaw -= 2.0 * M_PI;}
  while (to.yaw < -M_PI) {to.yaw += 2.0 * M_PI;}
  return to;
}

std::array<double, 36> propagated_covariance(
  const std::array<double, 36> & at_link_down,
  double elapsed,
  const PropagationParams & params)
{
  std::array<double, 36> out = at_link_down;
  const double dt = std::max(0.0, elapsed);
  out[kXX] += params.variance_rate * dt;
  out[kYY] += params.variance_rate * dt;
  out[kYawYaw] += params.yaw_variance_rate * dt;
  return out;
}

double admissible_error(double elapsed, const PropagationParams & params)
{
  return params.v_max * std::max(0.0, elapsed) + params.sigma_prop;
}

double positional_trace(const std::array<double, 36> & covariance)
{
  return covariance[kXX] + covariance[kYY];
}

}  // namespace epistemic_comm
