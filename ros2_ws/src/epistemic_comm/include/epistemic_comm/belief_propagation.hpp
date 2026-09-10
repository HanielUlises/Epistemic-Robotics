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

#ifndef EPISTEMIC_COMM__BELIEF_PROPAGATION_HPP_
#define EPISTEMIC_COMM__BELIEF_PROPAGATION_HPP_

#include <array>
#include <cstddef>

namespace epistemic_comm
{

/// A planar pose, which is all the propagation needs.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// The command the partner was last known to be executing.
///
/// RF-05 propagates "usando el modelo de movimiento del plan compartido", and
/// what a differential-drive plan reduces to at any instant is a pair of
/// velocities. The last pair received before the link fell is held and reused,
/// which is the weakest assumption that still yields a prediction: it says the
/// partner carried on doing what it was doing.
struct Twist2D
{
  double v{0.0};      ///< metres per second, along the body x axis
  double w{0.0};      ///< radians per second
};

/// Parameters of RF-05's propagation, all of them from the launch.
struct PropagationParams
{
  /// Maximum speed the platform can reach, in m/s. It appears in the
  /// acceptance bound v_max*dt + sigma_prop and nowhere else: the propagation
  /// itself uses the partner's last commanded speed, not this.
  double v_max{0.22};

  /// Standard deviation of the propagation model, in metres, at one second.
  double sigma_prop{0.05};

  /// Growth of the positional variance per second of disconnection, in m^2/s.
  /// Linear and not quadratic: the error of a dead-reckoned pose under a held
  /// velocity grows with elapsed time, and squaring it would make the bound
  /// unfalsifiable within a few seconds.
  double variance_rate{0.01};

  /// Growth of the heading variance per second, in rad^2/s.
  double yaw_variance_rate{0.005};

  /// Trace above which the estimate is no longer to be trusted, in m^2.
  /// RF-05's sigma_max^2: beyond it the proposition about the partner's
  /// position is to be marked uncertain rather than believed.
  double sigma_max_squared{1.0};
};

/// Integrate a differential-drive pose forward by `dt` seconds.
///
/// Exact for a constant twist rather than a first-order step: over a two
/// minute outage at 1 rad/s the Euler step accumulates metres of error that
/// belong to the integrator and not to the robot, and a measurement of RF-05's
/// bound would then be measuring this function.
Pose2D integrate(const Pose2D & from, const Twist2D & u, double dt);

/// The 6x6 row-major covariance ROS carries, for a planar propagation.
///
/// Only the x, y and yaw diagonal entries are populated; the rest stay zero,
/// which is what a planar estimate knows. The caller supplies the covariance
/// held when the link fell, and this adds the growth for `elapsed` seconds.
std::array<double, 36> propagated_covariance(
  const std::array<double, 36> & at_link_down,
  double elapsed,
  const PropagationParams & params);

/// RF-05's acceptance bound at `elapsed` seconds of disconnection, in metres.
///
/// v_max * dt + sigma_prop, verbatim from the requirement.
double admissible_error(double elapsed, const PropagationParams & params);

/// Trace of the positional part of a ROS covariance: the x and y variances.
///
/// The heading variance is excluded deliberately. It is not in metres, and
/// adding it to a positional trace to compare against a threshold in m^2
/// would be summing quantities of different dimension.
double positional_trace(const std::array<double, 36> & covariance);

}  // namespace epistemic_comm

#endif  // EPISTEMIC_COMM__BELIEF_PROPAGATION_HPP_
