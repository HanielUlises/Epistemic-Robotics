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

#include <array>
#include <cmath>

#include "epistemic_comm/belief_propagation.hpp"
#include "gtest/gtest.h"

using epistemic_comm::PropagationParams;
using epistemic_comm::Pose2D;
using epistemic_comm::Twist2D;

TEST(Integrate, StandingStillDoesNotMove)
{
  const Pose2D from{1.0, 2.0, 0.3};
  const auto to = epistemic_comm::integrate(from, Twist2D{0.0, 0.0}, 10.0);
  EXPECT_NEAR(to.x, from.x, 1e-12);
  EXPECT_NEAR(to.y, from.y, 1e-12);
  EXPECT_NEAR(to.yaw, from.yaw, 1e-12);
}

TEST(Integrate, StraightLineIsSpeedTimesTime)
{
  const auto to = epistemic_comm::integrate({0.0, 0.0, 0.0}, {0.22, 0.0}, 30.0);
  EXPECT_NEAR(to.x, 6.6, 1e-9);
  EXPECT_NEAR(to.y, 0.0, 1e-9);
}

TEST(Integrate, ATurnFollowsTheArcAndNotTheTangent)
{
  // A quarter turn of radius 1 m: v = 1, w = 1, for pi/2 seconds. The arc ends
  // at (1, 1); the tangent would end at (pi/2, 0), which is the error a
  // first-order step would introduce and the reason the closed form is used.
  const auto to = epistemic_comm::integrate({0.0, 0.0, 0.0}, {1.0, 1.0}, M_PI / 2.0);
  EXPECT_NEAR(to.x, 1.0, 1e-9);
  EXPECT_NEAR(to.y, 1.0, 1e-9);
  EXPECT_NEAR(to.yaw, M_PI / 2.0, 1e-9);
}

TEST(Integrate, AFullCircleReturnsToItsStart)
{
  const auto to = epistemic_comm::integrate({0.0, 0.0, 0.0}, {1.0, 1.0}, 2.0 * M_PI);
  EXPECT_NEAR(to.x, 0.0, 1e-9);
  EXPECT_NEAR(to.y, 0.0, 1e-9);
}

TEST(Integrate, HeadingStaysWrapped)
{
  // Ten turns. An unwrapped heading would be past sixty radians.
  const auto to = epistemic_comm::integrate({0.0, 0.0, 0.0}, {0.0, 1.0}, 20.0 * M_PI);
  EXPECT_LE(std::abs(to.yaw), M_PI);
}

TEST(Bound, IsVmaxTimesElapsedPlusSigma)
{
  PropagationParams p;
  p.v_max = 0.22;
  p.sigma_prop = 0.05;
  EXPECT_NEAR(epistemic_comm::admissible_error(0.0, p), 0.05, 1e-12);
  EXPECT_NEAR(epistemic_comm::admissible_error(30.0, p), 6.65, 1e-12);
}

TEST(Bound, NegativeElapsedIsTreatedAsZero)
{
  PropagationParams p;
  EXPECT_NEAR(
    epistemic_comm::admissible_error(-5.0, p),
    epistemic_comm::admissible_error(0.0, p), 1e-12);
}

TEST(Covariance, GrowsAndNeverShrinks)
{
  PropagationParams p;
  p.variance_rate = 0.01;
  std::array<double, 36> at_down{};
  at_down[0] = 0.001;
  at_down[7] = 0.001;

  double previous = epistemic_comm::positional_trace(at_down);
  for (double t = 0.0; t <= 60.0; t += 1.0) {
    const auto c = epistemic_comm::propagated_covariance(at_down, t, p);
    const double trace = epistemic_comm::positional_trace(c);
    EXPECT_GE(trace, previous);   // RF-05's invariant, verbatim
    previous = trace;
  }
}

TEST(Covariance, TraceIsPositionalOnly)
{
  // The heading variance is in rad^2 and must not be added to a trace that is
  // compared against a threshold in m^2.
  std::array<double, 36> c{};
  c[0] = 0.2;
  c[7] = 0.3;
  c[35] = 100.0;
  EXPECT_NEAR(epistemic_comm::positional_trace(c), 0.5, 1e-12);
}

TEST(Covariance, ReachesSigmaMaxWhenTheRateSaysItShould)
{
  PropagationParams p;
  p.variance_rate = 0.01;
  p.sigma_max_squared = 1.0;
  std::array<double, 36> at_down{};
  // Two axes at 0.01 m^2/s each: the trace grows 0.02 m^2 per second and
  // crosses 1.0 m^2 at fifty seconds.
  const auto before = epistemic_comm::propagated_covariance(at_down, 49.0, p);
  const auto after = epistemic_comm::propagated_covariance(at_down, 51.0, p);
  EXPECT_LT(epistemic_comm::positional_trace(before), p.sigma_max_squared);
  EXPECT_GT(epistemic_comm::positional_trace(after), p.sigma_max_squared);
}

TEST(Bound, HoldsForAPartnerThatKeepsItsCourse)
{
  // The case the scenario is built to exercise: the partner carries on at the
  // speed it was last seen at, so the propagation tracks it and the error
  // stays far inside the bound. This is arithmetic, not a claim about the
  // robot; the run is what tests the robot.
  PropagationParams p;
  const Pose2D start{0.0, 0.0, 0.0};
  const Twist2D held{0.22, 0.0};
  for (double t = 1.0; t <= 60.0; t += 1.0) {
    const auto believed = epistemic_comm::integrate(start, held, t);
    const auto actual = epistemic_comm::integrate(start, held, t);
    const double error = std::hypot(believed.x - actual.x, believed.y - actual.y);
    EXPECT_LE(error, epistemic_comm::admissible_error(t, p));
  }
}

TEST(Bound, IsBreachedByAPartnerThatReversesAtFullSpeed)
{
  // The worst case the bound admits: the partner turns around the instant the
  // link falls. The true displacement is then 2*v*t from the belief, which
  // exceeds v_max*t + sigma for any t past sigma/v_max. The bound is not a
  // guarantee about the world, and a run that never breaches it has not shown
  // that it cannot be breached.
  PropagationParams p;
  const Twist2D held{0.22, 0.0};
  const Twist2D reversed{-0.22, 0.0};
  for (double t = 10.0; t <= 60.0; t += 10.0) {
    const auto believed = epistemic_comm::integrate({0.0, 0.0, 0.0}, held, t);
    const auto actual = epistemic_comm::integrate({0.0, 0.0, 0.0}, reversed, t);
    const double error = std::hypot(believed.x - actual.x, believed.y - actual.y);
    EXPECT_GT(error, epistemic_comm::admissible_error(t, p));
  }
}
