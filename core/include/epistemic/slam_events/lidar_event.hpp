#pragma once

#include "event_model.hpp"
#include "world.hpp"

namespace epistemic {


// lidar observation
struct LidarObservation {
  std::vector<double> ranges;
  double max_range;
};

struct LidarSensorModel {
  double sigma;        // Gaussian noise
  double dropout_prob; // missed detection
};


// DEL event model from a lidar observation
EventModel build_lidar_event(
  const LidarObservation& obs,
  const LidarSensorModel& model
);

} // namespace epistemic
