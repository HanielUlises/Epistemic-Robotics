#include "epistemic/slam_events/lidar_event.hpp"

#include <cmath>
#include <string>

namespace epistemic {

EventModel build_lidar_event(
    const LidarObservation& obs,
    const LidarSensorModel& model
    ) {
    EventModel em;

    const std::size_t N = obs.ranges.size();
        if (N == 0) {
        return em;
    }

    // Events
    for (std::size_t i = 0; i < N; ++i) {

        Formula precondition{
            Atom{ "lidar_bin_" + std::to_string(i) }
        };

        em.events.push_back(
            Event{ i, precondition }
        );
    }

    // Epistemic accessibility (R^E)
    Agent sensing_agent = 0;

    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {

            // Reflexivity
            if (i == j) {
                em.accessibility[sensing_agent].push_back({i, j});
                continue;
            }

            double ri = obs.ranges[i];
            double rj = obs.ranges[j];

            bool ri_invalid = (ri <= 0.0 || ri >= obs.max_range);
            bool rj_invalid = (rj <= 0.0 || rj >= obs.max_range);

            if (ri_invalid && rj_invalid) {
                em.accessibility[sensing_agent].push_back({i, j});
                continue;
            }

            if (std::fabs(ri - rj) <= model.sigma) {
                em.accessibility[sensing_agent].push_back({i, j});
            }
        }
    }


    //Dropout i.e. total indistinguishability
    if (model.dropout_prob > 0.5) {
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
        em.accessibility[sensing_agent].push_back({i, j});
        }
    }
    }

    return em;
    }

} // namespace epistemic
