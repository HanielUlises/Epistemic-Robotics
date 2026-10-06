# One robot's belief about where its partner is, propagated while the link is
# down (RF-05). While the link is up this is the last received pose and the
# covariance the partner reported; while it is down it is the forward
# integration of the last shared plan's motion model, with the covariance
# growing without bound until the link returns.

std_msgs/Header header

# Whose belief this is, and about whom.
string holder
string subject

# The believed pose of `subject`, in the frame named by the header.
geometry_msgs/PoseWithCovariance pose

# False while the link is up: the estimate is then a received measurement and
# not a propagation, and the acceptance criterion of RF-05 does not apply.
bool propagated

# Seconds since the link fell. Zero while it is up. This is RF-05's Δt, and the
# bound v_max·Δt + sigma_prop is a function of it.
float64 elapsed_since_link_down

# The bound RF-05 admits at this instant, in metres, and the trace of the
# propagation covariance. Published so that a reader of a bag need not know the
# parameters the run was given.
float64 admissible_error
float64 covariance_trace
