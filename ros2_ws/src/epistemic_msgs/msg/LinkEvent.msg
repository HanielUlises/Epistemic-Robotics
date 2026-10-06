# The state of the radio link between two robots, as the communication monitor
# sees it. RF-05 names /comm_monitor/link_down and /comm_monitor/link_up; both
# carry this type, and `up` says which of the two a reader is holding so that a
# node may subscribe to one topic or to both without a second code path.
#
# The stamp is the instant the transition was declared, and it is what RF-05
# calls t_disc on a fall and t_recon on a rise. It is not the publication time:
# a subscriber that connects late receives the event with the stamp it had.

std_msgs/Header header

# The two robots the link joins, in the order the monitor was configured with.
# A partition of more than two is out of scope for this scenario.
string[2] pair

# True on link_up, false on link_down.
bool up

# Seconds the link had been in the previous state when this event was declared.
# Zero for the first event of a run.
float64 previous_state_held_for
