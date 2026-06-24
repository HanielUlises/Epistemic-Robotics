# Represents a DEL event lifted from a sensor observation.
# Used by the SLAM bridge to feed the EpistemicWorldManager.

std_msgs/Header header

# Originating agent (robot index)
uint32 agent_id

# Event type matching EPDDL canonical action types:
#   0 = public_ontic
#   1 = private_sensing
#   2 = semi_private_sensing
#   3 = public_announce
#   4 = private_announce
uint8 event_type

# Serialised formula (JSON-encoded, plank format)
# Represents the proposition learned or updated
string formula_json

# Confidence in [0.0, 1.0] — maps to MC-Dropout uncertainty from SLAM
float32 confidence
