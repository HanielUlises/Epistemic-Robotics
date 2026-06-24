# Request to the mu-calculus path planner.
# The planner computes: μZ.(at_goal ∨ ⟨move⟩Z ∧ ¬obstacle)

std_msgs/Header header

# Agent issuing the request
uint32 agent_id

# Goal zone name (must match a proposition in the current epistemic model)
string goal_zone

# Safety constraint formula in plank JSON format.
# If empty, defaults to ¬obstacle.
string safety_formula_json

# Whether to require K_i(at_goal) vs. just at_goal.
# True = epistemic goal (agent must *know* it reached goal)
# False = ontic goal (robot is at goal regardless of knowledge)
bool require_epistemic_goal
