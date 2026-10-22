# belief-empathy

Bramblett and Bezzo's heterogeneous team out of radio contact
([arXiv:2308.00579](https://arxiv.org/abs/2308.00579)), as an EPDDL task. An
aerial vehicle `u` and two ground vehicles, `g1` covering the west half of the
area and `g2` the east. Out of contact, every robot keeps three belief
particles for each ground vehicle: where it is along its known route with no
fault, one or two. A vehicle with a fault tracks its next particle, which is
the paper's empathy, so "`g` stands at particle `b`" and "`g` has had `b - 1`
faults" are the one atom `(pb g)`. At most two faults happen in the team: six
worlds, all designated. Each ground vehicle knows its own particle and nobody
knows another's.

`u` has found a task, `(task)`, which it knows and the ground vehicles do not.
The task needs `u` and a ground vehicle with fewer than two faults at its site:

```
(:goal (done))
```

A ground vehicle drives to the site only when it knows of the task,
`([?g] (task))`. Only a hail can tell it: `u` flies to one of its particles and
calls (`hail-pb`). If the vehicle is there it hears the call, and since the
event that it answered requires `(task)`, it learns of the task; if it is not,
`u` learns that. The particles lie in order along the route, so `u` reaches the
first directly and the second and third only by following the route back
(`back-p2`, `back-p3`). The half `u` is in is a fact, and flying to the other
half is an action of its own (`cross-west`, `cross-east`).

```sh
plank export -d belief-empathy.epddl -p instances/problem_east.epddl \
  -l ~/plank/benchmarks/libraries/intermediate.epddl hailing.epddl -o out
epistemic_planner --task out/problem_east.json --plan out/policy.json --conditional --consistent-beliefs
```

Aletheia's AO* returns, at depth 10 after 5848 expansions, one policy for the
six worlds. With the site in the east, `u` hails `g2` at its first particle and,
if nobody answers, at its second. If both are empty, `u` knows without going
there that `g2` holds its third particle, so it has had two faults and cannot do
the task, and from the fault budget that `g1` has had none: it crosses west and
hails `g1` at its first particle, where `g1` is sure to be. With the site in
the west (`instances/problem_west.epddl`) the policy is the same with the two
vehicles swapped.

## Who observes what

`hailing.epddl` holds the two action types the domain uses. A teammate out of
contact is `Unsure` of what another robot does: from every event it considers
every event of the action possible, the null one included. It is not
`Oblivious`, which would give it the belief that nothing happened. The paper's
update rules leave the knowledge of disconnected robots "not affected", which
is the oblivious reading, and `variants/hailing-oblivious.epddl` keeps it: a
vehicle that believes `u` never left then hears `u`'s hail, every world it
considers possible contradicts the hail's precondition, and it believes a
contradiction. With `--consistent-beliefs` Aletheia finds no policy; without
it, a policy of depth 8 whose knowledge preconditions hold only vacuously.

## The variant with latent tool faults

`belief-empathy-latent.epddl` separates a fault's effect on speed from its
effect on capability. A drive fault slows a vehicle and costs it nothing else;
a tool fault, `(not (tool ?g))`, is known to nobody until the vehicle attempts
the task, `perform`, whose two events are done and failed. At most two faults
of either kind, and at least one tool works: twelve designated patterns
(`instances/problem_latent_{east,west}.epddl`). Aletheia's policy, of depth 12
after 36136 expansions, recruits both vehicles: the first met is sent to the
site and the aerial vehicle seeks the second. `variants/belief-empathy-latent-direct.epddl`
lets the aerial vehicle fly to a second or third particle from anywhere once it
knows the particles before it empty; it was compared in the kinematic model
only.

## Files

- `belief-empathy.epddl`: the domain.
- `hailing.epddl`: the action types `unseen-ontic` and `hail`.
- `instances/problem_east.epddl`, `instances/problem_west.epddl`: the site in
  each half.
- `variants/hailing-oblivious.epddl`: the oblivious reading, for comparison.
- `belief-empathy-latent.epddl`, `instances/problem_latent_{east,west}.epddl`:
  the variant with latent tool faults.
- `variants/belief-empathy-latent-direct.epddl`: the variant with direct
  flights to a particle.

The executor in `ros2_ws/src/belief_empathy_demo` runs the policy in Gazebo
against the paper's own procedure, its ideal method and its flock method; the
page is <https://hanielulises.github.io/Epistemic-Robotics/belief_empathy.html>.
