# link-grades

Two robots, `r1` and `r2`, must both come to know whether a corridor is
blocked; only a robot at the corridor's mouth can decide it. What one robot
has learned reaches the other only by `share`, a message over a link that may
be down:

```
(and ([Kw. r1] (blocked)) ([Kw. r2] (blocked)))
```

`link.epddl` holds the action type `lossy-message`: three designated events,
delivered with one value, delivered with the other, or lost, each with trivial
postconditions and each observed fully by sender and receiver, the receiver
because the message arrives or not and the sender because the acknowledgement,
carried by the same link, does. The delivered events require `(link-up)`, the
lost one `(not (link-up))`.

Four instances:

- `problem_open`: neither robot knows whether the corridor is blocked or
  whether the link is up; four worlds, all designated.
- `problem_up`, `problem_down`: the link's state is common knowledge.
- `problem_lost`: r1 at the mouth knows the answer, r2 does not, the link is
  down; the task replanned in after an unacknowledged share.

```sh
plank export -d link-grades.epddl -p instances/problem_open.epddl \
  -l ~/plank/benchmarks/libraries/intermediate.epddl link.epddl -o out
epistemic_planner --task out/problem_open.json --plan out/plan.json --conditional --consistent-beliefs
```

On `problem_open` Aletheia's AO* returns, at depth 4 after 23 expansions, a
policy in which both robots go and look and nobody shares. AO* minimises the
depth of the worst branch, and a share whose lost branch falls back to looking
is one step deeper than looking alone, so with the link unknown it never tries
the link. On `problem_up` it returns, at depth 3, r1 looking and sharing, with
no lost branch, since the lost event is not applicable there.

The executor in `ros2_ws/src/link_grades_demo` runs both in Gazebo against
three classical formulations; the page is
<https://hanielulises.github.io/Epistemic-Robotics/link_grades.html>.
