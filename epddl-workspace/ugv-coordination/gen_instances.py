#!/usr/bin/env python3
"""Generate the UGV-coordination instance family.

Each instance is a corridor of L bays, N UGVs, and a target that is known to
sit in one of the last U bays. Two initial placements (all UGVs at the depot,
or spread round-robin along the corridor) and two goals (fleet-wide common
knowledge, or a nested second-order goal) are crossed with the two domains.
"""

import itertools
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "instances")

DOMAINS = {"public": "ugv-public", "radio": "ugv-radio"}


def agents(n):
    return [f"R{k + 1}" for k in range(n)]


def bays(L):
    return [f"b{k + 1}" for k in range(L)]


def placement_of(mode, n, L):
    """Bay index (0-based) each UGV starts in."""
    if mode == "depot":
        return [0] * n
    return [k % L for k in range(n)]


def goal_formula(mode, ag, target):
    if mode == "ck":
        conj = "\n                ".join(f"([Kw. {i}] (target-at {target}))" for i in ag)
        return f"([C. All]\n            (and\n                {conj} ))"
    # nested: R1 knows that R2 knows whether the target is in the target bay,
    # without that being common knowledge in the fleet.
    return f"([{ag[0]}] ([Kw. {ag[1]}] (target-at {target})))"


def instance(dom, n, L, U, placement, goal):
    ag, bs = agents(n), bays(L)
    cand = bs[-U:]                       # candidate bays: the far end of the corridor
    target = cand[0]
    start = placement_of(placement, n, L)

    name = f"{dom}-a{n}-b{L}-u{U}-{placement}-{goal}"

    adj = "\n        ".join(
        f"(adjacent {bs[k]} {bs[k + 1]})\n        (adjacent {bs[k + 1]} {bs[k]})"
        for k in range(L - 1)
    )
    positions = " ".join(f"(at {i} {bs[s]})" for i, s in zip(ag, start))
    not_cand = "\n                    ".join(
        f"(not (target-at {b}))" for b in bs if b not in cand
    )
    disj = " ".join(f"(target-at {b})" for b in cand)
    not_cand_block = f"{not_cand}\n                    " if not_cand else ""

    return name, f"""(define (problem {name})
    ;; {n} UGVs, {L} bays, target in one of {U} candidate bays, {placement} start,
    ;; {"fleet-wide common knowledge" if goal == "ck" else "second-order"} goal.
    (:domain {DOMAINS[dom]})

    (:requirements
        :typing :equality :list-comprehensions
        :finitary-S5-theories :modal-goals :facts
        :knowing-whether :group-modalities
        :disjunctive-preconditions
    )

    (:agents {" ".join(ag)})

    (:objects {" ".join(bs)} - location)

    (:facts-init
        {adj}
    )

    ;; The true bay is left open, so every candidate world stays designated and
    ;; the plan must succeed whichever way the scans come out.
    (:init
        (:and
            {positions}

            ([C. All]
                (and
                    {positions}
                    (forall (?i - agent ?l1 ?l2 - location | (/= ?l1 ?l2))
                        (imply (at ?i ?l1) (not (at ?i ?l2))) )

                    {not_cand_block}(or {disj})
                    (forall (?l1 ?l2 - location | (/= ?l1 ?l2))
                        (imply (target-at ?l1) (not (target-at ?l2))) )))

            (:forall (?i - agent)
                ([C. All] (<Kw. ?i> (target-at {target}))) )
        )
    )

    (:goal
        {goal_formula(goal, ag, target)}
    )
)
"""


def grid():
    for dom in DOMAINS:
        for n, L in itertools.product((2, 3, 4), (3, 4, 5, 6)):
            for U in (2, 3, 4):
                if U > L - 1 or (L == 6 and U < 4) or (L < 6 and U > 3):
                    continue          # keep the far end of the grid sparse
                for placement, goal in itertools.product(("depot", "spread"), ("ck", "nested")):
                    yield dom, n, L, U, placement, goal


def main():
    os.makedirs(OUT, exist_ok=True)
    count = 0
    for args in grid():
        name, text = instance(*args)
        with open(os.path.join(OUT, f"{name}.epddl"), "w") as fh:
            fh.write(text)
        count += 1
    print(f"wrote {count} instances to {OUT}")


if __name__ == "__main__":
    main()
