#!/usr/bin/env python3
"""Generate the extended (harder) instance families.

Two families beyond the corridor grid of gen_instances.py:

  directed  point-to-point radio, single detector in the fleet. Goals are
            second-order (R1 must know that R2 knows) or selective
            (R2 must be informed while R3 stays ignorant) --- neither is
            reachable under a broadcast model, where every transmission
            produces common knowledge.

  facility  heterogeneous fleet, several items to localise, and a locked bay
            in the middle of the corridor. Only one vehicle carries a
            detector and only one carries a key, so the ontic and epistemic
            halves of the mission must interlock.
"""

import itertools
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "instances")

HDR_REQ = """    (:requirements
        :typing :equality :list-comprehensions
        :finitary-S5-theories :modal-goals :facts
        :knowing-whether :group-modalities
        :disjunctive-preconditions
    )"""


def agents(n):
    return [f"R{k + 1}" for k in range(n)]


def bays(L):
    return [f"b{k + 1}" for k in range(L)]


def adjacency(bs):
    return "\n        ".join(
        f"(adjacent {bs[k]} {bs[k + 1]})\n        (adjacent {bs[k + 1]} {bs[k]})"
        for k in range(len(bs) - 1)
    )


def uniqueness(pred, args_prefix=""):
    return (f"(forall (?l1 ?l2 - location | (/= ?l1 ?l2))\n"
            f"                        (imply ({pred} {args_prefix}?l1) "
            f"(not ({pred} {args_prefix}?l2))) )")


# ----------------------------------------------------------------- directed

def directed(n, L, U, placement, goal):
    ag, bs = agents(n), bays(L)
    cand = bs[-U:]
    target = cand[0]
    start = [0] * n if placement == "depot" else [k % L for k in range(n)]
    name = f"directed-a{n}-b{L}-u{U}-{placement}-{goal}"

    positions = " ".join(f"(at {i} {bs[s]})" for i, s in zip(ag, start))
    not_cand = "\n                    ".join(
        f"(not (target-at {b}))" for b in bs if b not in cand)
    not_cand_block = f"{not_cand}\n                    " if not_cand else ""
    disj = " ".join(f"(target-at {b})" for b in cand)

    if goal == "nested":
        # R1 (the only vehicle with a detector) must come to know that R2 knows.
        goal_f = f"([{ag[0]}] ([Kw. {ag[1]}] (target-at {target})))"
    else:
        # Selective disclosure: inform R2, leave R3 ignorant.
        goal_f = (f"(and\n            ([Kw. {ag[1]}] (target-at {target}))\n"
                  f"            (not ([Kw. {ag[2]}] (target-at {target}))) )")

    return name, f"""(define (problem {name})
    ;; {n} UGVs ({ag[0]} is the only one with a detector), {L} bays, target in one
    ;; of {U} candidate bays, {placement} start, {goal} goal.
    (:domain ugv-directed)

{HDR_REQ}

    (:agents {" ".join(ag)})

    (:objects {" ".join(bs)} - location)

    (:facts-init
        {adjacency(bs)}
        (has-scanner {ag[0]})
    )

    (:init
        (:and
            {positions}

            ([C. All]
                (and
                    {positions}
                    (forall (?i - agent ?l1 ?l2 - location | (/= ?l1 ?l2))
                        (imply (at ?i ?l1) (not (at ?i ?l2))) )

                    {not_cand_block}(or {disj})
                    {uniqueness("target-at")}))

            (:forall (?i - agent)
                ([C. All] (<Kw. ?i> (target-at {target}))) )
        )
    )

    (:goal
        {goal_f}
    )
)
"""


# ----------------------------------------------------------------- facility

def facility(n, L, I):
    """n UGVs, L bays, I items. R1 scans, R2 unlocks, the rest are ballast."""
    ag, bs = agents(n), bays(L)
    name = f"facility-a{n}-b{L}-i{I}-depot-ck"

    # Item k is in one of two adjacent bays at the far end, working inwards.
    cands = [[bs[L - 1 - 2 * k], bs[L - 2 - 2 * k]] for k in range(I)]
    items = [f"o{k + 1}" for k in range(I)]
    locked = bs[(L - 1) // 2]          # a mid-corridor bay, blocking the way

    positions = " ".join(f"(at {i} {bs[0]})" for i in ag)

    item_blocks = []
    for o, cs in zip(items, cands):
        others = "\n                    ".join(
            f"(not (item-at {o} {b}))" for b in bs if b not in cs)
        item_blocks.append(
            f"""{others}
                    (or {" ".join(f"(item-at {o} {b})" for b in cs)})
                    {uniqueness("item-at", f"{o} ")}""")
    items_ck = "\n\n                    ".join(item_blocks)

    ignorance = "\n            ".join(
        f"(:forall (?i - agent)\n                ([C. All] (<Kw. ?i> (item-at {o} {cs[0]}))) )"
        for o, cs in zip(items, cands))

    goal_conj = "\n                ".join(
        f"([Kw. {i}] (item-at {o} {cs[0]}))"
        for o, cs in zip(items, cands) for i in ag)

    return name, f"""(define (problem {name})
    ;; {n} UGVs: {ag[0]} carries the detector, {ag[1]} the key. {L} bays with
    ;; {locked} locked, {I} item(s) to localise. Fleet-wide common-knowledge goal.
    (:domain ugv-facility)

{HDR_REQ}

    (:agents {" ".join(ag)})

    (:objects
        {" ".join(bs)} - location
        {" ".join(items)} - item
    )

    (:facts-init
        {adjacency(bs)}
        (has-scanner {ag[0]})
        (keyholder {ag[1]})
    )

    (:init
        (:and
            {positions}
            (locked {locked})

            ([C. All]
                (and
                    {positions}
                    (locked {locked})
                    (forall (?l - location | (/= ?l {locked})) (not (locked ?l)))
                    (forall (?i - agent ?l1 ?l2 - location | (/= ?l1 ?l2))
                        (imply (at ?i ?l1) (not (at ?i ?l2))) )

                    {items_ck}))

            {ignorance}
        )
    )

    (:goal
        ([C. All]
            (and
                {goal_conj} ))
    )
)
"""


def main():
    os.makedirs(OUT, exist_ok=True)
    written = []

    for n, (L, U) in itertools.product(
            (2, 3, 4), ((3, 2), (4, 2), (4, 3), (5, 2), (5, 3))):
        for placement in ("depot", "spread"):
            for goal in ("nested", "private"):
                if goal == "private" and n < 3:
                    continue          # needs a third vehicle to keep ignorant
                written.append(directed(n, L, U, placement, goal))

    for n, (L, I) in itertools.product((2, 3), ((4, 1), (4, 2), (5, 1), (5, 2), (6, 1))):
        written.append(facility(n, L, I))

    for name, text in written:
        with open(os.path.join(OUT, f"{name}.epddl"), "w") as fh:
            fh.write(text)
    print(f"wrote {len(written)} extended instances to {OUT}")


if __name__ == "__main__":
    main()
