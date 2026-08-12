#!/usr/bin/env python3
"""Turn results.csv into the LaTeX tables and plot included by the report."""

import csv
import os
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.abspath(os.path.join(HERE, "..", "..", "tables"))

FAMILY = {("public", "ck"): "S1 public / CK",
          ("radio", "ck"): "S2 radio / CK",
          ("public", "nested"): "S3a public / nested",
          ("radio", "nested"): "S3b radio / nested"}

# The extended families, reported separately: they vary different parameters
# (items, capabilities) and would distort the corridor grid's aggregates.
EXTENDED = {("directed", "nested"): "S5a directed / 2nd-order",
            ("directed", "private"): "S5b directed / selective",
            ("facility", "ck"): "S6 facility / CK"}

CORRIDOR_DOMAINS = ("public", "radio")


def load():
    with open(os.path.join(HERE, "results.csv")) as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k in ("agents", "bays", "candidates", "atoms", "worlds", "designated",
                  "ground_actions", "depth", "expanded", "generated", "leaves", "branches"):
            r[k] = int(r[k]) if r[k] else None
        for k in ("ground_s", "plan_s"):
            r[k] = float(r[k]) if r[k] else None
    return rows


def med(vals, fmt="{:.0f}"):
    vals = [v for v in vals if v is not None]
    return fmt.format(st.median(vals)) if vals else "--"


def solved(rows):
    return [r for r in rows if r["status"] == "solved"]


def write(name, body):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w") as fh:
        fh.write(body)
    print("wrote", os.path.join(OUT, name))


def t_family(rows):
    lines = []
    for (dom, goal), label in FAMILY.items():
        sub = [r for r in rows if r["domain"] == dom and r["goal"] == goal]
        dep = [r for r in sub if r["placement"] == "depot"]
        spr = [r for r in sub if r["placement"] == "spread"]
        lines.append(f"{label} & {len(sub)} & {len(dep)} & {len(spr)} & "
                     f"{min(r['ground_actions'] for r in sub if r['ground_actions'])}--"
                     f"{max(r['ground_actions'] for r in sub if r['ground_actions'])} \\\\")
    body = r"""\begin{table}[H]
\centering
\small
\caption{The instance family. Ground-action counts are the range over the
instances of each family after grounding with \textsf{plank}.}
\label{tab:family}
\begin{tabular}{@{}l r r r r@{}}
\toprule
\textbf{Family} & \textbf{Instances} & \textbf{Depot} & \textbf{Spread} & \textbf{Ground actions} \\
\midrule
""" + "\n".join(lines) + r"""
\midrule
Total & """ + str(len(rows)) + r""" & """ + str(sum(1 for r in rows if r["placement"] == "depot")) \
        + r""" & """ + str(sum(1 for r in rows if r["placement"] == "spread")) + r""" & \\
\bottomrule
\end{tabular}
\end{table}
"""
    write("family.tex", body)


def t_coverage(rows):
    lines = []
    for (dom, goal), label in FAMILY.items():
        sub = [r for r in rows if r["domain"] == dom and r["goal"] == goal]
        sv = solved(sub)
        pct = 100.0 * len(sv) / len(sub) if sub else 0
        lines.append(
            f"{label} & {len(sv)}/{len(sub)} & {pct:.0f}\\% & "
            f"{med(r['depth'] for r in sv)} & "
            f"{med((r['leaves'] for r in sv))} & "
            f"{med((r['expanded'] for r in sv))} & "
            f"{med((r['plan_s'] for r in sv), '{:.2f}')} & "
            f"{med((r['ground_s'] for r in sub), '{:.2f}')} \\\\")
    allsv = solved(rows)
    body = r"""\begin{table}[H]
\centering
\small
\caption{Coverage and cost by scenario family, under a 60\,s search budget.
Depth is the depth of the policy tree, leaves its number of terminal branches;
both are medians over solved instances, as are the times.}
\label{tab:coverage}
\begin{tabular}{@{}l r r r r r r r@{}}
\toprule
\textbf{Family} & \textbf{Solved} & \textbf{Cov.} & \textbf{Depth} & \textbf{Leaves}
 & \textbf{Expanded} & \textbf{Search (s)} & \textbf{Ground (s)} \\
\midrule
""" + "\n".join(lines) + r"""
\midrule
All & """ + f"{len(allsv)}/{len(rows)} & {100.0*len(allsv)/len(rows):.0f}\\% & " \
        + f"{med(r['depth'] for r in allsv)} & {med(r['leaves'] for r in allsv)} & " \
        + f"{med(r['expanded'] for r in allsv)} & {med((r['plan_s'] for r in allsv), '{:.2f}')} & " \
        + f"{med((r['ground_s'] for r in rows), '{:.2f}')}" + r""" \\
\bottomrule
\end{tabular}
\end{table}
"""
    write("coverage.tex", body)


def _scale_block(rows, key, label):
    out = []
    for val in sorted({r[key] for r in rows}):
        sub = [r for r in rows if r[key] == val]
        sv = solved(sub)
        out.append(f"{label} $= {val}$ & {len(sv)}/{len(sub)} & "
                   f"{med(r['ground_actions'] for r in sub)} & "
                   f"{med(r['worlds'] for r in sub)} & "
                   f"{med(r['expanded'] for r in sv)} & "
                   f"{med((r['plan_s'] for r in sv), '{:.2f}')} \\\\")
    return out


def t_scaling(rows):
    blocks = (_scale_block(rows, "agents", "Fleet $N$")
              + [r"\addlinespace"] + _scale_block(rows, "bays", "Corridor $L$")
              + [r"\addlinespace"] + _scale_block(rows, "candidates", "Uncertainty $U$"))
    dep, spr = ([r for r in rows if r["placement"] == p] for p in ("depot", "spread"))
    for lbl, sub in (("Depot start", dep), ("Spread start", spr)):
        sv = solved(sub)
        blocks.append(f"{lbl} & {len(sv)}/{len(sub)} & {med(r['ground_actions'] for r in sub)} & "
                      f"{med(r['worlds'] for r in sub)} & {med(r['expanded'] for r in sv)} & "
                      f"{med((r['plan_s'] for r in sv), '{:.2f}')} \\\\")
    blocks.insert(-2, r"\addlinespace")

    body = r"""\begin{table}[H]
\centering
\small
\caption{Scaling along each parameter, aggregated over all other parameters.
Worlds is $|W|$ of the grounded initial model; expanded nodes and search time
are medians over the solved instances in each stratum.}
\label{tab:scaling}
\begin{tabular}{@{}l r r r r r@{}}
\toprule
\textbf{Stratum} & \textbf{Solved} & \textbf{Ground actions} & \textbf{Worlds}
 & \textbf{Expanded} & \textbf{Search (s)} \\
\midrule
""" + "\n".join(blocks) + r"""
\bottomrule
\end{tabular}
\end{table}
"""
    write("scaling.tex", body)


def t_selection(rows):
    combos = {}
    for r in rows:
        if r["heuristic"]:
            combos.setdefault((r["heuristic"], r["strategy"]), []).append(r)
    lines = []
    for (h, s), sub in sorted(combos.items(), key=lambda kv: -len(kv[1])):
        sv = solved(sub)
        lines.append(f"\\texttt{{{h}}} & \\texttt{{{s}}} & {len(sub)} & {len(sv)} & "
                     f"{med(r['expanded'] for r in sv)} \\\\")
    body = r"""\begin{table}[H]
\centering
\small
\caption{Configurations chosen by the planner's automatic selection policy.
No configuration was set by hand: each instance is classified from features of
the grounded task before search starts. The eight timed-out instances are
absent, their output having been discarded when the run was killed.}
\label{tab:selection}
\begin{tabular}{@{}l l r r r@{}}
\toprule
\textbf{Heuristic} & \textbf{Strategy} & \textbf{Instances} & \textbf{Solved} & \textbf{Expanded} \\
\midrule
""" + "\n".join(lines) + r"""
\bottomrule
\end{tabular}
\end{table}
"""
    write("selection.tex", body)


AXIS_OPTS = r"""
    width=0.49\textwidth, height=5.8cm,
    ymode=log, log basis y=10,
    ylabel={expanded nodes (median)},
    grid=major, grid style={draw=rulegrey!60, very thin},
    axis line style={draw=slate}, tick label style={font=\small, color=slate},
    label style={font=\small, color=ink}, title style={font=\small, color=ink},
    legend style={font=\scriptsize, draw=rulegrey, fill=white, at={(0.5,-0.30)},
                  anchor=north, legend columns=2, cells={anchor=west}},
    cycle list={
        {steel, mark=*, mark size=1.6pt, thick},
        {slate, mark=square*, mark size=1.6pt, thick, dashed},
        {steellight, mark=triangle*, mark size=2pt, thick},
        {ink, mark=diamond*, mark size=2pt, thick, dotted}},
"""


def _plot(rows, key, restrict, xlabel, title, xtick, fname, legend=True):
    """Median expanded nodes against `key`, one series per scenario family."""
    series = []
    for (dom, goal), label in FAMILY.items():
        pts = []
        for v in sorted({r[key] for r in rows}):
            sv = solved([r for r in rows if r["domain"] == dom and r["goal"] == goal
                         and r[key] == v and restrict(r)])
            vals = [r["expanded"] for r in sv if r["expanded"]]
            if vals:
                pts.append((v, st.median(vals)))
        if pts:
            coords = " ".join(f"({v},{y:.0f})" for v, y in pts)
            series.append(f"\\addplot coordinates {{{coords}}};"
                          + (f"\n\\addlegendentry{{{label}}}" if legend else ""))
    body = (r"\begin{tikzpicture}" "\n" r"\begin{axis}[" + AXIS_OPTS
            + f"    xlabel={{{xlabel}}}, title={{{title}}}, xtick={{{xtick}}},\n"
            + ("" if legend else "    legend style={draw=none, fill=none},\n")
            + "]\n"
            + "\n".join(series) + "\n" r"\end{axis}" "\n" r"\end{tikzpicture}" "\n")
    write(fname, body)


def t_plot(rows):
    # Fleet size: restricted to L <= 5 so that the corridor-length/uncertainty
    # frontier (L = 6 always carries U = 4) does not swamp the fleet effect.
    _plot(rows, "agents", lambda r: r["bays"] <= 5,
          r"fleet size $N$ \quad{\scriptsize($L\le5$)}",
          "Fleet size", "2,3,4", "plot_agents.tex")
    _plot(rows, "candidates", lambda r: True,
          r"candidate bays $U = |W|$", "Initial uncertainty", "2,3,4",
          "plot_uncertainty.tex", legend=False)


def t_extended(rows):
    """Coverage and cost for the directed / facility families."""
    lines = []
    for (dom, goal), label in EXTENDED.items():
        sub = [r for r in rows if r["domain"] == dom and r["goal"] == goal]
        if not sub:
            continue
        sv = solved(sub)
        sizes = [r["ground_actions"] for r in sub if r["ground_actions"]]
        lines.append(
            f"{label} & {len(sv)}/{len(sub)} & "
            f"{min(sizes)}--{max(sizes)} & "
            f"{med(r['worlds'] for r in sv)} & "
            f"{med(r['depth'] for r in sv)} & "
            f"{med(r['leaves'] for r in sv)} & "
            f"{med(r['expanded'] for r in sv)} & "
            f"{med((r['plan_s'] for r in sv), '{:.2f}')} \\\\")
    body = r"""\begin{table}[H]
\centering
\small
\caption{The extended families. Ground actions are the range over the family;
the remaining columns are medians over solved instances. Compare the corridor
families of Table~\ref{tab:coverage}: addressed communication roughly doubles
the action count at equal fleet size, and the facility family is the only one
whose policies exceed depth~6.}
\label{tab:extended}
\begin{tabular}{@{}l r r r r r r r@{}}
\toprule
\textbf{Family} & \textbf{Solved} & \textbf{Actions} & \textbf{Worlds}
 & \textbf{Depth} & \textbf{Leaves} & \textbf{Expanded} & \textbf{Search (s)} \\
\midrule
""" + "\n".join(lines) + r"""
\bottomrule
\end{tabular}
\end{table}
"""
    write("extended.tex", body)


def main():
    rows = load()
    corridor = [r for r in rows if r["domain"] in CORRIDOR_DOMAINS]
    extended = [r for r in rows if r["domain"] not in CORRIDOR_DOMAINS]

    t_family(corridor)
    t_coverage(corridor)
    t_scaling(corridor)
    t_plot(corridor)
    t_selection(rows)
    if extended:
        t_extended(extended)

    sv = solved(rows)
    print(f"\n{len(sv)}/{len(rows)} solved "
          f"(corridor {len(solved(corridor))}/{len(corridor)}, "
          f"extended {len(solved(extended))}/{len(extended)})")
    for st_ in sorted({r['status'] for r in rows}):
        print(f"  {st_}: {sum(1 for r in rows if r['status'] == st_)}")


if __name__ == "__main__":
    main()
