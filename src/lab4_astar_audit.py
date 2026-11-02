# Lab 4 - heuristic checks, A* with a trace, UCS and the overestimate test
# astar_traced is the manual A* with print/logging added, the logic is the same.
# I count a node as expanded only when its successors are looked at
# (goal and stale entries are not counted).
import heapq

try:
    from .astar_supplied import astar, g as GRAPH, h as H
except ImportError:                   # so it also runs as: python src/lab4_astar_audit.py
    from astar_supplied import astar, g as GRAPH, h as H


def astar_traced(graph, h, start, goal):
    pq = [(h[start], 0, start, [start])]; best = {start: 0}
    rows, processed, expanded = [], [], []
    while pq:
        f, g, node, path = heapq.heappop(pq)
        if node == goal:
            processed.append(node)
            rows.append(dict(kind="goal", node=node, g=g, h=h[node], f=f, path=path, pushed=[], frontier=sorted(pq)))
            return dict(cost=g, path=path, processed=processed, expanded=expanded, rows=rows)
        if g != best[node]:
            rows.append(dict(kind="stale", node=node, g=g, h=h[node], f=f, path=path, pushed=[], frontier=sorted(pq)))
            continue
        processed.append(node); expanded.append(node)
        pushed = []
        for nxt, w in graph.get(node, []):
            ng = g + w
            if ng < best.get(nxt, float("inf")):
                best[nxt] = ng
                heapq.heappush(pq, (ng + h[nxt], ng, nxt, path + [nxt]))
                pushed.append("%s: g=%d+%d=%d, h=%d, f=%d+%d=%d" % (nxt, g, w, ng, h[nxt], ng, h[nxt], ng + h[nxt]))
            else:
                pushed.append("%s: g=%d+%d=%d not better than %d (not pushed)" % (nxt, g, w, ng, best[nxt]))
        rows.append(dict(kind="expand", node=node, g=g, h=h[node], f=f, path=path, pushed=pushed, frontier=sorted(pq)))
    return dict(cost=None, path=None, processed=processed, expanded=expanded, rows=rows)


def fmt_frontier(fr):
    return "[" + ", ".join("%s(f=%d,g=%d)" % (n, f, g) for f, g, n, _ in fr) + "]" if fr else "[]"


def print_run(title, graph, h, start="S", goal="G"):
    res = astar_traced(graph, h, start, goal)
    print(title)
    print("  h =", h)
    print("  step | action | node | g | h | f | successors generated | frontier after (pop order)")
    for i, r in enumerate(res["rows"], 1):
        act = {"expand": "EXPAND", "goal": "GOAL (not counted)", "stale": "STALE (ignored)"}[r["kind"]]
        print("  %d | %s | %s | %d | %d | %d | %s | %s" % (i, act, r["node"], r["g"], r["h"], r["f"],
              "; ".join(r["pushed"]) or "-", fmt_frontier(r["frontier"])))
    total, expr = path_cost(graph, res["path"])
    print("  returned path: %s | reported cost %d | independent sum %s = %d"
          % ("-".join(res["path"]), res["cost"], expr, total))
    print("  processed-node trace: %s | expansions (goal excluded): %d -> %s"
          % (", ".join(res["processed"]), len(res["expanded"]), ", ".join(res["expanded"])))
    return res


def path_cost(graph, path):
    parts, total = [], 0
    for u, v in zip(path, path[1:]):
        w = dict(graph[u])[v]; parts.append(str(w)); total += w
    return total, " + ".join(parts)


def true_remaining(graph, goal):
    zero = {n: 0 for n in graph}
    out = {}
    for n in graph:
        r = astar(graph, zero, n, goal)
        out[n] = r[0] if r else float("inf")
    return out


def audit(graph, h, goal="G"):
    hstar = true_remaining(graph, goal)
    print("Admissibility check  h(n) <= h*(n):")
    print("  node | h(n) | true remaining h*(n) | admissible")
    for n in graph:
        print("  %s | %d | %s | %s" % (n, h[n], hstar[n], "yes" if h[n] <= hstar[n] else "NO (overestimate)"))
    print("Consistency check  h(u) <= cost(u,v) + h(v)  for every directed edge:")
    print("  edge | h(u) | cost + h(v) | consistent")
    for u, edges in graph.items():
        for v, w in edges:
            ok = h[u] <= w + h[v]
            print("  %s->%s | %d | %d + %d = %d | %s" % (u, v, h[u], w, h[v], w + h[v], "yes" if ok else "NO"))
    return hstar


def main():
    print("=" * 72)
    print("LAB 4  Informed search: heuristic audit, A*, UCS control, overestimate")
    print("=" * 72)
    print("Supplied graph:", GRAPH)
    print("Supplied heuristic:", H)
    print("Supplied astar(g,h,'S','G') ->", astar(GRAPH, H, "S", "G"))
    print()
    audit(GRAPH, H)
    print()
    a = print_run("Task 1/2  A* with the supplied heuristic:", GRAPH, H)
    print()
    zero = {n: 0 for n in H}
    u = print_run("Task 2  UCS control (all h = 0, same implementation):", GRAPH, zero)
    print()
    over = dict(H); over["C"] = 10
    print("Task 3  Overestimate: h(C) changed from 2 to 10 (true remaining cost of C = 3); nothing else changed")
    audit(GRAPH, over)
    o = print_run("A* with h(C)=10:", GRAPH, over)
    print()
    print("Comparison table:")
    print("  run | path | cost | processed trace | expansions (goal excluded)")
    for name, r in (("A* supplied h", a), ("UCS (h=0)", u), ("A* h(C)=10", o)):
        print("  %s | %s | %d | %s | %d" % (name, "-".join(r["path"]), r["cost"], ", ".join(r["processed"]), len(r["expanded"])))
    print()
    other = dict(H); other["B"] = 10
    print("Supplementary check (not the Task 3 run): a different single overestimate, h(B)=10 (true 5):")
    b = print_run("A* with h(B)=10:", GRAPH, other)
    print("  -> this overestimate happens to leave the path and cost unchanged, so an unchanged run does not prove a heuristic safe.")


if __name__ == "__main__":
    main()
