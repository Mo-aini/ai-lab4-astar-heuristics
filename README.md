# AI Lab 4 - A* Search and Heuristics

Lab 4 for the AI lab course (CS4944). I traced A* by hand, checked the heuristic, compared A* with UCS, and tried an overestimating heuristic. Everything uses the graph and A* code from the manual.

## Files

- `main.py` runs everything
- `src/astar_supplied.py` A*, graph and heuristic from the manual
- `src/lab4_astar_audit.py` my code for the tasks (traced A*, UCS, the checks)
- `outputs/output.txt` output of my run
- `report/Lab4_Report_Mohammed_Emad_ALaini.docx` the report

## How to run

Only Python 3 is needed, no extra packages.

```
python main.py
```

## Results

Graph: S->A 1, S->B 4, A->C 2, B->G 5, C->G 3. h = {S:5, A:4, B:4, C:2, G:0}. The heuristic is admissible and consistent.

First three expansions:

| Expansion | Node | g | h | f | Frontier after |
|-----------|------|---|---|---|----------------|
| 1 | S | 0 | 5 | 5 | A(f5,g1), B(f8,g4) |
| 2 | A | 1 | 4 | 5 | C(f5,g3), B(f8,g4) |
| 3 | C | 3 | 2 | 5 | G(f6,g6), B(f8,g4) |

| Run | Path | Cost | Processed | Expansions |
|-----|------|------|-----------|------------|
| A* | S-A-C-G | 6 | S, A, C, G | 3 |
| UCS (h = 0) | S-A-C-G | 6 | S, A, C, B, G | 4 |
| A* with h(C) = 10 | S-B-G | 9 | S, A, B, G | 3 |

I only count a node as expanded when its successors are looked at. With h(C) = 10 (the real cost is 3) A* returns a worse path with cost 9. Setting h(B) = 10 instead changes nothing, so one run that looks fine does not prove the heuristic is fine.

## Author

Mohammed Emad ALaini

MIT License, see LICENSE.
