# A* Pathfinding

A heuristic-guided search algorithm that finds the optimal shortest path. Combines Dijkstra's guaranteed optimality with greedy search's speed.

## How It Works

A* evaluates nodes using `f(n) = g(n) + h(n)`:

- **g(n):** Actual cost from start to n (known).
- **h(n):** Heuristic estimate from n to goal (must be admissible — never overestimate).
- **f(n):** Estimated total cost through n.

The algorithm maintains a priority queue sorted by f. At each step, it expands the node with the lowest f. If h is admissible, A* is optimal. If h is also consistent (h(n) ≤ cost(n,n') + h(n')), A* never re-expands nodes.

**Common Heuristics for Grids:**
- Manhattan: |dx| + |dy| — for 4-directional movement.
- Euclidean: √(dx² + dy²) — for any-angle movement.
- Chebyshev: max(|dx|, |dy|) — for 8-directional with uniform cost.

## Complexity

| Metric | Value | Notes |
|--------|-------|-------|
| Time   | O(b^d) worst case | b = branching factor, d = depth |
| Space  | O(b^d) | Stores all generated nodes |
| Optimal? | Yes | If h is admissible |

In practice, with a good heuristic, A* explores far fewer nodes than Dijkstra's.

## Applications

**Game AI:** Pathfinding for NPCs, RTS units, open-world navigation meshes. Most game engines use A* with hierarchical abstractions.

**Robotics:** Motion planning for autonomous robots. A* on configuration space with collision checking.

**GPS Navigation:** Route planning on road graphs. Contraction Hierarchies + A* gives sub-millisecond queries on continental road networks.

**Puzzle Solving:** 15-puzzle, Rubik's Cube, Sokoban — A* with pattern database heuristics solves optimally.
