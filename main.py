#!/usr/bin/env python3
"""A* Pathfinding demo."""

from astar import astar, GridGraph
from astar.pathfinder import manhattan


def main():
    print("=== A* Pathfinding Demo ===
")

    g = GridGraph(10, 10)
    # Add a wall
    for y in range(3, 8):
        g.add_wall(5, y)

    path = astar((0, 0), (9, 9), lambda n: g.neighbors(*n), manhattan)
    print(f"Path from (0,0) to (9,9) with wall at x=5:")
    print(f"  Length: {len(path)}")
    print(f"  Path: {path}")


if __name__ == "__main__":
    main()
