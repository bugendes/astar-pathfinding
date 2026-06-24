"""A* search algorithm implementation.

A* finds the shortest path from start to goal using:
  f(n) = g(n) + h(n)

where g(n) = cost from start, h(n) = admissible heuristic estimate to goal.

If h never overestimates, A* is guaranteed to find the optimal path.
With h=0, A* degrades to Dijkstra's. With h=actual cost, it's greedy.

Includes a 2D grid graph with Manhattan/Euclidean heuristics.

Used in: game AI, robotics, GPS navigation, network routing.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Generic, Hashable, List, Optional, Set, Tuple, TypeVar

T = TypeVar("T")


@dataclass(order=True)
class _Entry:
    f: float
    node: Hashable = field(compare=False)
    g: float = field(compare=False)


class GridGraph:
    """2D grid with obstacles. Supports 4- and 8-directional movement."""

    def __init__(self, width: int, height: int, allow_diagonal: bool = False) -> None:
        self.width = width
        self.height = height
        self.allow_diagonal = allow_diagonal
        self.walls: Set[Tuple[int, int]] = set()

    def add_wall(self, x: int, y: int) -> None:
        self.walls.add((x, y))

    def is_valid(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height and (x, y) not in self.walls

    def neighbors(self, x: int, y: int) -> List[Tuple[Tuple[int, int], float]]:
        """Return (neighbor, cost) pairs."""
        dirs = [(0,1),(0,-1),(1,0),(-1,0)]
        if self.allow_diagonal:
            dirs += [(1,1),(1,-1),(-1,1),(-1,-1)]

        result = []
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                cost = math.sqrt(dx*dx + dy*dy)
                result.append(((nx, ny), cost))
        return result


def manhattan(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def astar(
    start: T,
    goal: T,
    neighbors: Callable[[T], List[Tuple[T, float]]],
    heuristic: Callable[[T, T], float],
) -> Optional[List[T]]:
    """Find the shortest path from start to goal.

    Args:
        start: Start node.
        goal: Goal node.
        neighbors: Function returning (neighbor, cost) pairs.
        heuristic: Admissible heuristic function h(node, goal).

    Returns:
        List of nodes from start to goal, or None if no path exists.
    """
    open_set: List[_Entry] = []
    g_score: Dict[T, float] = {start: 0}
    came_from: Dict[T, T] = {}
    closed: Set[T] = set()

    heapq.heappush(open_set, _Entry(heuristic(start, goal), start, 0))

    while open_set:
        entry = heapq.heappop(open_set)
        current = entry.node

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        if current in closed:
            continue
        closed.add(current)

        for neighbor, cost in neighbors(current):
            if neighbor in closed:
                continue
            tentative_g = g_score[current] + cost
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, _Entry(f, neighbor, tentative_g))

    return None  # no path
