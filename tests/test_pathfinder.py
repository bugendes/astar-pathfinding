"""Tests for A* pathfinding."""

import pytest
from astar import astar, GridGraph
from astar.pathfinder import manhattan


class TestAStar:
    def test_simple_path(self):
        g = GridGraph(5, 5)
        path = astar((0,0), (4,4), lambda n: g.neighbors(*n), manhattan)
        assert path is not None
        assert path[0] == (0,0)
        assert path[-1] == (4,4)

    def test_wall_blocks(self):
        g = GridGraph(5, 5)
        for y in range(5):
            g.add_wall(2, y)
        path = astar((0,0), (4,0), lambda n: g.neighbors(*n), manhattan)
        assert path is None  # wall completely blocks

    def test_no_path_needed(self):
        g = GridGraph(5, 5)
        path = astar((1,1), (1,1), lambda n: g.neighbors(*n), manhattan)
        assert path == [(1,1)]

    def test_optimal_length(self):
        g = GridGraph(3, 3)
        path = astar((0,0), (2,2), lambda n: g.neighbors(*n), manhattan)
        assert len(path) == 5  # (0,0)->(1,0)->(2,0)->(2,1)->(2,2) or similar
