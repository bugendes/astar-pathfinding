#!/usr/bin/env python3
"""A* pathfinding on 2D grid."""
import heapq

def astar(grid, start, end):
    rows, cols = len(grid), len(grid[0])
    def h(p): return abs(p[0]-end[0]) + abs(p[1]-end[1])
    heap = [(h(start), 0, start)]
    came_from = {}; g = {start: 0}
    while heap:
        _, cost, cur = heapq.heappop(heap)
        if cur == end:
            path = [cur]
            while cur in came_from: cur = came_from[cur]; path.append(cur)
            return path[::-1]
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nb = (cur[0]+dr, cur[1]+dc)
            if 0<=nb[0]<rows and 0<=nb[1]<cols and grid[nb[0]][nb[1]]==0:
                ng = g[cur] + 1
                if ng < g.get(nb, 1e9):
                    g[nb] = ng; came_from[nb] = cur
                    heapq.heappush(heap, (ng+h(nb), ng, nb))
    return None

if __name__ == "__main__":
    grid = [[0,0,0,0,0,0,0,0],[0,1,1,0,0,1,1,0],[0,0,1,0,0,1,0,0],
            [0,0,0,0,0,0,0,0],[0,1,1,1,1,1,0,0],[0,0,0,0,0,0,0,0]]
    path = astar(grid, (0,0), (5,7))
    print(f"A* path: {path} (length={len(path)})")\n