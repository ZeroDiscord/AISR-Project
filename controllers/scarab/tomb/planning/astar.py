"""A* search on the 8-connected configuration-space grid (Hart, Nilsson & Raphael, 1968).

Step cost = move length (1 or sqrt 2 cells) x (1 + clearance penalty of the
cell entered). The octile-distance heuristic never overestimates this cost, so
the returned path is optimal for the costmap. Diagonal moves may not cut the
corner of a blocked cell.
"""

import heapq
import math

SQRT2 = math.sqrt(2.0)
MOVES = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2))


def octile(r, c, gr, gc):
    dr, dc = abs(r - gr), abs(c - gc)
    return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)


def astar(costmap, start, goal):
    """Cells [(row, col), ...] from start to goal, or None if no path exists.

    Returns (path, expanded) where expanded counts the nodes popped, which is
    useful when comparing planners in the report.
    """
    trav = costmap.traversable
    if not (trav[start] and trav[goal]):
        return None, 0
    # work in the bounding box of the traversable area, as plain Python lists for speed
    rows = trav.any(axis=1).nonzero()[0]
    cols = trav.any(axis=0).nonzero()[0]
    r0, c0 = int(rows[0]), int(cols[0])
    ok = trav[r0:rows[-1] + 1, c0:cols[-1] + 1].tolist()
    pen = costmap.penalty[r0:rows[-1] + 1, c0:cols[-1] + 1].tolist()
    nr, nc = len(ok), len(ok[0])
    sr, sc = start[0] - r0, start[1] - c0
    gr, gc = goal[0] - r0, goal[1] - c0

    g = {(sr, sc): 0.0}
    parent = {(sr, sc): None}
    heap = [(octile(sr, sc, gr, gc), 0.0, sr, sc)]
    closed = set()
    expanded = 0
    while heap:
        _, gcost, r, c = heapq.heappop(heap)
        if (r, c) in closed:
            continue
        closed.add((r, c))
        expanded += 1
        if r == gr and c == gc:
            path = []
            node = (r, c)
            while node is not None:
                path.append((node[0] + r0, node[1] + c0))
                node = parent[node]
            return path[::-1], expanded
        for dr, dc, length in MOVES:
            rr, cc = r + dr, c + dc
            if not (0 <= rr < nr and 0 <= cc < nc) or not ok[rr][cc] or (rr, cc) in closed:
                continue
            if dr and dc and not (ok[r][cc] and ok[rr][c]):
                continue                      # no corner cutting
            new_g = gcost + length * (1.0 + pen[rr][cc])
            if new_g < g.get((rr, cc), math.inf):
                g[(rr, cc)] = new_g
                parent[(rr, cc)] = (r, c)
                heapq.heappush(heap, (new_g + octile(rr, cc, gr, gc), new_g, rr, cc))
    return None, expanded
