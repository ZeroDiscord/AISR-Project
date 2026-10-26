"""Tomb layouts: the single source of truth for every world.

Each layout lists the obstacles the lidar can see (walls, pillars, sarcophagi,
urns, ...) as 2D footprints extruded upwards from the floor. world_gen.py turns
a layout into a Webots .wbt file AND a ground-truth occupancy grid, so the map
the robot builds can always be scored against the true tomb.

Rules that keep the ground truth honest:
  * every obstacle is a vertical prism at least OBSTACLE_MIN_H tall, so the
    lidar plane (~0.31 m above the floor) cuts it exactly at its footprint;
  * purely decorative parts (torches, statue heads, coins on top of piles)
    sit above the lidar plane and are not listed as obstacles.

Coordinates are metres in the Webots ENU frame (x east, y north, z up),
with the world centred on the origin.
"""

import math
import random
from dataclasses import dataclass, field

WALL_T = 0.2          # wall thickness (m)
WALL_H = 1.5          # wall height (m)
OBSTACLE_MIN_H = 0.6  # every obstacle is at least this tall


@dataclass
class Box:
    x: float
    y: float
    sx: float
    sy: float
    h: float
    yaw: float = 0.0
    look: str = "stone"
    kind: str = "block"


@dataclass
class Cyl:
    x: float
    y: float
    r: float
    h: float
    look: str = "stone"
    kind: str = "pillar"


@dataclass
class Torch:
    x: float
    y: float
    facing: float  # direction the torch points into the room (rad)


@dataclass
class Layout:
    name: str
    title: str
    story: str
    inner_w: float          # interior size along x
    inner_h: float          # interior size along y
    start: tuple            # (x, y, yaw) of the robot
    obstacles: list = field(default_factory=list)
    torches: list = field(default_factory=list)


def outer_walls(w, h, look="glyphs"):
    """Four walls whose inner faces enclose a w x h room centred on the origin."""
    t = WALL_T
    return [
        Box(0, h / 2 + t / 2, w + 2 * t, t, WALL_H, look=look, kind="wall"),
        Box(0, -h / 2 - t / 2, w + 2 * t, t, WALL_H, look=look, kind="wall"),
        # side walls overlap the corners so no seam is left for lidar rays to slip through
        Box(w / 2 + t / 2, 0, t, h + 2 * t, WALL_H, look=look, kind="wall"),
        Box(-w / 2 - t / 2, 0, t, h + 2 * t, WALL_H, look=look, kind="wall"),
    ]


def wall_torches(w, h, xs=(), ys=()):
    """Torches on the north/south walls at the given x, east/west walls at the given y."""
    torches = []
    for x in xs:
        torches.append(Torch(x, h / 2 - 0.05, -math.pi / 2))
        torches.append(Torch(x, -h / 2 + 0.05, math.pi / 2))
    for y in ys:
        torches.append(Torch(w / 2 - 0.05, y, math.pi))
        torches.append(Torch(-w / 2 + 0.05, y, 0.0))
    return torches


# --------------------------------------------------------------------------
# World 1 - Antechamber: one hall, four pillars, guardians at the sealed door
# --------------------------------------------------------------------------
def antechamber():
    w, h = 8.0, 6.0
    obs = outer_walls(w, h)
    for x in (-1.5, 1.5):
        for y in (-1.5, 1.5):
            obs.append(Cyl(x, y, 0.3, WALL_H, look="stone", kind="pillar"))
    # obsidian guardians flanking the sealed door on the east wall
    obs.append(Box(3.3, 2.2, 0.6, 0.6, 1.8, look="obsidian", kind="statue"))
    obs.append(Box(3.3, -2.2, 0.6, 0.6, 1.8, look="obsidian", kind="statue"))
    # sealed door slab, flush with the east wall (decor only, sits inside the wall)
    obs.append(Box(w / 2 + WALL_T / 2 - 0.01, 0, WALL_T + 0.02, 1.4, 1.3, look="door", kind="door"))
    obs.append(Box(0.0, 0.0, 1.0, 0.6, 0.7, look="stone", kind="altar"))
    obs.append(Cyl(-3.4, 2.4, 0.2, 0.6, look="clay", kind="urn"))
    obs.append(Cyl(-3.4, -2.4, 0.2, 0.6, look="clay", kind="urn"))
    obs.append(Box(2.2, -0.7, 0.8, 0.5, 0.6, yaw=0.4, look="stone", kind="block"))
    return Layout(
        name="antechamber",
        title="Antechamber of the Scarab King",
        story="The entrance hall. Four pillars, an offering altar and two "
              "obsidian guardians watching the sealed door.",
        inner_w=w, inner_h=h, start=(-3.0, 0.0, 0.0),
        obstacles=obs,
        torches=wall_torches(w, h, xs=(-2.5, 2.5), ys=(0.0,)),
    )


# --------------------------------------------------------------------------
# World 2 - Labyrinth: 5 x 5 maze of 2 m cells with dead ends and loops
# --------------------------------------------------------------------------
MAZE_N = 6
MAZE_CELL = 1.6
MAZE_SEED = 7
MAZE_EXTRA_OPENINGS = 2


def carve_maze(n, seed, extra):
    """Randomised DFS maze; returns the set of open passages between cells."""
    rng = random.Random(seed)
    seen = {(0, 0)}
    stack = [(0, 0)]
    open_edges = set()
    while stack:
        cx, cy = stack[-1]
        nbrs = [(cx + dx, cy + dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= cx + dx < n and 0 <= cy + dy < n and (cx + dx, cy + dy) not in seen]
        if not nbrs:
            stack.pop()
            continue
        nxt = rng.choice(nbrs)
        open_edges.add(frozenset(((cx, cy), nxt)))
        seen.add(nxt)
        stack.append(nxt)
    # knock out a few extra walls so the maze has loops, not only one route
    closed = [frozenset(((x, y), (x + dx, y + dy)))
              for x in range(n) for y in range(n) for dx, dy in ((1, 0), (0, 1))
              if x + dx < n and y + dy < n
              and frozenset(((x, y), (x + dx, y + dy))) not in open_edges]
    rng.shuffle(closed)
    open_edges.update(closed[:extra])
    return open_edges


def labyrinth():
    n, c = MAZE_N, MAZE_CELL
    w = h = n * c
    obs = outer_walls(w, h)
    open_edges = carve_maze(n, MAZE_SEED, MAZE_EXTRA_OPENINGS)

    def centre(i, j):
        return -w / 2 + c * (i + 0.5), -h / 2 + c * (j + 0.5)

    for i in range(n):
        for j in range(n):
            x0, y0 = -w / 2 + c * i, -h / 2 + c * j
            if i + 1 < n and frozenset(((i, j), (i + 1, j))) not in open_edges:
                obs.append(Box(x0 + c, y0 + c / 2, WALL_T, c + WALL_T, WALL_H, look="stone", kind="wall"))
            if j + 1 < n and frozenset(((i, j), (i, j + 1))) not in open_edges:
                obs.append(Box(x0 + c / 2, y0 + c, c + WALL_T, WALL_T, WALL_H, look="stone", kind="wall"))

    # an urn at the far end of every dead end (cells with a single opening),
    # except the start cell and the central cell that holds the death mask
    mask_cell = (n // 2, n // 2)
    for i in range(n):
        for j in range(n):
            if (i, j) in ((0, 0), mask_cell):
                continue
            exits = [(i + dx, j + dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                     if frozenset(((i, j), (i + dx, j + dy))) in open_edges]
            if len(exits) == 1:
                cx, cy = centre(i, j)
                ex, ey = centre(*exits[0])
                d = math.hypot(ex - cx, ey - cy)
                obs.append(Cyl(cx - 0.55 * (ex - cx) / d, cy - 0.55 * (ey - cy) / d,
                               0.18, 0.6, look="clay", kind="urn"))

    # golden death mask on a pedestal in the central cell
    mx, my = centre(*mask_cell)
    obs.append(Cyl(mx, my, 0.2, 0.9, look="stone", kind="mask"))

    # start in the south-west cell, facing its open side
    sx, sy = centre(0, 0)
    yaw = 0.0 if frozenset(((0, 0), (1, 0))) in open_edges else math.pi / 2
    return Layout(
        name="labyrinth",
        title="Labyrinth of Forgotten Priests",
        story="Narrow passages, dead ends guarded by burial urns, and the "
              "golden death mask hidden somewhere in the middle.",
        inner_w=w, inner_h=h, start=(sx, sy, yaw),
        obstacles=obs,
        torches=wall_torches(w, h, xs=(-c * 1.5, c * 1.5), ys=(-c * 1.5, c * 1.5)),
    )


# --------------------------------------------------------------------------
# World 3 - Treasury: cluttered hall plus a vault behind a doorway
# --------------------------------------------------------------------------
def treasury():
    w, h = 12.0, 10.0
    obs = outer_walls(w, h)
    # partition wall at x = 3 with a 1.6 m doorway into the vault
    px, door = 3.0, 1.6
    seg = (h - door) / 2 + WALL_T          # each half reaches into the outer wall
    obs.append(Box(px, h / 2 + WALL_T - seg / 2, WALL_T, seg, WALL_H, look="glyphs", kind="wall"))
    obs.append(Box(px, -h / 2 - WALL_T + seg / 2, WALL_T, seg, WALL_H, look="glyphs", kind="wall"))

    # main hall
    for x in (-4.0, -1.5, 1.0):
        for y in (-3.2, 3.2):
            obs.append(Cyl(x, y, 0.3, WALL_H, look="stone", kind="pillar"))
    obs.append(Box(-3.5, 1.2, 2.0, 0.8, 0.8, look="painted", kind="sarcophagus"))
    obs.append(Box(-3.5, -1.2, 2.0, 0.8, 0.8, look="painted", kind="sarcophagus"))
    obs.append(Box(0.0, 0.0, 2.2, 0.9, 0.9, yaw=0.5, look="gold", kind="sarcophagus"))
    for x, y in ((-5.4, 4.4), (-5.4, -4.4), (-2.0, 4.5), (0.5, -4.5), (2.3, 4.4), (2.3, -1.8)):
        obs.append(Cyl(x, y, 0.2, 0.6, look="clay", kind="urn"))
    obs.append(Box(-1.0, 1.6, 0.7, 0.45, 0.6, yaw=0.3, look="wood", kind="chest"))
    obs.append(Box(1.5, -2.2, 0.7, 0.45, 0.6, yaw=-0.4, look="wood", kind="chest"))
    obs.append(Box(-0.6, -2.0, 0.9, 0.5, 0.6, yaw=1.0, look="stone", kind="block"))

    # the vault
    for x, y in ((4.5, 3.0), (5.2, -3.5), (4.2, -2.0)):
        obs.append(Cyl(x, y, 0.35, 0.6, look="gold", kind="coins"))
    obs.append(Box(5.3, 0.0, 0.8, 0.8, 1.6, look="gold", kind="statue"))
    obs.append(Box(4.0, 4.0, 0.7, 0.45, 0.6, look="wood", kind="chest"))
    obs.append(Box(5.4, 2.0, 0.7, 0.45, 0.6, yaw=0.5, look="wood", kind="chest"))
    for x, y in ((3.6, -4.4), (5.6, 4.4)):
        obs.append(Cyl(x, y, 0.2, 0.6, look="clay", kind="urn"))

    return Layout(
        name="treasury",
        title="Treasury of the Pharaoh",
        story="A crowded hall of sarcophagi, chests and urns, and behind a "
              "narrow doorway the vault with the Pharaoh's golden statue.",
        inner_w=w, inner_h=h, start=(-5.0, 0.0, 0.0),
        obstacles=obs,
        torches=wall_torches(w, h, xs=(-4.5, -1.0, 4.5), ys=(-2.5, 2.5)),
    )


LAYOUTS = {f.__name__: f for f in (antechamber, labyrinth, treasury)}


def get_layout(name):
    return LAYOUTS[name]()
