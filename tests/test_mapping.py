"""Unit tests for the OccupancyGrid mapping subsystem."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "controllers", "scarab"))

from tomb import config
from tomb.mapping import FREE, OCCUPIED, UNKNOWN, OccupancyGrid


def test_coordinate_transforms():
    grid = OccupancyGrid(res=0.1, n=100, origin=(-5.0, -5.0))
    # Test origin
    col, row = grid.to_cell(-5.0, -5.0)
    assert col == 0 and row == 0
    x, y = grid.to_world(0, 0)
    assert math.isclose(x, -4.95, abs_tol=1e-5)
    assert math.isclose(y, -4.95, abs_tol=1e-5)

    # Test center
    col, row = grid.to_cell(0.0, 0.0)
    assert col == 50 and row == 50
    wx, wy = grid.to_world(col, row)
    assert math.isclose(wx, 0.05, abs_tol=1e-5)
    assert math.isclose(wy, 0.05, abs_tol=1e-5)

    # Test array inputs
    cols, rows = grid.to_cell([-5.0, 0.0], [-5.0, 0.0])
    assert np.array_equal(cols, [0, 50])
    assert np.array_equal(rows, [0, 50])


def test_inverse_sensor_model_single_beam():
    grid = OccupancyGrid()
    # Initial state should be completely UNKNOWN
    assert (grid.state() == UNKNOWN).all()
    assert (grid.log_odds == 0.0).all()

    # Cast a single beam at distance 2.0 m along +x
    ranges = np.array([2.0])
    angles = np.array([0.0])
    grid.update(0.0, 0.0, 0.0, ranges, angles)

    # Endpoint (2.0, 0.0) should have positive log-odds
    occ_col, occ_row = grid.to_cell(2.0, 0.0)
    assert grid.log_odds[occ_row, occ_col] > 0.0

    # Intermediate cell (1.0, 0.0) should have negative log-odds
    free_col, free_row = grid.to_cell(1.0, 0.0)
    assert grid.log_odds[free_row, free_col] < 0.0

    # Cell beyond endpoint (3.0, 0.0) should be untouched
    untouched_col, untouched_row = grid.to_cell(3.0, 0.0)
    assert grid.log_odds[untouched_row, untouched_col] == 0.0


def test_log_odds_clamping():
    grid = OccupancyGrid()
    ranges = np.array([2.0])
    angles = np.array([0.0])

    # Hit 50 times in the same spot
    for _ in range(50):
        grid.update(0.0, 0.0, 0.0, ranges, angles)

    occ_col, occ_row = grid.to_cell(2.0, 0.0)
    assert math.isclose(grid.log_odds[occ_row, occ_col], config.L_MAX, abs_tol=1e-5)

    free_col, free_row = grid.to_cell(1.0, 0.0)
    assert math.isclose(grid.log_odds[free_row, free_col], config.L_MIN, abs_tol=1e-5)


def test_state_classification():
    grid = OccupancyGrid()
    # At L=0, p=0.5 -> UNKNOWN
    assert grid.state()[0, 0] == UNKNOWN

    # Set cell to strong occupied log-odds
    grid.log_odds[10, 10] = config.L_MAX
    assert grid.state()[10, 10] == OCCUPIED

    # Set cell to strong free log-odds
    grid.log_odds[10, 20] = config.L_MIN
    assert grid.state()[10, 20] == FREE


def test_known_area_and_quantities():
    grid = OccupancyGrid(res=0.1, n=50)
    # Set 10 free cells and 5 occupied cells
    for i in range(10):
        grid.log_odds[5, i] = config.L_MIN
    for i in range(5):
        grid.log_odds[15, i] = config.L_MAX

    cell_area = 0.1 * 0.1
    assert math.isclose(grid.free_area(), 10 * cell_area, abs_tol=1e-5)
    assert math.isclose(grid.occupied_area(), 5 * cell_area, abs_tol=1e-5)
    assert math.isclose(grid.known_area(), 15 * cell_area, abs_tol=1e-5)


def test_shannon_entropy():
    grid = OccupancyGrid(res=0.1, n=20, origin=(-1.0, -1.0))
    # 20x20 = 400 cells. All at p=0.5 -> 1 bit per cell -> 400 bits
    h_initial = grid.entropy()
    assert math.isclose(h_initial, 400.0, rel_tol=1e-4)

    # Updating with sensor observations resolves uncertainty: entropy MUST decrease
    ranges = np.linspace(0.4, 0.7, 30)
    angles = np.linspace(-math.pi / 2, math.pi / 2, 30)
    grid.update(0.0, 0.0, 0.0, ranges, angles)

    h_updated = grid.entropy()
    assert h_updated < h_initial


def test_reset_and_copy():
    grid = OccupancyGrid(res=0.1, n=10)
    grid.log_odds[2, 2] = 3.5

    clone = grid.copy()
    assert math.isclose(clone.log_odds[2, 2], 3.5)

    clone.log_odds[2, 2] = 1.0
    assert math.isclose(grid.log_odds[2, 2], 3.5)  # independent memory

    grid.reset()
    assert (grid.log_odds == 0.0).all()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
