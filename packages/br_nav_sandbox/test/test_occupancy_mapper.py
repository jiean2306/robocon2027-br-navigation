"""Unit tests for Bayesian log-odds LiDAR occupancy grid mapping and Bresenham raycasting."""

import math
import pytest

from br_nav_sandbox.occupancy_mapper import (
    LogOddsOccupancyMapper,
    bresenham_line,
    log_odds_to_prob,
    prob_to_log_odds,
)


def test_prob_log_odds_round_trip():
    for p in (0.05, 0.20, 0.30, 0.50, 0.70, 0.85, 0.95):
        l_val = prob_to_log_odds(p)
        p_recovered = log_odds_to_prob(l_val)
        assert abs(p - p_recovered) < 1e-12
    assert abs(prob_to_log_odds(0.5)) < 1e-12


def test_prob_out_of_range_raises():
    with pytest.raises(ValueError):
        prob_to_log_odds(0.0)
    with pytest.raises(ValueError):
        prob_to_log_odds(1.0)


def test_bresenham_line_cardinal_and_diagonal():
    horiz = bresenham_line(2, 5, 6, 5)
    assert horiz == [(2, 5), (3, 5), (4, 5), (5, 5), (6, 5)]

    vert = bresenham_line(4, 7, 4, 3)
    assert vert == [(4, 7), (4, 6), (4, 5), (4, 4), (4, 3)]

    diag = bresenham_line(1, 1, 4, 4)
    assert diag == [(1, 1), (2, 2), (3, 3), (4, 4)]


def test_log_odds_ray_hit_marks_free_and_occupied():
    mapper = LogOddsOccupancyMapper(
        grid_resolution=0.10,
        width_m=2.0,
        height_m=2.0,
        origin_x=0.0,
        origin_y=0.0,
        p_occ=0.75,
        p_free=0.25,
        occ_prob_threshold=0.65,
        free_prob_threshold=0.35,
    )
    # Initial grid is all unknown (-1)
    initial_data = mapper.to_occupancy_grid_data()
    assert all(v == -1 for v in initial_data)

    # Fire 3 repeated rays from (0.25, 0.55) along +X (angle=0) hitting an obstacle at range 1.0 m -> (1.25, 0.55)
    for _ in range(3):
        mapper.update_ray(
            sensor_x=0.25,
            sensor_y=0.55,
            beam_angle=0.0,
            range_m=1.0,
            range_min=0.05,
            range_max=5.0,
        )

    grid_data = mapper.to_occupancy_grid_data()
    start_cx, start_cy = mapper.world_to_cell(0.25, 0.55)
    hit_cx, hit_cy = mapper.world_to_cell(1.25, 0.55)
    behind_cx, behind_cy = mapper.world_to_cell(1.65, 0.55)

    # Traversed cells from start_cx to hit_cx - 1 must be free (0)
    for cx in range(start_cx, hit_cx):
        assert grid_data[start_cy * mapper.width + cx] == 0

    # Hit endpoint cell must be occupied (100)
    assert grid_data[hit_cy * mapper.width + hit_cx] == 100

    # Unseen cell behind obstacle must remain unknown (-1)
    assert grid_data[behind_cy * mapper.width + behind_cx] == -1


def test_max_range_ray_clears_without_occupied_endpoint():
    mapper = LogOddsOccupancyMapper(
        grid_resolution=0.10,
        width_m=2.0,
        height_m=1.0,
        origin_x=0.0,
        origin_y=0.0,
    )
    for _ in range(3):
        mapper.update_ray(
            sensor_x=0.15,
            sensor_y=0.55,
            beam_angle=0.0,
            range_m=1.50,
            range_min=0.05,
            range_max=1.50,
        )
    grid_data = mapper.to_occupancy_grid_data()
    assert 100 not in grid_data
    end_cx, end_cy = mapper.world_to_cell(1.65, 0.55)
    assert grid_data[end_cy * mapper.width + end_cx] == 0


def test_clamping_allows_dynamic_obstacle_clearing():
    mapper = LogOddsOccupancyMapper(
        grid_resolution=0.10,
        width_m=2.0,
        height_m=1.0,
        origin_x=0.0,
        origin_y=0.0,
        l_min=-2.0,
        l_max=2.0,
    )
    # Saturate cell at (1.05, 0.55) as occupied
    for _ in range(10):
        mapper.update_ray(0.15, 0.55, 0.0, range_m=0.90, range_min=0.05, range_max=5.0)
    hit_cx, hit_cy = mapper.world_to_cell(1.05, 0.55)
    assert abs(mapper.get_cell_log_odds(hit_cx, hit_cy) - 2.0) < 1e-9

    # Now obstacle moves away; longer rays pass through (1.05, 0.55) and clear it back to free (0)
    for _ in range(6):
        mapper.update_ray(0.15, 0.55, 0.0, range_m=1.70, range_min=0.05, range_max=5.0)
    grid_data = mapper.to_occupancy_grid_data()
    assert grid_data[hit_cy * mapper.width + hit_cx] == 0
