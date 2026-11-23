"""Unit tests for BR footprint inflation, exponential cost decay, and L1 restricted zones."""

import math

from br_nav_sandbox.traversability_costmap import (
    COST_FREE,
    COST_INSCRIBED,
    COST_LETHAL,
    COST_UNKNOWN,
    TraversabilityCostmapGenerator,
    cost100_to_cost254,
    cost254_to_cost100,
    is_point_restricted_on_l1,
)


def test_cost_curve_zones_and_monotonic_decay():
    gen = TraversabilityCostmapGenerator(
        grid_resolution=0.05,
        inscribed_radius=0.35,
        circumscribed_radius=0.50,
        inflation_radius=0.80,
        cost_scaling_factor=5.0,
        enforce_l1_arena_bounds=False,
    )

    # 1. Lethal obstacle at d = 0
    assert gen.compute_cell_cost_254(0.0) == COST_LETHAL

    # 2. Inscribed zone: 0 < d <= 0.35 m -> COST_INSCRIBED (253)
    for d in (0.05, 0.20, 0.35):
        assert gen.compute_cell_cost_254(d) == COST_INSCRIBED

    # 3. Exponential decay zone: 0.35 < d <= 0.80 m
    c_040 = gen.compute_cell_cost_254(0.40)
    c_050 = gen.compute_cell_cost_254(0.50)  # circumscribed radius
    c_065 = gen.compute_cell_cost_254(0.65)
    c_080 = gen.compute_cell_cost_254(0.80)

    assert COST_INSCRIBED > c_040 > c_050 > c_065 > c_080 > COST_FREE
    expected_050 = int(math.floor(252.0 * math.exp(-5.0 * (0.50 - 0.35))))
    assert c_050 == expected_050

    # 4. Beyond inflation radius: d > 0.80 m -> COST_FREE (0)
    assert gen.compute_cell_cost_254(0.85) == COST_FREE
    assert gen.compute_cell_cost_254(1.50) == COST_FREE


def test_cost254_and_cost100_conversion():
    assert cost254_to_cost100(COST_LETHAL) == 100
    assert cost254_to_cost100(COST_INSCRIBED) == 99
    assert cost254_to_cost100(COST_FREE) == 0
    assert cost254_to_cost100(COST_UNKNOWN) == -1

    assert cost100_to_cost254(100) == COST_LETHAL
    assert cost100_to_cost254(99) == COST_INSCRIBED
    assert cost100_to_cost254(0) == COST_FREE
    assert cost100_to_cost254(-1) == COST_UNKNOWN


def test_abu_robocon_2027_l1_restricted_zones():
    # Valid locations for Red Team BR on Level 1:
    # - Red L1 Retry Zone (-1.85, 2.62)
    assert not is_point_restricted_on_l1(-1.85, 2.62, team_side='red')
    # - Red L1 West corridor (-2.20, 0.0)
    assert not is_point_restricted_on_l1(-2.20, 0.0, team_side='red')
    # - Red Transfer Area (-3.50, -1.30)
    assert not is_point_restricted_on_l1(-3.50, -1.30, team_side='red')
    # - North & South L1 Shared Areas (x in [-0.50, 0.50], |y| in [2.22, 2.97])
    assert not is_point_restricted_on_l1(0.25, 2.55, team_side='red')
    assert not is_point_restricted_on_l1(0.25, -2.55, team_side='red')

    # Restricted / non-traversable locations for Red Team BR on Level 1:
    # - Central Level 2 platform (0.0, 0.0) and (-1.20, 1.20)
    assert is_point_restricted_on_l1(0.0, 0.0, team_side='red')
    assert is_point_restricted_on_l1(-1.20, 1.20, team_side='red')
    # - Level 1 Center Divider Fences (x=0, y = +-1.86 m)
    assert is_point_restricted_on_l1(0.0, 1.86, team_side='red')
    assert is_point_restricted_on_l1(0.0, -1.86, team_side='red')
    # - Opponent (Blue) exclusive L1 floor (+2.20, 0.0) and (+1.85, 2.62)
    assert is_point_restricted_on_l1(2.20, 0.0, team_side='red')
    assert is_point_restricted_on_l1(1.85, 2.62, team_side='red')
    # - Blue Transfer Area (+3.50, -1.30) when playing Red
    assert is_point_restricted_on_l1(3.50, -1.30, team_side='red')
    # - Ground Area drop-off outside L1 and Transfer Area (-3.50, +1.00)
    assert is_point_restricted_on_l1(-3.50, 1.00, team_side='red')
    # - L1 30 mm Perimeter Barrier (-2.985, 0.50)
    assert is_point_restricted_on_l1(-2.985, 0.50, team_side='red')


def test_single_obstacle_inflation_symmetry():
    width, height = 41, 41
    res = 0.05
    occ = [0] * (width * height)
    center_cx, center_cy = 20, 20
    occ[center_cy * width + center_cx] = 100

    gen = TraversabilityCostmapGenerator(
        grid_resolution=res,
        inscribed_radius=0.35,
        circumscribed_radius=0.50,
        inflation_radius=0.75,
        cost_scaling_factor=5.0,
        enforce_l1_arena_bounds=False,
    )
    costmap = gen.generate_costmap_254(occ, width, height, origin_x=0.0, origin_y=0.0)

    # Obstacle center is lethal (254)
    assert costmap[center_cy * width + center_cx] == COST_LETHAL

    # 7 cells away (0.35 m = inscribed_radius) along +X, -X, +Y, -Y must be COST_INSCRIBED (253)
    for dx, dy in ((7, 0), (-7, 0), (0, 7), (0, -7)):
        assert costmap[(center_cy + dy) * width + (center_cx + dx)] == COST_INSCRIBED

    # 10 cells away (0.50 m = circumscribed_radius) must be in decay range (1..252)
    c_10 = costmap[center_cy * width + (center_cx + 10)]
    assert 1 <= c_10 < COST_INSCRIBED

    # 17 cells away (0.85 m > inflation_radius) must be COST_FREE (0)
    assert costmap[center_cy * width + (center_cx + 17)] == COST_FREE
