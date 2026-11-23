"""Unit tests for cost-aware A* pathfinding, obstacle avoidance, and full L1 navigation."""

import math

from br_nav_sandbox.astar_pathfinder import CostAwareAStarPlanner
from br_nav_sandbox.l1_sim_bridge import simulate_l1_laser_scan
from br_nav_sandbox.occupancy_mapper import LogOddsOccupancyMapper
from br_nav_sandbox.traversability_costmap import (
    COST_FREE,
    COST_INSCRIBED,
    COST_LETHAL,
    TraversabilityCostmapGenerator,
)


def test_astar_blocks_lethal_and_inscribed_cells():
    width, height = 30, 30
    res = 0.10
    occ = [0] * (width * height)
    # Vertical obstacle wall at cx = 15, cy in [5..24]
    for cy in range(5, 25):
        occ[cy * width + 15] = 100

    gen = TraversabilityCostmapGenerator(
        grid_resolution=res,
        inscribed_radius=0.20,
        circumscribed_radius=0.30,
        inflation_radius=0.50,
        cost_scaling_factor=5.0,
        enforce_l1_arena_bounds=False,
    )
    costmap_254 = gen.generate_costmap_254(occ, width, height, origin_x=0.0, origin_y=0.0)

    planner = CostAwareAStarPlanner(allow_diagonal=True, cost_penalty_weight=3.0)
    res_plan = planner.plan_on_costmap_254(
        costmap_254=costmap_254,
        width=width,
        height=height,
        resolution=res,
        origin_x=0.0,
        origin_y=0.0,
        start_world=(0.55, 1.55),
        goal_world=(2.45, 1.55),
    )

    assert res_plan.success
    assert len(res_plan.path_cells) > 10
    # Verify no cell on the path enters a lethal (254) or inscribed (253) cell
    for cx, cy in res_plan.path_cells:
        assert costmap_254[cy * width + cx] < COST_INSCRIBED


def test_astar_cost_penalty_chooses_safer_clearance_path():
    """
    Verify that increasing `cost_penalty_weight` shifts the A* trajectory farther
    away from an obstacle's exponential inflation halo (safest vs shortest path).
    """
    width, height = 40, 40
    res = 0.05
    occ = [0] * (width * height)
    # Place an obstacle cluster at (cx=20, cy=18..20)
    for cy in range(18, 21):
        occ[cy * width + 20] = 100

    gen = TraversabilityCostmapGenerator(
        grid_resolution=res,
        inscribed_radius=0.20,
        circumscribed_radius=0.30,
        inflation_radius=0.70,
        cost_scaling_factor=3.5,
        enforce_l1_arena_bounds=False,
    )
    costmap_254 = gen.generate_costmap_254(occ, width, height, origin_x=0.0, origin_y=0.0)

    start_xy = (0.30, 1.00)
    goal_xy = (1.70, 1.00)

    # 1. Pure shortest geometric path (cost_penalty_weight = 0.0)
    shortest_planner = CostAwareAStarPlanner(allow_diagonal=True, cost_penalty_weight=0.0)
    shortest_res = shortest_planner.plan_on_costmap_254(
        costmap_254, width, height, res, 0.0, 0.0, start_xy, goal_xy
    )

    # 2. Cost-penalized safest path (cost_penalty_weight = 8.0)
    safest_planner = CostAwareAStarPlanner(allow_diagonal=True, cost_penalty_weight=8.0)
    safest_res = safest_planner.plan_on_costmap_254(
        costmap_254, width, height, res, 0.0, 0.0, start_xy, goal_xy
    )

    assert shortest_res.success and safest_res.success

    max_cost_shortest = max(costmap_254[cy * width + cx] for cx, cy in shortest_res.path_cells)
    max_cost_safest = max(costmap_254[cy * width + cx] for cx, cy in safest_res.path_cells)

    # The cost-aware safest planner must keep a strictly lower peak inflation cost
    # (greater physical clearance from the obstacle) than the zero-penalty planner.
    assert max_cost_safest < max_cost_shortest


def test_full_l1_pipeline_lidar_to_costmap_to_astar():
    """
    End-to-end integration test of the ABU Robocon 2027 Level 1 pipeline:
      1. Simulate 2D LiDAR scans from multiple L1 poses
      2. Update Bayesian LogOddsOccupancyMapper (/br_occupancy_grid)
      3. Generate footprint-inflated L1 TraversabilityCostmap (/br_traversability_costmap)
      4. Plan collision-free A* paths from Red L1 Retry Zone (-1.85, 2.45) to:
           (a) Red Transfer Area (-3.45, -1.30)
           (b) South L1 Shared Building Spot approach (-0.25, -2.40)
    """
    mapper = LogOddsOccupancyMapper(
        grid_resolution=0.05,
        width_m=9.0,
        height_m=7.0,
        origin_x=-4.5,
        origin_y=-3.5,
    )

    for pose_x, pose_y in ((-1.85, 2.45), (-2.20, 0.0), (-2.20, -2.20)):
        angle_min, angle_inc, ranges = simulate_l1_laser_scan(
            sensor_x=pose_x, sensor_y=pose_y, sensor_yaw=0.0, num_beams=360
        )
        for _ in range(3):
            mapper.update_from_scan(
                sensor_x=pose_x,
                sensor_y=pose_y,
                sensor_yaw=0.0,
                angle_min=angle_min,
                angle_increment=angle_inc,
                ranges=ranges,
                range_min=0.08,
                range_max=8.0,
            )

    occ_data = mapper.to_occupancy_grid_data()
    assert 100 in occ_data and 0 in occ_data

    costmap_gen = TraversabilityCostmapGenerator(
        grid_resolution=0.05,
        inscribed_radius=0.35,
        circumscribed_radius=0.50,
        inflation_radius=0.80,
        cost_scaling_factor=5.0,
        team_side='red',
        enforce_l1_arena_bounds=True,
    )
    costmap_254, grid_100 = costmap_gen.generate_occupancy_grid_100(
        occ_data,
        width=mapper.width,
        height=mapper.height,
        origin_x=mapper.origin_x,
        origin_y=mapper.origin_y,
    )

    planner = CostAwareAStarPlanner(allow_diagonal=True, cost_penalty_weight=4.0)

    # Path (a): Red L1 Retry Zone (-1.85, 2.45) -> Red Transfer Area (-3.45, -1.30)
    res_transfer = planner.plan_on_occupancy_grid_100(
        grid_100=grid_100,
        width=mapper.width,
        height=mapper.height,
        resolution=mapper.resolution,
        origin_x=mapper.origin_x,
        origin_y=mapper.origin_y,
        start_world=(-1.85, 2.45),
        goal_world=(-3.45, -1.30),
    )
    assert res_transfer.success, f'Failed to plan to Red Transfer Area: {res_transfer.reason}'

    # Path (b): Red L1 Retry Zone (-1.85, 2.45) -> South L1 Shared Area approach (-0.55, -2.45)
    res_shared = planner.plan_on_occupancy_grid_100(
        grid_100=grid_100,
        width=mapper.width,
        height=mapper.height,
        resolution=mapper.resolution,
        origin_x=mapper.origin_x,
        origin_y=mapper.origin_y,
        start_world=(-1.85, 2.45),
        goal_world=(-0.55, -2.45),
    )
    assert res_shared.success, f'Failed to plan to South L1 Shared Area: {res_shared.reason}'

    # Verify every waypoint maintains > 0.35 m inscribed clearance from Level 2 ([-1.5, 1.5]^2)
    for wx, wy in res_shared.path_world:
        assert not ((-1.80 < wx < 1.80) and (-1.80 < wy < 1.80)), (
            f'Waypoint ({wx:.2f}, {wy:.2f}) violated L2 inscribed clearance!'
        )
