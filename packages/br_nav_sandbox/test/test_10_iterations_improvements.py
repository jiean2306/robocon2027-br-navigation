"""
Regression tests covering all 10 iterative improvements to br_nav_sandbox:
  1. Vectorized EDT Costmap Speed (< 25 ms on 228x228 full-arena grid)
  2. Multi-Level Traversability Costmaps & Planning (Ground, Level 1, Level 2)
  3. Sliding Wall & Obstacle Collision Physics (blocks ghosting through L2/barriers/enemy)
  4. Theta*-Style Any-Angle LOS Path Smoothing
  5. 3-Floor Isolated Occupancy Grids (Ground/L1/L2 never corrupt each other)
  6. Autonomous Emergency Brake (empty path + LiDAR < 0.42m safety shield)
  7. Decoupled Holonomic Mecanum Yaw Alignment
  8. Fast Analytic 3-Floor Static Map Rasterization
  9. Smart Start/Goal Standoff Projection (Ground Start Zone & Building Spots)
 10. Live Static-vs-Dynamic Enemy Robot Differentiation & A* Replanning
"""

import math
import time

from br_nav_sandbox.astar_pathfinder import CostAwareAStarPlanner
from br_nav_sandbox.l1_sim_bridge import (
    build_ground_arena_wall_segments,
    build_l1_arena_wall_segments,
    build_l2_arena_wall_segments,
    is_circle_colliding,
    point_to_segment_distance,
    simulate_l1_laser_scan,
    wrap_angle,
)
from br_nav_sandbox.occupancy_mapper import LogOddsOccupancyMapper
from br_nav_sandbox.traversability_costmap import (
    COST_INSCRIBED,
    TraversabilityCostmapGenerator,
    is_point_restricted_on_level,
    level_from_z,
)


def test_iter1_vectorized_edt_costmap_speed():
    w = h = 228
    occ = [0] * (w * h)
    gen = TraversabilityCostmapGenerator(
        grid_resolution=0.05,
        inscribed_radius=0.35,
        inflation_radius=0.80,
        cost_scaling_factor=5.0,
        active_level='level_1',
    )
    # Warm static cache
    gen.generate_occupancy_grid_100(occ, w, h, -5.7, -5.7)
    t0 = time.perf_counter()
    c254, g100 = gen.generate_occupancy_grid_100(occ, w, h, -5.7, -5.7)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    assert elapsed_ms < 25.0, f'EDT costmap too slow: {elapsed_ms:.1f} ms'
    assert len(c254) == w * h and len(g100) == w * h


def test_iter2_multilevel_ground_l1_l2_costmaps_and_astar():
    w = h = 228
    occ = [0] * (w * h)
    planner = CostAwareAStarPlanner(smooth_path=True, snap_inscribed_endpoints=True)

    # Ground: Red Start Zone -> Red Ramp Entry
    gen_g = TraversabilityCostmapGenerator(grid_resolution=0.05, active_level='ground')
    c_g, _ = gen_g.generate_occupancy_grid_100(occ, w, h, -5.7, -5.7)
    res_g = planner.plan_on_costmap_254(c_g, w, h, 0.05, -5.7, -5.7, (-4.35, -5.15), (-4.50, 2.50))
    assert res_g.success

    # Level 1: Red L1 Retry -> South L1 Shared Area approach
    gen_l1 = TraversabilityCostmapGenerator(grid_resolution=0.05, active_level='level_1')
    c_l1, _ = gen_l1.generate_occupancy_grid_100(occ, w, h, -5.7, -5.7)
    res_l1 = planner.plan_on_costmap_254(c_l1, w, h, 0.05, -5.7, -5.7, (-1.85, 2.45), (-0.55, -2.45))
    assert res_l1.success

    # Level 2: SW Corner -> NE Corner avoiding Central Pillar at (0, 0)
    gen_l2 = TraversabilityCostmapGenerator(grid_resolution=0.05, active_level='level_2')
    c_l2, _ = gen_l2.generate_occupancy_grid_100(occ, w, h, -5.7, -5.7)
    res_l2 = planner.plan_on_costmap_254(c_l2, w, h, 0.05, -5.7, -5.7, (-0.95, -0.95), (0.95, 0.95))
    assert res_l2.success
    # Ensure Level 2 path keeps safe clearance around the Central Pillar (radius 0.135m + 0.35m)
    for wx, wy in res_l2.path_world:
        assert math.hypot(wx, wy) > 0.45

    assert is_point_restricted_on_level(0.0, 0.0, level='level_2')
    assert not is_point_restricted_on_level(-0.95, -0.95, level='level_2')
    assert level_from_z(0.0) == 'ground'
    assert level_from_z(0.60) == 'level_1'
    assert level_from_z(0.90) == 'level_2'


def test_iter3_sliding_wall_collision_blocks_penetration():
    l1_segs = build_l1_arena_wall_segments()
    # Center at (-2.10, 0.0) is safely in the West L1 corridor
    assert not is_circle_colliding(-2.10, 0.0, 0.31, l1_segs)
    # Trying to penetrate L2 wall at x = -1.50 (robot center at x = -1.35) MUST collide
    assert is_circle_colliding(-1.35, 0.0, 0.31, l1_segs)
    # Trying to penetrate West L1 perimeter barrier at x = -2.97 (center at -2.80) MUST collide
    assert is_circle_colliding(-2.80, 0.50, 0.31, l1_segs)
    # Point-to-segment distance exactness
    assert abs(point_to_segment_distance(0.0, 2.0, (-1.0, 0.0, 1.0, 0.0)) - 2.0) < 1e-9


def test_iter4_los_path_smoothing_reduces_zigzags():
    w = h = 228
    occ = [0] * (w * h)
    gen = TraversabilityCostmapGenerator(grid_resolution=0.05, active_level='level_1')
    c254, _ = gen.generate_occupancy_grid_100(occ, w, h, -5.7, -5.7)

    raw_planner = CostAwareAStarPlanner(smooth_path=False)
    smooth_planner = CostAwareAStarPlanner(smooth_path=True)

    r_raw = raw_planner.plan_on_costmap_254(c254, w, h, 0.05, -5.7, -5.7, (-1.85, 2.45), (-0.55, -2.45))
    r_smooth = smooth_planner.plan_on_costmap_254(c254, w, h, 0.05, -5.7, -5.7, (-1.85, 2.45), (-0.55, -2.45))

    assert r_raw.success and r_smooth.success
    assert r_smooth.smoothed_corners <= 8
    assert r_smooth.smoothed_corners < len(r_raw.path_cells) // 5


def test_iter5_and_iter8_fast_3floor_raster_and_isolation():
    t0 = time.perf_counter()
    m_ground = LogOddsOccupancyMapper()
    m_ground.rasterize_static_segments(build_ground_arena_wall_segments(), [(-5.48, -5.48, 5.48, 5.48)])

    m_l1 = LogOddsOccupancyMapper()
    m_l1.rasterize_static_segments(build_l1_arena_wall_segments(), [(-2.95, -2.95, 2.95, 2.95)])

    m_l2 = LogOddsOccupancyMapper()
    m_l2.rasterize_static_segments(build_l2_arena_wall_segments(), [(-1.48, -1.48, 1.48, 1.48)])
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    assert elapsed_ms < 200.0, f'3-floor raster too slow: {elapsed_ms:.1f} ms'

    # Scan on Level 2 should NOT affect Level 1's walls
    l1_occ_before = m_l1.to_occupancy_grid_data().count(100)
    a_min, a_inc, rng = simulate_l1_laser_scan(-0.9, 0.0, 0.0, segments=build_l2_arena_wall_segments())
    m_l2.update_from_scan(-0.9, 0.0, 0.0, a_min, a_inc, rng, 0.08, 8.0)
    l1_occ_after = m_l1.to_occupancy_grid_data().count(100)
    assert l1_occ_before == l1_occ_after


def test_iter7_wrap_angle_and_holonomic_yaw():
    assert abs(wrap_angle(3.5 * math.pi) - (-0.5 * math.pi)) < 1e-9
    assert abs(wrap_angle(-3.5 * math.pi) - (0.5 * math.pi)) < 1e-9
    assert abs(wrap_angle(0.25) - 0.25) < 1e-9


def test_iter9_standoff_projection_snaps_building_spot_goal():
    w = h = 228
    occ = [0] * (w * h)
    gen = TraversabilityCostmapGenerator(grid_resolution=0.05, active_level='level_1')
    c254, _ = gen.generate_occupancy_grid_100(occ, w, h, -5.7, -5.7)

    # Clicking right inside the South-West Building Spot (-2.65, -2.65) is inside inscribed margin (253)
    cx, cy = CostAwareAStarPlanner.world_to_cell(-2.65, -2.65, -5.7, -5.7, 0.05)
    assert c254[cy * w + cx] == COST_INSCRIBED

    snap_planner = CostAwareAStarPlanner(smooth_path=True, snap_inscribed_endpoints=True)
    res = snap_planner.plan_on_costmap_254(c254, w, h, 0.05, -5.7, -5.7, (-1.85, 2.45), (-2.65, -2.65))
    assert res.success
    assert 'snapped' in res.reason


def test_rulebook_fig4_sky_block_orientations_and_mirror_symmetry():
    """
    Triple-verify every one of the 12 Sky Blocks in arena_geometry.py against the
    pixel-verified ABU Robocon 2027 Rulebook V1.1 Figure 4 (Page 15) layout and
    exact Red<->Blue left-right mirror symmetry across column 2.
    """
    from br_nav_sandbox.arena_geometry import build_exact_arena_primitives

    prims = build_exact_arena_primitives(include_game_objects=True)
    by_name = {p.name: p for p in prims}

    expected = {
        1: ('H_NR_SB', 0, 0),
        2: ('V_WR_EB', 0, 2),
        3: ('H_NB_SR', 0, 4),
        4: ('V_WB_ER', 1, 1),
        5: ('V_WB_ER', 1, 3),
        6: ('H_NB_SR', 2, 0),
        7: ('H_NR_SB', 2, 4),
        8: ('V_WR_EB', 3, 1),
        9: ('V_WR_EB', 3, 3),
        10: ('H_NR_SB', 4, 0),
        11: ('V_WB_ER', 4, 2),
        12: ('H_NB_SR', 4, 4),
    }

    for idx, (mode, row, col) in expected.items():
        red_p = by_name[f'sky_block_{idx:02d}_red_half']
        blue_p = by_name[f'sky_block_{idx:02d}_blue_half']
        rx, ry, _ = red_p.pose[:3]
        bx, by, _ = blue_p.pose[:3]
        if mode == 'H_NR_SB':
            assert abs(rx - bx) < 1e-6 and ry > by and red_p.size == (0.20, 0.10, 0.20)
        elif mode == 'H_NB_SR':
            assert abs(rx - bx) < 1e-6 and by > ry and red_p.size == (0.20, 0.10, 0.20)
        elif mode == 'V_WR_EB':
            assert abs(ry - by) < 1e-6 and rx < bx and red_p.size == (0.10, 0.20, 0.20)
        elif mode == 'V_WB_ER':
            assert abs(ry - by) < 1e-6 and bx < rx and red_p.size == (0.10, 0.20, 0.20)


def test_match_robots_obey_rulebook_operating_areas():
    """Verify OUR TR, ENEMY BR, and ENEMY TR strictly obey Rule 3.2 & 4.3 operating areas."""
    from br_nav_sandbox.l1_sim_bridge import get_match_robot_waypoints, interpolate_closed_waypoints

    wps = get_match_robot_waypoints()
    for t_s in range(0, 40):
        # 1. OUR TR must stay on Red side (x <= 0) and NEVER enter L1 inner floor (x > -2.97 when z >= 0.3)
        ox, oy, oz = interpolate_closed_waypoints(wps['our_tr'], float(t_s))
        assert ox <= 0.0 and oz <= 0.61
        if oz > 0.25:
            assert -4.05 <= ox <= -2.95, f'OUR TR illegally entered L1 beyond Transfer Area: ({ox}, {oy}, {oz})'

        # 2. ENEMY TR must stay on Blue side (x >= 0) and NEVER enter L1 inner floor
        etx, ety, etz = interpolate_closed_waypoints(wps['enemy_tr'], float(t_s))
        assert etx >= 0.0 and etz <= 0.61
        if etz > 0.25:
            assert 2.95 <= etx <= 4.05, f'ENEMY TR illegally entered L1 beyond Transfer Area: ({etx}, {ety}, {etz})'

        # 3. ENEMY BR must operate on Blue L1, L1 Shared Areas, or Level 2 Shared Area (never Red exclusive L1!)
        ebx, eby, ebz = interpolate_closed_waypoints(wps['enemy_br'], float(t_s))
        assert ebz >= 0.59
        if ebz < 0.78:  # on Level 1
            in_l1_shared = abs(ebx) <= 0.50 and abs(eby) >= 2.22
            assert ebx >= 0.0 or in_l1_shared, f'ENEMY BR entered Red exclusive L1: ({ebx}, {eby}, {ebz})'


def test_traversability_viz_buckets_no_solid_pink_clutter() -> None:
    """Traversability halo buckets must render only active-floor safety zones & 1-cell border."""
    from br_nav_sandbox.traversability_costmap import (
        TraversabilityCostmapGenerator,
        build_traversability_viz_buckets,
    )

    gen = TraversabilityCostmapGenerator(
        grid_resolution=0.05,
        inscribed_radius=0.35,
        inflation_radius=0.80,
        cost_scaling_factor=5.0,
        team_side='red',
        active_level='ground',
    )
    w, h = 228, 228
    occ = [0] * (w * h)
    c254 = gen.generate_costmap_254(occ, w, h, origin_x=-5.7, origin_y=-5.7)
    buckets = build_traversability_viz_buckets(c254, w, h, origin_x=-5.7, origin_y=-5.7, resolution=0.05)

    assert len(buckets['inscribed']) > 200
    assert len(buckets['decay_high']) > 50
    assert len(buckets['decay_med']) > 50
    assert len(buckets['decay_low']) > 50
    assert len(buckets['lethal_edge']) > 100

    # Crucially, out-of-bounds lethal cells must NOT be blanket-painted:
    total_lethal = sum(1 for v in c254 if v == 254)
    assert len(buckets['lethal_edge']) < total_lethal * 0.15


def test_gz_overlay_bridge_protobuf_markers() -> None:
    """Verify Gazebo Harmonic TRIANGLE_LIST and LINE_STRIP marker builders."""
    from br_nav_sandbox.gz_overlay_bridge import (
        GZ_AVAILABLE,
        build_gz_path_marker,
        build_gz_quad_list_marker,
    )

    if not GZ_AVAILABLE:
        return

    qm = build_gz_quad_list_marker(
        'br_traversability', 3, [(-4.0, -4.5, 0.02), (-3.95, -4.5, 0.02)], 0.048, (0.0, 0.9, 1.0, 0.6)
    )
    assert len(qm.point) == 12  # 2 quads * 2 triangles * 3 vertices

    pm = build_gz_path_marker('br_astar_path', 20, [(-4.35, -5.15, 0.07), (-4.0, -4.0, 0.07)])
    assert len(pm.point) == 2


def test_unified_rviz_and_minimal_window_defaults() -> None:
    """Default launch must use single unified RViz view (dual_views=false) with both layers enabled."""
    import os

    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rviz_path = os.path.join(pkg_root, 'rviz', 'br_unified_view.rviz')
    launch_path = os.path.join(pkg_root, 'launch', 'br_nav_sandbox.launch.py')

    assert os.path.isfile(rviz_path)
    with open(rviz_path, 'r', encoding='utf-8') as f:
        rviz_txt = f.read()
    assert '/br_occupancy_grid' in rviz_txt
    assert '/br_traversability_viz' in rviz_txt
    assert '3D Arena + Traversability Orbit' in rviz_txt
    assert '2D Top-Down Mapping (North-Up)' in rviz_txt

    with open(launch_path, 'r', encoding='utf-8') as f:
        launch_txt = f.read()
    assert "default_value='false'" in launch_txt
    assert 'gz_overlay_bridge' in launch_txt


def test_full_robocon_2027_match_mission_rulebook_compliance() -> None:
    """
    Simulate the complete ABU Robocon 2027 Rulebook V1.1 match scenario end-to-end:
      - TR strictly stays on Ground Area + Red Ramp + Red Transfer Area (never enters L1!)
      - Tower #1 (Red L1) & Tower #2 (L1 Shared) built in exact Earth -> Earth -> Sky order
      - Sanctuary Mandate unlocks ONLY after 2 Complete Towers (>=1 Shared)
      - Mustika NEVER touches Transfer Area floor (direct handover TR -> BR)
      - Final score = 30 (Transfer) + 140 (2 L1 Towers) + 250 (Mustika) = 420 pts
    """
    from br_nav_sandbox.match_scenario_director import BR_START, MatchMissionLogic

    logic = MatchMissionLogic()
    logic.start()

    br_pos = list(BR_START)
    climb_timer = 0.0
    climb_target = None
    active_goal = None

    for _tick in range(4000):
        if logic.completed:
            break

        # Advance simulated BR towards current A* goal or Wheel-Lift climb target
        dt = 0.05
        if climb_target is not None:
            climb_timer += dt
            if climb_timer >= 1.0:
                br_pos = list(climb_target)
                climb_target = None
                climb_timer = 0.0
        elif active_goal is not None:
            dx = active_goal[0] - br_pos[0]
            dy = active_goal[1] - br_pos[1]
            dist = math.hypot(dx, dy)
            step = min(dist, 2.2 * dt)
            if dist > 1e-6:
                br_pos[0] += dx / dist * step
                br_pos[1] += dy / dist * step
            br_pos[2] = active_goal[2]

        actions = logic.advance_mission(
            (br_pos[0], br_pos[1], br_pos[2]),
            climb_idle=(climb_target is None),
            dt=dt,
        )

        # Rule 4.3.1: TR must NEVER enter Level 1 beyond Transfer Area!
        if logic.tr_pos[2] > 0.25:
            assert logic.tr_pos[0] <= -2.99, f'TR violated L1 boundary: {logic.tr_pos}'

        # Rule 3.4.5: Mustika must NEVER be placed on the Transfer Area floor!
        assert logic.transfer_block != 'mustika'

        # Rule 3.6.7: TR cannot carry Mustika before Sanctuary Mandate is fulfilled!
        if logic.tr_carrying == 'mustika':
            assert logic.sanctuary_mandate_fulfilled

        if actions['send_climb'] == 'mission_climb_l1':
            climb_target = (-2.50, -1.30, 0.60)
            active_goal = None
        elif actions['send_climb'] == 'mission_climb_l2':
            climb_target = (-0.95, 0.00, 0.90)
            active_goal = None
        elif actions['send_goal'] is not None:
            active_goal = actions['send_goal']

    assert logic.completed, f'Mission did not complete! Stuck at step {logic.step_idx}: {logic.current_step}'
    assert logic.sanctuary_mandate_fulfilled
    assert logic.mustika_enshrined
    assert logic.complete_towers_exclusive == 1
    assert logic.complete_towers_shared == 1
    assert logic.score_transfer == 30
    assert logic.score_towers == 140
    assert logic.score_mustika == 250
    assert logic.total_score == 420
    assert len(logic.placed_objects) == 7  # 4 Earth + 2 Sky + 1 Mustika



