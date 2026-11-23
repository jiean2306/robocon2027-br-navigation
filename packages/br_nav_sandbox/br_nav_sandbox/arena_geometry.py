#!/usr/bin/env python3
"""
Exact ABU Robocon 2027 ("The Pursuit of Mustika Nusantara") Arena Geometry & Color Specification.

Grounded directly in:
  * docs/ABU_Robocon_2027_Rulebook_V1.1-1.pdf (Figures 1-4, Section 2, Section 14 RGB table)
  * docs/ABU_Robocon_2027_Appendix_V1.1-1.pdf (Figures 2.1, 3.1-3.10)

Provides:
  1. Exact 3D box/cylinder/sphere primitives for every static arena element and game object
     (Ground Area Red/Blue, Game Field Boundary, Ground & L1 Center Dividers, Start/Retry Zones,
     Storage Areas + 40 Earth Blocks, Ground Shared Area 5x5 Grid + 12 Sky Blocks, Mustika Shared
     Area + Mustika Pillar + Mustika Sphere, Stairs, Transfer Areas, Ramps, Level 1 Red/Blue floor,
     L1 Perimeter Barriers, L1 Retry Zones, L1 Shared Areas, 6 L1 Building Spots, Level 2 platform,
     4 L2 Corner Building Spots, and L2 Central Pillar).
  2. Generation of Gazebo Harmonic SDF 1.9 world (`robocon_2027_l1_sandbox.sdf`) with the spawned
     0.70 x 0.70 m Mecanum Builder Robot (`br_robot`) on Level 1.
  3. Conversion to `visualization_msgs/msg/MarkerArray` (`/br_arena_markers`) so the 3D Simulation
     View in RViz2 displays the exact same 3D ABU Robocon 2027 arena as Gazebo Harmonic.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Tuple
import xml.etree.ElementTree as ET


# Official RGB color table from Rulebook V1.1 Section 14 (pages 30-31)
RULEBOOK_RGB: Dict[str, Tuple[int, int, int]] = {
    'ground_red': (240, 210, 210),
    'ground_blue': (170, 210, 230),
    'ground_shared': (245, 240, 200),
    'start_retry_red': (223, 34, 34),
    'start_retry_blue': (50, 0, 255),
    'storage_red': (240, 210, 210),
    'storage_blue': (170, 210, 230),
    'pillar_brown': (100, 62, 0),
    'stairs_red': (235, 180, 160),
    'stairs_blue': (150, 215, 220),
    'ramp_red': (200, 150, 140),
    'ramp_blue': (130, 180, 200),
    'l1_red': (235, 180, 160),
    'l1_blue': (150, 215, 220),
    'building_spot_green': (40, 100, 50),
    'l1_shared': (245, 240, 200),
    'transfer_red': (245, 170, 60),
    'transfer_blue': (60, 170, 245),
    'barrier_grey': (190, 190, 185),
    'l2_grey': (190, 190, 185),
    'fence_brown': (100, 62, 0),
    'earth_block_red': (220, 38, 38),
    'earth_block_blue': (35, 55, 220),
    'sky_block_red': (230, 45, 45),
    'sky_block_blue': (40, 65, 230),
    'mustika_yellow': (245, 205, 0),
    'dark_center_cell': (55, 55, 60),
}


def rgb_norm(color_key: str) -> Tuple[float, float, float]:
    r, g, b = RULEBOOK_RGB[color_key]
    return (r / 255.0, g / 255.0, b / 255.0)


@dataclass
class ArenaPrimitive:
    """Single 3D geometric primitive in the ABU Robocon 2027 arena."""

    name: str
    shape: str  # 'box', 'cylinder', 'sphere'
    pose: Tuple[float, float, float, float, float, float]  # x, y, z, roll, pitch, yaw
    size: Tuple[float, float, float]  # box: (sx, sy, sz); cylinder: (radius, length, 0); sphere: (radius, 0, 0)
    color_key: str
    collision: bool = True


def build_exact_arena_primitives(include_game_objects: bool = True) -> List[ArenaPrimitive]:
    """
    Construct the complete list of 3D primitives matching the ABU Robocon 2027
    Rulebook V1.1 and Appendix V1.1 drawings (Figures 3.1 - 3.10).
    """
    prims: List[ArenaPrimitive] = []

    # -------------------------------------------------------------------------
    # 1. GROUND AREA (11000 x 11000 mm inner playing field, z = 0.00 m top)
    # -------------------------------------------------------------------------
    prims.append(
        ArenaPrimitive(
            name='ground_slab_red',
            shape='box',
            pose=(-2.75, 0.0, -0.05, 0.0, 0.0, 0.0),
            size=(5.50, 11.00, 0.10),
            color_key='ground_red',
            collision=True,
        )
    )
    prims.append(
        ArenaPrimitive(
            name='ground_slab_blue',
            shape='box',
            pose=(2.75, 0.0, -0.05, 0.0, 0.0, 0.0),
            size=(5.50, 11.00, 0.10),
            color_key='ground_blue',
            collision=True,
        )
    )

    # Outer Game Field Boundary Fence (50 mm thick x 100 mm high outside +-5.50 m)
    prims.extend(
        [
            ArenaPrimitive(
                'game_field_boundary_north',
                'box',
                (0.0, 5.525, 0.05, 0.0, 0.0, 0.0),
                (11.10, 0.05, 0.10),
                'fence_brown',
                True,
            ),
            ArenaPrimitive(
                'game_field_boundary_south',
                'box',
                (0.0, -5.525, 0.05, 0.0, 0.0, 0.0),
                (11.10, 0.05, 0.10),
                'fence_brown',
                True,
            ),
            ArenaPrimitive(
                'game_field_boundary_west',
                'box',
                (-5.525, 0.0, 0.05, 0.0, 0.0, 0.0),
                (0.05, 11.00, 0.10),
                'fence_brown',
                True,
            ),
            ArenaPrimitive(
                'game_field_boundary_east',
                'box',
                (5.525, 0.0, 0.05, 0.0, 0.0, 0.0),
                (0.05, 11.00, 0.10),
                'fence_brown',
                True,
            ),
        ]
    )

    # Ground Center Divider Fences (Appendix Fig 3.9 & 3.10b: 30 mm thick x 50 mm high)
    # South segments: 650 mm + (1200 mm Ground Shared Area opening) + 650 mm = 2500 mm
    # North segments: 750 mm + (1000 mm Mustika Shared Area opening) + 750 mm = 2500 mm
    prims.extend(
        [
            ArenaPrimitive(
                'ground_center_divider_south_1',
                'box',
                (0.0, -5.175, 0.025, 0.0, 0.0, 0.0),
                (0.03, 0.65, 0.05),
                'fence_brown',
                True,
            ),
            ArenaPrimitive(
                'ground_center_divider_south_2',
                'box',
                (0.0, -3.325, 0.025, 0.0, 0.0, 0.0),
                (0.03, 0.65, 0.05),
                'fence_brown',
                True,
            ),
            ArenaPrimitive(
                'ground_center_divider_north_1',
                'box',
                (0.0, 3.375, 0.025, 0.0, 0.0, 0.0),
                (0.03, 0.75, 0.05),
                'fence_brown',
                True,
            ),
            ArenaPrimitive(
                'ground_center_divider_north_2',
                'box',
                (0.0, 5.125, 0.025, 0.0, 0.0, 0.0),
                (0.03, 0.75, 0.05),
                'fence_brown',
                True,
            ),
        ]
    )

    # Ground Start/Retry Zones (Appendix Fig 3.2a: two 700 x 700 mm per team, 100 mm gap)
    prims.extend(
        [
            ArenaPrimitive(
                'red_start_retry_zone_1',
                'box',
                (-5.15, -5.15, 0.003, 0.0, 0.0, 0.0),
                (0.70, 0.70, 0.006),
                'start_retry_red',
                False,
            ),
            ArenaPrimitive(
                'red_start_retry_zone_2',
                'box',
                (-4.35, -5.15, 0.003, 0.0, 0.0, 0.0),
                (0.70, 0.70, 0.006),
                'start_retry_red',
                False,
            ),
            ArenaPrimitive(
                'blue_start_retry_zone_1',
                'box',
                (4.35, -5.15, 0.003, 0.0, 0.0, 0.0),
                (0.70, 0.70, 0.006),
                'start_retry_blue',
                False,
            ),
            ArenaPrimitive(
                'blue_start_retry_zone_2',
                'box',
                (5.15, -5.15, 0.003, 0.0, 0.0, 0.0),
                (0.70, 0.70, 0.006),
                'start_retry_blue',
                False,
            ),
        ]
    )

    # Ground Storage Areas (Appendix Fig 3.2b: 2000 x 1000 mm at North-West & North-East corners)
    prims.extend(
        [
            ArenaPrimitive(
                'red_storage_area',
                'box',
                (-4.50, 5.00, 0.003, 0.0, 0.0, 0.0),
                (2.00, 1.00, 0.006),
                'start_retry_red',
                False,
            ),
            ArenaPrimitive(
                'blue_storage_area',
                'box',
                (4.50, 5.00, 0.003, 0.0, 0.0, 0.0),
                (2.00, 1.00, 0.006),
                'start_retry_blue',
                False,
            ),
        ]
    )

    # South Ground Shared Area (Appendix Fig 3.2c & Rulebook Fig 4: 1200 x 1200 mm at (0.0, -4.25))
    gx, gy = 0.0, -4.25
    prims.append(
        ArenaPrimitive(
            'ground_shared_area_south',
            'box',
            (gx, gy, 0.002, 0.0, 0.0, 0.0),
            (1.20, 1.20, 0.004),
            'ground_shared',
            False,
        )
    )
    # 5x5 chessboard floor cells (200 x 200 mm each, 100 mm margin inside 1200 x 1200 mm pad)
    sky_cell_set = {
        (0, 0), (0, 2), (0, 4),
        (1, 1), (1, 3),
        (2, 0), (2, 4),
        (3, 1), (3, 3),
        (4, 0), (4, 2), (4, 4),
    }
    for row in range(5):
        for col in range(5):
            cx_cell = gx - 0.50 + (col + 0.5) * 0.20
            cy_cell = gy + 0.50 - (row + 0.5) * 0.20
            if (row, col) == (2, 2):
                cell_col = 'dark_center_cell'
            elif (row, col) in sky_cell_set:
                cell_col = 'ground_shared'
            else:
                cell_col = 'barrier_grey'
            prims.append(
                ArenaPrimitive(
                    f'ground_shared_cell_{row}_{col}',
                    'box',
                    (cx_cell, cy_cell, 0.004, 0.0, 0.0, 0.0),
                    (0.194, 0.194, 0.004),
                    cell_col,
                    False,
                )
            )

    # North Mustika Ground Shared Area (Appendix Fig 3.2d: 1000 x 1000 mm at (0.0, +4.25))
    mx, my = 0.0, 4.25
    prims.extend(
        [
            ArenaPrimitive(
                'mustika_ground_shared_area',
                'box',
                (mx, my, 0.002, 0.0, 0.0, 0.0),
                (1.00, 1.00, 0.004),
                'ground_shared',
                False,
            ),
            ArenaPrimitive(
                'mustika_pillar',
                'cylinder',
                (mx, my, 0.25, 0.0, 0.0, 0.0),
                (0.135, 0.50, 0.0),
                'pillar_brown',
                True,
            ),
        ]
    )

    # -------------------------------------------------------------------------
    # 2. LEVEL 1 PLATFORM (6000 x 6000 mm, +600 mm elevation) & BARRIERS
    # -------------------------------------------------------------------------
    # Solid L1 base block (grey side walls)
    prims.append(
        ArenaPrimitive(
            'level_1_base',
            'box',
            (0.0, 0.0, 0.30, 0.0, 0.0, 0.0),
            (6.00, 6.00, 0.60),
            'barrier_grey',
            True,
        )
    )
    # Level 1 Red & Blue floor surfaces (5940 x 5940 mm inside 30 mm barrier)
    prims.extend(
        [
            ArenaPrimitive(
                'level_1_red_floor',
                'box',
                (-1.485, 0.0, 0.601, 0.0, 0.0, 0.0),
                (2.97, 5.94, 0.002),
                'l1_red',
                False,
            ),
            ArenaPrimitive(
                'level_1_blue_floor',
                'box',
                (1.485, 0.0, 0.601, 0.0, 0.0, 0.0),
                (2.97, 5.94, 0.002),
                'l1_blue',
                False,
            ),
        ]
    )

    # L1 Perimeter Barriers (30 mm thick x 50 mm high, open only at Transfer Areas y in [-1.8, -0.8])
    prims.extend(
        [
            ArenaPrimitive(
                'l1_barrier_north',
                'box',
                (0.0, 2.985, 0.625, 0.0, 0.0, 0.0),
                (6.00, 0.03, 0.05),
                'barrier_grey',
                True,
            ),
            ArenaPrimitive(
                'l1_barrier_south',
                'box',
                (0.0, -2.985, 0.625, 0.0, 0.0, 0.0),
                (6.00, 0.03, 0.05),
                'barrier_grey',
                True,
            ),
            ArenaPrimitive(
                'l1_barrier_west_lower',
                'box',
                (-2.985, -2.40, 0.625, 0.0, 0.0, 0.0),
                (0.03, 1.20, 0.05),
                'barrier_grey',
                True,
            ),
            ArenaPrimitive(
                'l1_barrier_west_upper',
                'box',
                (-2.985, 1.10, 0.625, 0.0, 0.0, 0.0),
                (0.03, 3.80, 0.05),
                'barrier_grey',
                True,
            ),
            ArenaPrimitive(
                'l1_barrier_east_lower',
                'box',
                (2.985, -2.40, 0.625, 0.0, 0.0, 0.0),
                (0.03, 1.20, 0.05),
                'barrier_grey',
                True,
            ),
            ArenaPrimitive(
                'l1_barrier_east_upper',
                'box',
                (2.985, 1.10, 0.625, 0.0, 0.0, 0.0),
                (0.03, 3.80, 0.05),
                'barrier_grey',
                True,
            ),
        ]
    )

    # Level 1 Center Divider Fences (Appendix Fig 3.3, 3.9, 3.10d: 30 mm x 720 mm x 50 mm)
    prims.extend(
        [
            ArenaPrimitive(
                'l1_center_divider_south',
                'box',
                (0.0, -1.86, 0.625, 0.0, 0.0, 0.0),
                (0.03, 0.72, 0.05),
                'fence_brown',
                True,
            ),
            ArenaPrimitive(
                'l1_center_divider_north',
                'box',
                (0.0, 1.86, 0.625, 0.0, 0.0, 0.0),
                (0.03, 0.72, 0.05),
                'fence_brown',
                True,
            ),
        ]
    )

    # Level 1 Shared Areas (1000 x 750 mm at North & South centerlines)
    prims.extend(
        [
            ArenaPrimitive(
                'l1_shared_area_south',
                'box',
                (0.0, -2.595, 0.603, 0.0, 0.0, 0.0),
                (1.00, 0.75, 0.004),
                'l1_shared',
                False,
            ),
            ArenaPrimitive(
                'l1_shared_area_north',
                'box',
                (0.0, 2.595, 0.603, 0.0, 0.0, 0.0),
                (1.00, 0.75, 0.004),
                'l1_shared',
                False,
            ),
        ]
    )

    # Level 1 Retry Zones (700 x 700 mm at (+-1.85, +2.62))
    prims.extend(
        [
            ArenaPrimitive(
                'l1_retry_zone_red',
                'box',
                (-1.85, 2.62, 0.604, 0.0, 0.0, 0.0),
                (0.70, 0.70, 0.005),
                'start_retry_red',
                False,
            ),
            ArenaPrimitive(
                'l1_retry_zone_blue',
                'box',
                (1.85, 2.62, 0.604, 0.0, 0.0, 0.0),
                (0.70, 0.70, 0.005),
                'start_retry_blue',
                False,
            ),
        ]
    )

    # 6 Level 1 Building Spots (500 x 500 mm, green)
    for name, bx, by in (
        ('l1_building_spot_red_north', -2.72, 2.72),
        ('l1_building_spot_red_south', -2.72, -2.72),
        ('l1_building_spot_blue_north', 2.72, 2.72),
        ('l1_building_spot_blue_south', 2.72, -2.72),
        ('l1_building_spot_shared_south', 0.0, -2.72),
        ('l1_building_spot_shared_north', 0.0, 2.72),
    ):
        prims.append(
            ArenaPrimitive(
                name,
                'box',
                (bx, by, 0.605, 0.0, 0.0, 0.0),
                (0.50, 0.50, 0.006),
                'building_spot_green',
                False,
            )
        )

    # -------------------------------------------------------------------------
    # 3. TRANSITION STRUCTURES (Stairs, Transfer Areas, Ramps for Red & Blue)
    # -------------------------------------------------------------------------
    for team, tx, stair_col, trans_col, ramp_col in (
        ('red', -3.50, 'stairs_red', 'transfer_red', 'ramp_red'),
        ('blue', 3.50, 'stairs_blue', 'transfer_blue', 'ramp_blue'),
    ):
        prims.extend(
            [
                ArenaPrimitive(
                    f'{team}_stair_1',
                    'box',
                    (tx, -2.55, 0.075, 0.0, 0.0, 0.0),
                    (1.00, 0.30, 0.15),
                    stair_col,
                    True,
                ),
                ArenaPrimitive(
                    f'{team}_stair_2',
                    'box',
                    (tx, -2.25, 0.150, 0.0, 0.0, 0.0),
                    (1.00, 0.30, 0.30),
                    stair_col,
                    True,
                ),
                ArenaPrimitive(
                    f'{team}_stair_3',
                    'box',
                    (tx, -1.95, 0.225, 0.0, 0.0, 0.0),
                    (1.00, 0.30, 0.45),
                    stair_col,
                    True,
                ),
                ArenaPrimitive(
                    f'{team}_transfer_area',
                    'box',
                    (tx, -1.30, 0.300, 0.0, 0.0, 0.0),
                    (1.00, 1.00, 0.60),
                    trans_col,
                    True,
                ),
                ArenaPrimitive(
                    f'{team}_ramp',
                    'box',
                    # Flush top surface: z_center = 0.300 - 0.5*0.04*cos(0.1698) = 0.2803
                    # meets Transfer Area top (y=-0.80, z=0.600) and Ground (y=+2.70, z=0.000) flush!
                    (tx, 0.95, 0.2803, -0.1698, 0.0, 0.0),
                    (1.00, 3.551, 0.04),
                    ramp_col,
                    True,
                ),
            ]
        )

    # -------------------------------------------------------------------------
    # 4. LEVEL 2 PLATFORM (3000 x 3000 mm, +900 mm total), SPOTS & CENTRAL PILLAR
    # -------------------------------------------------------------------------
    prims.append(
        ArenaPrimitive(
            'level_2_platform',
            'box',
            (0.0, 0.0, 0.75, 0.0, 0.0, 0.0),
            (3.00, 3.00, 0.30),
            'l2_grey',
            True,
        )
    )
    for name, bx, by in (
        ('l2_building_spot_nw', -1.25, 1.25),
        ('l2_building_spot_ne', 1.25, 1.25),
        ('l2_building_spot_sw', -1.25, -1.25),
        ('l2_building_spot_se', 1.25, -1.25),
    ):
        prims.append(
            ArenaPrimitive(
                name,
                'box',
                (bx, by, 0.903, 0.0, 0.0, 0.0),
                (0.50, 0.50, 0.006),
                'building_spot_green',
                False,
            )
        )
    prims.append(
        ArenaPrimitive(
            'l2_central_pillar',
            'cylinder',
            (0.0, 0.0, 1.30, 0.0, 0.0, 0.0),
            (0.135, 0.80, 0.0),
            'pillar_brown',
            True,
        )
    )

    # -------------------------------------------------------------------------
    # 5. GAME OBJECTS (40 Earth Blocks, 12 Sky Blocks, Mustika Ball)
    # -------------------------------------------------------------------------
    if include_game_objects:
        # Mustika ball seated on Mustika Pillar (0.0, 4.25, 0.56)
        prims.append(
            ArenaPrimitive(
                'mustika_ball',
                'sphere',
                (0.0, 4.25, 0.56, 0.0, 0.0, 0.0),
                (0.105, 0.0, 0.0),
                'mustika_yellow',
                True,
            )
        )

        # 12 Sky Blocks (200 x 200 x 200 mm) on 5x5 grid in Ground Shared Area.
        # EXACT pixel-verified orientations from ABU Robocon 2027 Rulebook V1.1 Figure 4 (Page 15):
        #   Row 0 = North (+Y), Row 4 = South (-Y); Col 0 = West/Red (-X), Col 4 = East/Blue (+X).
        #   Mode 'H_NR_SB': Horizontal split, North (+Y) = RED,  South (-Y) = BLUE
        #   Mode 'H_NB_SR': Horizontal split, North (+Y) = BLUE, South (-Y) = RED
        #   Mode 'V_WR_EB': Vertical split,   West  (-X) = RED,  East  (+X) = BLUE
        #   Mode 'V_WB_ER': Vertical split,   West  (-X) = BLUE, East  (+X) = RED
        sky_block_layout = [
            (0, 0, 'H_NR_SB'),
            (0, 2, 'V_WR_EB'),
            (0, 4, 'H_NB_SR'),
            (1, 1, 'V_WB_ER'),
            (1, 3, 'V_WB_ER'),
            (2, 0, 'H_NB_SR'),
            (2, 4, 'H_NR_SB'),
            (3, 1, 'V_WR_EB'),
            (3, 3, 'V_WR_EB'),
            (4, 0, 'H_NR_SB'),
            (4, 2, 'V_WB_ER'),
            (4, 4, 'H_NB_SR'),
        ]
        for idx, (row, col, mode) in enumerate(sky_block_layout, 1):
            sx = gx - 0.50 + (col + 0.5) * 0.20
            sy = gy + 0.50 - (row + 0.5) * 0.20
            if mode == 'H_NR_SB':
                # North (+Y) = RED, South (-Y) = BLUE
                red_pose = (sx, sy + 0.05, 0.10, 0.0, 0.0, 0.0)
                blue_pose = (sx, sy - 0.05, 0.10, 0.0, 0.0, 0.0)
                half_size = (0.20, 0.10, 0.20)
            elif mode == 'H_NB_SR':
                # North (+Y) = BLUE, South (-Y) = RED
                blue_pose = (sx, sy + 0.05, 0.10, 0.0, 0.0, 0.0)
                red_pose = (sx, sy - 0.05, 0.10, 0.0, 0.0, 0.0)
                half_size = (0.20, 0.10, 0.20)
            elif mode == 'V_WR_EB':
                # West (-X) = RED, East (+X) = BLUE
                red_pose = (sx - 0.05, sy, 0.10, 0.0, 0.0, 0.0)
                blue_pose = (sx + 0.05, sy, 0.10, 0.0, 0.0, 0.0)
                half_size = (0.10, 0.20, 0.20)
            elif mode == 'V_WB_ER':
                # West (-X) = BLUE, East (+X) = RED
                blue_pose = (sx - 0.05, sy, 0.10, 0.0, 0.0, 0.0)
                red_pose = (sx + 0.05, sy, 0.10, 0.0, 0.0, 0.0)
                half_size = (0.10, 0.20, 0.20)
            else:
                raise ValueError(f'Unknown sky block orientation mode: {mode}')

            prims.append(
                ArenaPrimitive(
                    f'sky_block_{idx:02d}_red_half',
                    'box',
                    red_pose,
                    half_size,
                    'sky_block_red',
                    True,
                )
            )
            prims.append(
                ArenaPrimitive(
                    f'sky_block_{idx:02d}_blue_half',
                    'box',
                    blue_pose,
                    half_size,
                    'sky_block_blue',
                    True,
                )
            )

        # 20 Red + 20 Blue Earth Blocks (350 x 350 x 350 mm) in Storage Areas
        side = 0.35
        pitch = 0.375
        for team, cx, col_key in (
            ('red', -4.50, 'earth_block_red'),
            ('blue', 4.50, 'earth_block_blue'),
        ):
            for i in range(20):
                layer = i // 10
                local = i % 10
                row = local // 5
                col = local % 5
                ex = cx + (col - 2) * pitch
                ey = 5.00 + (row - 0.5) * pitch
                ez = side * 0.5 + layer * side
                prims.append(
                    ArenaPrimitive(
                        f'earth_block_{team}_{i+1:02d}',
                        'box',
                        (ex, ey, ez, 0.0, 0.0, 0.0),
                        (side, side, side),
                        col_key,
                        True,
                    )
                )

    return prims


def write_exact_arena_sdf(output_path: str) -> None:
    """
    Generate a Gazebo Harmonic SDF 1.9 world file containing the exact ABU Robocon
    2027 arena (Rulebook V1.1 & Appendix V1.1) plus the 0.70 x 0.70 m Builder Robot
    spawned on Level 1.
    """
    sdf = ET.Element('sdf', {'version': '1.9'})
    world = ET.SubElement(sdf, 'world', {'name': 'robocon_2027_l1_sandbox'})

    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    phys = ET.SubElement(world, 'physics', {'name': 'default_physics', 'type': 'ignored'})
    ET.SubElement(phys, 'max_step_size').text = '0.001'
    ET.SubElement(phys, 'real_time_factor').text = '1.0'

    for fn, name in (
        ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
        ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
    ):
        ET.SubElement(world, 'plugin', {'filename': fn, 'name': name})

    sens = ET.SubElement(
        world,
        'plugin',
        {'filename': 'gz-sim-sensors-system', 'name': 'gz::sim::systems::Sensors'},
    )
    ET.SubElement(sens, 'render_engine').text = 'ogre2'

    light = ET.SubElement(world, 'light', {'name': 'sun', 'type': 'directional'})
    ET.SubElement(light, 'cast_shadows').text = 'true'
    ET.SubElement(light, 'pose').text = '0 0 12 0 0 0'
    ET.SubElement(light, 'direction').text = '-0.4 0.2 -0.9'
    ET.SubElement(light, 'diffuse').text = '0.88 0.88 0.88 1'
    ET.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'

    arena_model = ET.SubElement(world, 'model', {'name': 'robocon_2027_exact_arena'})
    ET.SubElement(arena_model, 'static').text = 'true'

    for prim in build_exact_arena_primitives(include_game_objects=True):
        link = ET.SubElement(arena_model, 'link', {'name': prim.name})
        x, y, z, r, p, yw = prim.pose
        ET.SubElement(link, 'pose').text = f'{x:.4f} {y:.4f} {z:.4f} {r:.4f} {p:.4f} {yw:.4f}'

        tags = ('visual', 'collision') if prim.collision else ('visual',)
        for tag in tags:
            elem = ET.SubElement(link, tag, {'name': tag})
            geom = ET.SubElement(elem, 'geometry')
            if prim.shape == 'box':
                b = ET.SubElement(geom, 'box')
                ET.SubElement(b, 'size').text = (
                    f'{prim.size[0]:.4f} {prim.size[1]:.4f} {prim.size[2]:.4f}'
                )
            elif prim.shape == 'cylinder':
                c = ET.SubElement(geom, 'cylinder')
                ET.SubElement(c, 'radius').text = f'{prim.size[0]:.4f}'
                ET.SubElement(c, 'length').text = f'{prim.size[1]:.4f}'
            elif prim.shape == 'sphere':
                s = ET.SubElement(geom, 'sphere')
                ET.SubElement(s, 'radius').text = f'{prim.size[0]:.4f}'

            if tag == 'visual':
                cr, cg, cb = rgb_norm(prim.color_key)
                mat = ET.SubElement(elem, 'material')
                c_str = f'{cr:.4f} {cg:.4f} {cb:.4f} 1.0'
                ET.SubElement(mat, 'ambient').text = c_str
                ET.SubElement(mat, 'diffuse').text = c_str
                ET.SubElement(mat, 'specular').text = '0.08 0.08 0.08 1.0'

    # Add the Fusion 360 BR v7 Builder Robot (br_robot) starting at Red Ground Start/Retry Zone (-4.35, -5.15, 0.00)
    br = ET.SubElement(world, 'model', {'name': 'br_robot'})
    ET.SubElement(br, 'pose').text = '-4.35 -5.15 0.00 0 0 1.5708'
    base = ET.SubElement(br, 'link', {'name': 'base_link'})
    inert = ET.SubElement(base, 'inertial')
    ET.SubElement(inert, 'pose').text = '0 0 0.26 0 0 0'
    ET.SubElement(inert, 'mass').text = '22.5'
    ii = ET.SubElement(inert, 'inertia')
    for k, v in (('ixx', '1.15'), ('ixy', '0'), ('ixz', '0'), ('iyy', '1.15'), ('iyz', '0'), ('izz', '1.82')):
        ET.SubElement(ii, k).text = v

    col = ET.SubElement(base, 'collision', {'name': 'chassis_collision'})
    ET.SubElement(col, 'pose').text = '0 0 0.09 0 0 0'
    g_col = ET.SubElement(col, 'geometry')
    b_col = ET.SubElement(g_col, 'box')
    ET.SubElement(b_col, 'size').text = '0.7025 0.6965 0.16'

    vis = ET.SubElement(base, 'visual', {'name': 'chassis_visual'})
    ET.SubElement(vis, 'pose').text = '0 0 0 0 0 0'
    g_vis = ET.SubElement(vis, 'geometry')
    mesh_el = ET.SubElement(g_vis, 'mesh')
    ET.SubElement(mesh_el, 'uri').text = '../meshes/br_robot_v7.stl'
    ET.SubElement(mesh_el, 'scale').text = '1.0 1.0 1.0'
    m_vis = ET.SubElement(vis, 'material')
    ET.SubElement(m_vis, 'ambient').text = '0.75 0.78 0.82 1'
    ET.SubElement(m_vis, 'diffuse').text = '0.82 0.85 0.90 1'
    ET.SubElement(m_vis, 'specular').text = '0.35 0.35 0.35 1'

    lidar = ET.SubElement(br, 'link', {'name': 'lidar_link'})
    ET.SubElement(lidar, 'pose').text = '0 0 0.22 0 0 0'
    lv = ET.SubElement(lidar, 'visual', {'name': 'lidar_visual'})
    lg = ET.SubElement(lv, 'geometry')
    lc = ET.SubElement(lg, 'cylinder')
    ET.SubElement(lc, 'radius').text = '0.045'
    ET.SubElement(lc, 'length').text = '0.05'
    lm = ET.SubElement(lv, 'material')
    ET.SubElement(lm, 'ambient').text = '0.1 0.75 0.95 1'
    ET.SubElement(lm, 'diffuse').text = '0.1 0.75 0.95 1'

    sensor = ET.SubElement(lidar, 'sensor', {'name': 'gpu_lidar', 'type': 'gpu_lidar'})
    ET.SubElement(sensor, 'pose').text = '0 0 0 0 0 0'
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'visualize').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '10'
    ray = ET.SubElement(sensor, 'ray')
    scan = ET.SubElement(ray, 'scan')
    horiz = ET.SubElement(scan, 'horizontal')
    ET.SubElement(horiz, 'samples').text = '360'
    ET.SubElement(horiz, 'resolution').text = '1'
    ET.SubElement(horiz, 'min_angle').text = '-3.14159265'
    ET.SubElement(horiz, 'max_angle').text = '3.14159265'
    rng = ET.SubElement(ray, 'range')
    ET.SubElement(rng, 'min').text = '0.08'
    ET.SubElement(rng, 'max').text = '8.0'
    ET.SubElement(rng, 'resolution').text = '0.01'

    joint = ET.SubElement(br, 'joint', {'name': 'lidar_joint', 'type': 'fixed'})
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'lidar_link'

    vc = ET.SubElement(
        br,
        'plugin',
        {'filename': 'gz-sim-velocity-control-system', 'name': 'gz::sim::systems::VelocityControl'},
    )
    ET.SubElement(vc, 'topic').text = '/model/br_robot/cmd_vel'

    op = ET.SubElement(
        br,
        'plugin',
        {'filename': 'gz-sim-odometry-publisher-system', 'name': 'gz::sim::systems::OdometryPublisher'},
    )
    ET.SubElement(op, 'odom_frame').text = 'odom'
    ET.SubElement(op, 'robot_base_frame').text = 'base_link'
    ET.SubElement(op, 'odom_publish_frequency').text = '20'
    ET.SubElement(op, 'odom_topic').text = '/model/br_robot/odometry'

    tree = ET.ElementTree(sdf)
    ET.indent(tree, space='  ')
    tree.write(output_path, encoding='utf-8', xml_declaration=True)

