#!/usr/bin/env python3
"""
Full ABU Robocon 2027 ("The Pursuit of Mustika Nusantara") Rulebook V1.1
End-to-End Match Gameplay Scenario Director.

Orchestrates a complete, Rulebook-compliant Red Team match demonstration using
the REAL live navigation stack (LiDAR -> Occupancy Grid -> EDT Traversability
Costmap -> Cost-Aware A* Pathfinding -> Holonomic Mecanum Follower & 3-Stage
Wheel-Lift Climber):

  1. BR starts in Ground Start/Retry Zone (-4.35, -5.15, z=0.00m), drives via
     Ground A* to Red Stairs, executes Wheel-Lift Climb onto Level 1, and waits
     at the Red Transfer Area loading zone (-2.50, -1.30, z=0.60m).
  2. OUR TR operates STRICTLY on the Ground Floor + Red Ramp + Red Transfer Area
     (Rules 3.2.1, 3.4.5, 4.3.1 — NEVER crosses into L1 beyond Transfer Area!),
     harvesting Earth Blocks from Red Storage Area and Sky Blocks from the
     5x5 Ground Shared Area and placing them in the Transfer Area (+5 pts each).
  3. Tower #1 (Red Exclusive L1 Building Spot South, (-2.72, -2.72)):
       Earth Block 1 (+10) -> Earth Block 2 (+20) -> Sky Block 1 Red UP (+40).
  4. Tower #2 (L1 Shared Area Building Spot North, (0.00, +2.72)):
       Earth Block 3 (+10) -> Earth Block 4 (+20) -> Sky Block 2 Red UP (+40).
       -> Fulfills SANCTUARY MANDATE (Rule 3.6.1: >=2 Complete Towers standing
          simultaneously with >=1 in a Shared Area!).
  5. Mustika Retrieval & Enshrinement (Rules 3.4.5, 3.6.3, 3.6.4, 4.5.2):
       TR fetches the Golden Mustika from the North Mustika Pillar (0.0, +4.25),
       drives up the Red Ramp to Transfer Area, and performs DIRECT HANDOVER
       to BR (Mustika NEVER touches the floor!).
       BR runs L1 A* to the L2 wall, executes 3-Stage Wheel-Lift Climb onto
       Level 2 (+0.90m), navigates via L2 A* to the Central Pillar (0.0, 0.0),
       and enshrines the Mustika (+250 pts -> Total Score = 420 pts!).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


# Exact Rulebook V1.1 / Appendix coordinates (meters)
BR_START = (-4.35, -5.15, 0.00)
TR_START = (-5.15, -5.15, 0.00)

BR_STAIRS_APPROACH = (-3.50, -3.25, 0.00)
BR_TRANSFER_STAGE = (-2.50, -1.30, 0.60)
BR_TOWER1_APPROACH = (-2.22, -2.22, 0.60)   # Standoff for Red South L1 spot (-2.72, -2.72)
BR_TOWER2_APPROACH = (-0.55, 2.45, 0.60)    # Standoff for North L1 Shared spot (0.00, +2.72)
BR_L2_CLIMB_STAGE = (-2.05, 0.00, 0.60)     # L1 staging before wheel-lift to L2
BR_L2_PILLAR_APPROACH = (-0.62, 0.00, 0.90) # L2 standoff for Central Pillar (0.00, 0.00)

TOWER1_SPOT = (-2.72, -2.72, 0.60)          # Red Exclusive L1 Building Spot South
TOWER2_SPOT = (0.00, 2.72, 0.60)            # L1 Shared Area Building Spot North
TRANSFER_DROP = (-3.50, -1.30, 0.60)        # Red Transfer Area center
CENTRAL_PILLAR_TOP = (0.00, 0.00, 1.78)     # Central Pillar receptacle (z=0.90 + 0.80 + ball)
MUSTIKA_PILLAR_TOP = (0.00, 4.25, 0.58)

# TR Ground + Red Ramp + Transfer Area waypoints (STRICTLY Rule 3.2.1 / 4.3.1 compliant)
TR_STORAGE_PICK = (-4.50, 4.10, 0.00)
TR_SKY_PICK = (-0.85, -4.25, 0.00)
TR_MUSTIKA_PICK = (-0.65, 4.25, 0.00)
TR_RAMP_BOTTOM = (-3.50, 2.85, 0.00)
TR_RAMP_MID = (-3.50, 0.95, 0.30)
TR_TRANSFER_WAIT = (-3.50, -1.30, 0.60)


def _step_along_waypoints(
    pos: Tuple[float, float, float],
    waypoints: List[Tuple[float, float, float]],
    wp_idx: int,
    speed_mps: float,
    dt: float,
) -> Tuple[Tuple[float, float, float], int, bool]:
    """Advance a 3D point along a polyline at constant speed. Returns (new_pos, new_idx, arrived)."""
    if not waypoints or wp_idx >= len(waypoints):
        return pos, wp_idx, True

    x, y, z = pos
    rem = max(0.0, speed_mps * dt)
    idx = wp_idx
    while idx < len(waypoints) and rem > 1e-6:
        tx, ty, tz = waypoints[idx]
        dx, dy, dz = tx - x, ty - y, tz - z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= rem + 1e-6:
            x, y, z = tx, ty, tz
            rem -= dist
            idx += 1
        else:
            frac = rem / dist
            x += dx * frac
            y += dy * frac
            z += dz * frac
            rem = 0.0
    return (x, y, z), idx, (idx >= len(waypoints))


@dataclass
class PlacedObject:
    """A single Earth Block, Sky Block, or Mustika placed in the arena."""
    obj_id: int
    kind: str  # 'earth' | 'sky' | 'mustika'
    x: float
    y: float
    z: float


@dataclass
class MissionStep:
    """Declarative step in the ABU Robocon 2027 match sequence."""
    name: str
    description: str
    # TR action: ('fetch_earth' | 'fetch_sky' | 'fetch_mustika' | 'park_ground' | 'wait')
    tr_task: str
    # BR action: ('nav_goal' | 'climb_l1' | 'climb_l2' | 'pickup_transfer' | 'place_tower' | 'handover_mustika' | 'enshrine_mustika')
    br_task: str
    br_goal: Optional[Tuple[float, float, float]] = None
    tower_id: int = 0          # 1 or 2
    block_kind: str = ''       # 'earth' | 'sky' | 'mustika'
    layer_idx: int = 0         # 1 (bottom earth), 2 (mid earth), 3 (top sky)


def build_official_match_sequence() -> List[MissionStep]:
    """
    Build the complete ABU Robocon 2027 Rulebook V1.1 Red Team gameplay script:
      - BR climbs to L1 first and waits at Transfer Area
      - TR stays on Ground + Red Ramp + Transfer Area only
      - Tower #1 (Red L1 Exclusive): Earth -> Earth -> Sky (70 pts + 15 transfer)
      - Tower #2 (L1 Shared Area):   Earth -> Earth -> Sky (70 pts + 15 transfer)
        -> Unlocks Sanctuary Mandate!
      - TR fetches Mustika -> Direct Handover to BR at Transfer Area (no floor drop!)
      - BR climbs L1 -> L2 via Wheel-Lift -> Enshrines Mustika on Central Pillar (+250 pts)
      - Total Score: 420 pts!
    """
    return [
        MissionStep(
            name='PHASE 1a: BR Approaches Red Stairs (Ground A*)',
            description='BR drives via Ground A* to Stairs; TR harvests Earth Block #1 from Storage Area',
            tr_task='fetch_earth',
            br_task='nav_goal',
            br_goal=BR_STAIRS_APPROACH,
        ),
        MissionStep(
            name='PHASE 1b: BR Climbs to Level 1 & Waits at Loading Zone',
            description='BR activates 3-Stage Wheel-Lift onto Level 1 while TR delivers Earth Block #1 via Red Ramp',
            tr_task='wait_deliver_earth',
            br_task='climb_l1',
            br_goal=BR_TRANSFER_STAGE,
        ),
        MissionStep(
            name='PHASE 2a: BR Picks Up Earth Block #1 (Tower 1 Base)',
            description='TR placed Earth Block #1 in Transfer Area (+5 pts); BR autonomously detects & lifts it',
            tr_task='park_ground',
            br_task='pickup_transfer',
            block_kind='earth',
        ),
        MissionStep(
            name='PHASE 2b: BR Navigates L1 A* to Building Spot #1',
            description='BR carries Earth Block #1 across L1 costmap to Red Exclusive Spot (-2.72,-2.72); TR fetches Earth #2',
            tr_task='fetch_earth',
            br_task='nav_goal',
            br_goal=BR_TOWER1_APPROACH,
        ),
        MissionStep(
            name='PHASE 2c: BR Places Earth Block #1 (Tower 1 Layer 1)',
            description='Earth Block #1 placed on Building Spot #1 (+10 pts)',
            tr_task='wait_deliver_earth',
            br_task='place_tower',
            tower_id=1,
            block_kind='earth',
            layer_idx=1,
        ),
        MissionStep(
            name='PHASE 3a: BR Returns to Transfer Area for Earth Block #2',
            description='BR runs L1 A* back to Transfer Area; TR finishes placing Earth Block #2 (+5 pts)',
            tr_task='wait_deliver_earth',
            br_task='nav_goal',
            br_goal=BR_TRANSFER_STAGE,
        ),
        MissionStep(
            name='PHASE 3b: BR Picks Up Earth Block #2',
            description='BR lifts Earth Block #2; TR heads to Ground Shared Area for Sky Block #1',
            tr_task='fetch_sky',
            br_task='pickup_transfer',
            block_kind='earth',
        ),
        MissionStep(
            name='PHASE 3c: BR Navigates L1 A* & Stacks Earth Block #2',
            description='BR carries Earth Block #2 to Tower #1 while TR brings Sky Block #1 up Red Ramp',
            tr_task='wait_deliver_sky',
            br_task='nav_goal',
            br_goal=BR_TOWER1_APPROACH,
        ),
        MissionStep(
            name='PHASE 3d: Tower #1 Now 2 Earth Blocks High!',
            description='Earth Block #2 stacked on Tower #1 (+20 pts) -> Ready for Sky Block #1!',
            tr_task='wait_deliver_sky',
            br_task='place_tower',
            tower_id=1,
            block_kind='earth',
            layer_idx=2,
        ),
        MissionStep(
            name='PHASE 4a: BR Returns to Transfer Area for Sky Block #1',
            description='BR runs L1 A* to Transfer Area where TR placed Sky Block #1 (+5 pts)',
            tr_task='wait_deliver_sky',
            br_task='nav_goal',
            br_goal=BR_TRANSFER_STAGE,
        ),
        MissionStep(
            name='PHASE 4b: BR Picks Up Sky Block #1',
            description='BR lifts Sky Block #1 (Red face UP); TR starts fetching Earth Block #3 for Tower #2',
            tr_task='fetch_earth',
            br_task='pickup_transfer',
            block_kind='sky',
        ),
        MissionStep(
            name='PHASE 4c: BR Crowns Tower #1 with Sky Block #1!',
            description='BR navigates L1 A* to Building Spot #1 to crown Complete Tower #1',
            tr_task='wait_deliver_earth',
            br_task='nav_goal',
            br_goal=BR_TOWER1_APPROACH,
        ),
        MissionStep(
            name='PHASE 4d: COMPLETE TOWER #1 BUILT (Red Exclusive L1)!',
            description='Sky Block #1 placed Red-UP (+40 pts -> Tower #1 = 70 pts)! 1 more tower in Shared Area unlocks Mustika!',
            tr_task='wait_deliver_earth',
            br_task='place_tower',
            tower_id=1,
            block_kind='sky',
            layer_idx=3,
        ),
        # ---- TOWER #2 (L1 Shared Area North -> Unlocks Sanctuary Mandate!) ----
        MissionStep(
            name='PHASE 5a: BR Returns for Earth Block #3 (Tower 2 - L1 Shared Area)',
            description='BR navigates L1 A* back to Transfer Area for Tower #2 (L1 Shared Area unlocks Mustika!)',
            tr_task='wait_deliver_earth',
            br_task='nav_goal',
            br_goal=BR_TRANSFER_STAGE,
        ),
        MissionStep(
            name='PHASE 5b: BR Picks Up Earth Block #3',
            description='BR lifts Earth Block #3; TR fetches Earth Block #4 from Red Storage Area',
            tr_task='fetch_earth',
            br_task='pickup_transfer',
            block_kind='earth',
        ),
        MissionStep(
            name='PHASE 5c: BR Navigates L1 A* North to L1 Shared Area',
            description='BR routes around L2 wall & live ENEMY BR to L1 Shared Building Spot (0.00, +2.72)',
            tr_task='wait_deliver_earth',
            br_task='nav_goal',
            br_goal=BR_TOWER2_APPROACH,
        ),
        MissionStep(
            name='PHASE 5d: BR Places Earth Block #3 (Shared Tower Layer 1)',
            description='Earth Block #3 placed in L1 Shared Area (+10 pts)',
            tr_task='wait_deliver_earth',
            br_task='place_tower',
            tower_id=2,
            block_kind='earth',
            layer_idx=1,
        ),
        MissionStep(
            name='PHASE 6a: BR Returns to Transfer Area for Earth Block #4',
            description='BR runs L1 A* back to Transfer Area where TR delivered Earth Block #4 (+5 pts)',
            tr_task='wait_deliver_earth',
            br_task='nav_goal',
            br_goal=BR_TRANSFER_STAGE,
        ),
        MissionStep(
            name='PHASE 6b: BR Picks Up Earth Block #4',
            description='BR lifts Earth Block #4; TR harvests Sky Block #2 from Ground Shared Area',
            tr_task='fetch_sky',
            br_task='pickup_transfer',
            block_kind='earth',
        ),
        MissionStep(
            name='PHASE 6c: BR Navigates L1 A* to Shared Building Spot',
            description='BR carries Earth Block #4 to L1 Shared Spot while TR delivers Sky Block #2',
            tr_task='wait_deliver_sky',
            br_task='nav_goal',
            br_goal=BR_TOWER2_APPROACH,
        ),
        MissionStep(
            name='PHASE 6d: Shared Tower #2 Now 2 Earth Blocks High!',
            description='Earth Block #4 stacked (+20 pts) -> Ready for Sky Block #2 to unlock Mustika!',
            tr_task='wait_deliver_sky',
            br_task='place_tower',
            tower_id=2,
            block_kind='earth',
            layer_idx=2,
        ),
        MissionStep(
            name='PHASE 7a: BR Returns to Transfer Area for Sky Block #2',
            description='BR runs L1 A* to Transfer Area where TR placed Sky Block #2 (+5 pts)',
            tr_task='wait_deliver_sky',
            br_task='nav_goal',
            br_goal=BR_TRANSFER_STAGE,
        ),
        MissionStep(
            name='PHASE 7b: BR Picks Up Sky Block #2',
            description='BR lifts Sky Block #2 (Red face UP); TR waits on Ground until Sanctuary Mandate unlocks!',
            tr_task='park_ground',
            br_task='pickup_transfer',
            block_kind='sky',
        ),
        MissionStep(
            name='PHASE 7c: BR Navigates L1 A* to Crown Shared Tower #2',
            description='BR routes to L1 Shared Building Spot to complete 2nd Tower',
            tr_task='park_ground',
            br_task='nav_goal',
            br_goal=BR_TOWER2_APPROACH,
        ),
        MissionStep(
            name='PHASE 7d: SANCTUARY MANDATE FULFILLED! (2 Complete Towers, 1 Shared)',
            description='Sky Block #2 placed (+40 pts -> 170 pts)! Rule 3.6.1 UNLOCKED -> TR may now retrieve Mustika!',
            tr_task='park_ground',
            br_task='place_tower',
            tower_id=2,
            block_kind='sky',
            layer_idx=3,
        ),
        # ---- MUSTIKA RETRIEVAL, DIRECT HANDOVER & L2 ENSHRINEMENT ----
        MissionStep(
            name='PHASE 8a: TR Fetches Mustika Pillar -> Red Ramp; BR Returns to Transfer',
            description='TR drives Ground Floor to Mustika Pillar (0,+4.25) & up Red Ramp; BR runs L1 A* to Transfer Area',
            tr_task='fetch_mustika',
            br_task='nav_goal',
            br_goal=BR_TRANSFER_STAGE,
        ),
        MissionStep(
            name='PHASE 8b: DIRECT MUSTIKA HANDOVER (TR -> BR, Never Touches Floor!)',
            description='Rule 3.4.5: TR hands Golden Mustika directly into BR gripper inside Transfer Area!',
            tr_task='wait_handover_mustika',
            br_task='handover_mustika',
            block_kind='mustika',
        ),
        MissionStep(
            name='PHASE 9a: BR Navigates L1 A* to Level 2 Climb Position',
            description='BR carries Mustika along L1 costmap to Level 2 West step (-2.05, 0.00)',
            tr_task='park_ground',
            br_task='nav_goal',
            br_goal=BR_L2_CLIMB_STAGE,
        ),
        MissionStep(
            name='PHASE 9b: BR Wheel-Lift Climbs L1 (+0.60m) -> Level 2 (+0.90m)!',
            description='BR executes 3-Stage Wheel-Lifting Step-Climb up the +300mm Level 2 cliff!',
            tr_task='park_ground',
            br_task='climb_l2',
            br_goal=(-0.95, 0.00, 0.90),
        ),
        MissionStep(
            name='PHASE 9c: BR Navigates Level 2 A* to Central Pillar',
            description='BR runs Level 2 A* avoiding L2 drop-offs & Central Pillar radius to (-0.62, 0.00)',
            tr_task='park_ground',
            br_task='nav_goal',
            br_goal=BR_L2_PILLAR_APPROACH,
        ),
        MissionStep(
            name='PHASE 9d: MUSTIKA ENSHRINED ON CENTRAL PILLAR! (+250 PTS -> 420 TOTAL!)',
            description='BR releases Golden Mustika stably into Central Pillar receptacle (0,0,z=1.78m)! Match continues to 3:00!',
            tr_task='park_ground',
            br_task='enshrine_mustika',
            block_kind='mustika',
        ),
    ]


class MatchMissionLogic:
    """
    Pure-Python testable state machine for the ABU Robocon 2027 Rulebook V1.1
    gameplay simulation (TR logistics + BR A* navigation + Tower & Mustika rules).
    """

    def __init__(self) -> None:
        self.steps: List[MissionStep] = build_official_match_sequence()
        self.reset()

    def reset(self) -> None:
        self.active: bool = False
        self.completed: bool = False
        self.step_idx: int = 0
        self.step_timer_s: float = 0.0

        # Scoreboard & Rulebook state
        self.score_transfer: int = 0
        self.score_towers: int = 0
        self.score_mustika: int = 0
        self.complete_towers_exclusive: int = 0
        self.complete_towers_shared: int = 0
        self.sanctuary_mandate_fulfilled: bool = False
        self.mustika_enshrined: bool = False

        # Object locations
        self.tr_pos: Tuple[float, float, float] = TR_START
        self.tr_waypoints: List[Tuple[float, float, float]] = []
        self.tr_wp_idx: int = 0
        self.tr_carrying: Optional[str] = None     # 'earth' | 'sky' | 'mustika' | None
        self.tr_active_task: str = 'idle'
        self.tr_pending_task: Optional[str] = None
        self.tr_delivered_this_trip: bool = False
        self.tr_picked_this_trip: bool = False

        self.transfer_block: Optional[str] = None  # 'earth' | 'sky' | None (NEVER 'mustika'!)
        self.br_carrying: Optional[str] = None     # 'earth' | 'sky' | 'mustika' | None
        self.placed_objects: List[PlacedObject] = []
        self._next_obj_id: int = 100

        # BR coordination flags
        self.goal_sent_for_step: bool = False
        self.climb_sent_for_step: bool = False
        self.last_goal_resend_s: float = 0.0

    @property
    def total_score(self) -> int:
        return self.score_transfer + self.score_towers + self.score_mustika

    @property
    def current_step(self) -> Optional[MissionStep]:
        if 0 <= self.step_idx < len(self.steps):
            return self.steps[self.step_idx]
        return None

    def start(self) -> None:
        self.reset()
        self.active = True
        self._configure_tr_for_step(self.steps[0])

    def stop(self) -> None:
        self.active = False

    def _plan_tr_route(self, task: str) -> None:
        """Assign strictly Rulebook-legal Ground + Red Ramp + Transfer Area waypoints for TR."""
        tx, ty, tz = self.tr_pos
        on_transfer = tz > 0.35 or (abs(tx - (-3.50)) < 0.45 and ty < 2.50)

        def descend_prefix() -> List[Tuple[float, float, float]]:
            if on_transfer:
                return [TR_RAMP_MID, TR_RAMP_BOTTOM]
            return []

        if task == 'fetch_earth':
            self.tr_active_task = 'fetch_earth'
            self.tr_picked_this_trip = False
            self.tr_delivered_this_trip = False
            self.tr_waypoints = descend_prefix() + [
                (-4.50, 3.60, 0.00),
                TR_STORAGE_PICK,          # pickup trigger index
                (-4.20, 3.40, 0.00),
                TR_RAMP_BOTTOM,
                TR_RAMP_MID,
                TR_TRANSFER_WAIT,         # drop trigger
            ]
            self.tr_wp_idx = 0
        elif task == 'fetch_sky':
            self.tr_active_task = 'fetch_sky'
            self.tr_picked_this_trip = False
            self.tr_delivered_this_trip = False
            self.tr_waypoints = descend_prefix() + [
                (-4.50, 3.20, 0.00),
                (-4.50, -3.50, 0.00),
                TR_SKY_PICK,              # pickup trigger
                (-4.50, -3.50, 0.00),
                (-4.50, 3.20, 0.00),
                TR_RAMP_BOTTOM,
                TR_RAMP_MID,
                TR_TRANSFER_WAIT,         # drop trigger
            ]
            self.tr_wp_idx = 0
        elif task == 'fetch_mustika':
            # Rule 3.6.7 & 4.5.1 guard: Mustika retrieval ONLY allowed after Sanctuary Mandate!
            if not self.sanctuary_mandate_fulfilled:
                raise RuntimeError('Rule 3.6.7 violation: TR attempted Mustika before Sanctuary Mandate!')
            self.tr_active_task = 'fetch_mustika'
            self.tr_picked_this_trip = False
            self.tr_delivered_this_trip = False
            self.tr_waypoints = descend_prefix() + [
                (-2.20, 4.25, 0.00),
                TR_MUSTIKA_PICK,          # pickup trigger
                (-2.20, 4.25, 0.00),
                TR_RAMP_BOTTOM,
                TR_RAMP_MID,
                TR_TRANSFER_WAIT,         # direct handover wait
            ]
            self.tr_wp_idx = 0
        elif task == 'park_ground':
            if on_transfer or tz > 0.05:
                self.tr_active_task = 'park_ground'
                self.tr_waypoints = [TR_RAMP_MID, TR_RAMP_BOTTOM, (-4.40, 3.40, 0.00)]
                self.tr_wp_idx = 0

    def _configure_tr_for_step(self, step: MissionStep) -> None:
        tr_busy = self.tr_active_task in ('fetch_earth', 'fetch_sky', 'fetch_mustika') and not self.tr_delivered_this_trip
        if step.tr_task in ('fetch_earth', 'fetch_sky', 'fetch_mustika'):
            if tr_busy:
                # Never abort an in-progress delivery; queue next fetch cleanly!
                self.tr_pending_task = step.tr_task
            elif self.tr_active_task != step.tr_task or self.tr_delivered_this_trip:
                self.tr_pending_task = None
                self._plan_tr_route(step.tr_task)
        elif step.tr_task == 'park_ground':
            if tr_busy:
                self.tr_pending_task = 'park_ground'
            elif self.tr_active_task != 'park_ground' and self.tr_carrying is None:
                self.tr_pending_task = None
                self._plan_tr_route('park_ground')

    def step_tr(self, dt: float, tr_speed_mps: float = 2.6) -> None:
        """Advance OUR TR smoothly along its Ground + Ramp + Transfer trajectory."""
        if not self.active or not self.tr_waypoints:
            return

        self.tr_pos, self.tr_wp_idx, arrived = _step_along_waypoints(
            self.tr_pos, self.tr_waypoints, self.tr_wp_idx, tr_speed_mps, dt
        )

        # Rule 4.3.1 assertion: TR must NEVER enter Level 1 beyond Transfer Area (x > -3.00 while z > 0.25)
        if self.tr_pos[2] > 0.25 and self.tr_pos[0] > -2.99:
            self.tr_pos = (-3.00, self.tr_pos[1], self.tr_pos[2])

        # Check pickup triggers when passing Storage / Ground Shared / Mustika Pillar
        if not self.tr_picked_this_trip:
            if self.tr_active_task == 'fetch_earth' and math.hypot(
                self.tr_pos[0] - TR_STORAGE_PICK[0], self.tr_pos[1] - TR_STORAGE_PICK[1]
            ) < 0.20:
                self.tr_carrying = 'earth'
                self.tr_picked_this_trip = True
            elif self.tr_active_task == 'fetch_sky' and math.hypot(
                self.tr_pos[0] - TR_SKY_PICK[0], self.tr_pos[1] - TR_SKY_PICK[1]
            ) < 0.20:
                self.tr_carrying = 'sky'
                self.tr_picked_this_trip = True
            elif self.tr_active_task == 'fetch_mustika' and math.hypot(
                self.tr_pos[0] - TR_MUSTIKA_PICK[0], self.tr_pos[1] - TR_MUSTIKA_PICK[1]
            ) < 0.20:
                self.tr_carrying = 'mustika'
                self.tr_picked_this_trip = True

        # Check Transfer Area delivery upon arrival
        if arrived and self.tr_picked_this_trip and not self.tr_delivered_this_trip:
            if self.tr_active_task in ('fetch_earth', 'fetch_sky'):
                # Place block inside Transfer Area -> +5 Transfer Points (Rule 8.1)
                self.transfer_block = self.tr_carrying
                self.tr_carrying = None
                self.tr_delivered_this_trip = True
                self.score_transfer += 5
                if self.tr_pending_task is not None:
                    nxt_task = self.tr_pending_task
                    self.tr_pending_task = None
                    self._plan_tr_route(nxt_task)
            elif self.tr_active_task == 'fetch_mustika':
                # Rule 3.4.5: Mustika NEVER placed on floor — TR holds it for direct handover to BR!
                self.tr_delivered_this_trip = True

    def advance_mission(
        self,
        br_pose: Tuple[float, float, float],
        climb_idle: bool,
        dt: float,
    ) -> Dict[str, object]:
        """
        Evaluate one tick of the full match state machine.
        Returns action directives for the ROS 2 node:
          {'send_goal': (x,y,z)|None, 'send_climb': str|None, 'phase_changed': bool}
        """
        actions: Dict[str, object] = {
            'send_goal': None,
            'send_climb': None,
            'phase_changed': False,
        }
        if not self.active or self.completed:
            return actions

        self.step_tr(dt)
        step = self.current_step
        if step is None:
            self.completed = True
            return actions

        self.step_timer_s += dt
        bx, by, bz = br_pose
        done_step = False

        if step.br_task == 'nav_goal' and step.br_goal is not None:
            gx, gy, gz = step.br_goal
            if not self.goal_sent_for_step or (self.step_timer_s - self.last_goal_resend_s) >= 2.2:
                actions['send_goal'] = (gx, gy, gz)
                self.goal_sent_for_step = True
                self.last_goal_resend_s = self.step_timer_s
            dist_xy = math.hypot(bx - gx, by - gy)
            if dist_xy <= 0.24 and abs(bz - gz) <= 0.22:
                done_step = True

        elif step.br_task == 'climb_l1':
            if not self.climb_sent_for_step:
                actions['send_climb'] = 'mission_climb_l1'
                self.climb_sent_for_step = True
            elif climb_idle and bz >= 0.52 and self.step_timer_s >= 1.2:
                # Wait until TR has finished placing Earth Block #1 in Transfer Area
                if self.transfer_block == 'earth':
                    done_step = True

        elif step.br_task == 'climb_l2':
            if not self.climb_sent_for_step:
                actions['send_climb'] = 'mission_climb_l2'
                self.climb_sent_for_step = True
            elif climb_idle and bz >= 0.85 and self.step_timer_s >= 1.2:
                done_step = True

        elif step.br_task == 'pickup_transfer':
            # Wait for TR to place the expected block in Transfer Area (Rule 11.11 sensor perception)
            if self.transfer_block == step.block_kind and self.step_timer_s >= 0.65:
                self.br_carrying = self.transfer_block
                self.transfer_block = None
                done_step = True

        elif step.br_task == 'place_tower':
            if self.step_timer_s >= 0.70:
                spot_x, spot_y, spot_z = TOWER1_SPOT if step.tower_id == 1 else TOWER2_SPOT
                if step.layer_idx == 1:
                    oz = spot_z + 0.175
                    self.score_towers += 10
                elif step.layer_idx == 2:
                    oz = spot_z + 0.525
                    self.score_towers += 20
                else:
                    oz = spot_z + 0.800
                    self.score_towers += 40
                    if step.tower_id == 1:
                        self.complete_towers_exclusive += 1
                    else:
                        self.complete_towers_shared += 1
                    # Rule 3.6.1: >=2 Complete Towers standing simultaneously, >=1 in Shared Area
                    if (
                        self.complete_towers_exclusive + self.complete_towers_shared >= 2
                        and self.complete_towers_shared >= 1
                    ):
                        self.sanctuary_mandate_fulfilled = True

                self.placed_objects.append(
                    PlacedObject(self._next_obj_id, step.block_kind, spot_x, spot_y, oz)
                )
                self._next_obj_id += 1
                self.br_carrying = None
                done_step = True

        elif step.br_task == 'handover_mustika':
            # Direct TR -> BR handover in Transfer Area (Mustika NEVER touches floor!)
            tr_at_transfer = (
                self.tr_active_task == 'fetch_mustika'
                and self.tr_delivered_this_trip
                and self.tr_carrying == 'mustika'
            )
            if tr_at_transfer and self.step_timer_s >= 0.90:
                self.br_carrying = 'mustika'
                self.tr_carrying = None
                done_step = True

        elif step.br_task == 'enshrine_mustika':
            if self.step_timer_s >= 0.90:
                self.placed_objects.append(
                    PlacedObject(
                        self._next_obj_id,
                        'mustika',
                        CENTRAL_PILLAR_TOP[0],
                        CENTRAL_PILLAR_TOP[1],
                        CENTRAL_PILLAR_TOP[2],
                    )
                )
                self._next_obj_id += 1
                self.br_carrying = None
                self.mustika_enshrined = True
                self.score_mustika = 250
                done_step = True

        if done_step:
            self.step_idx += 1
            self.step_timer_s = 0.0
            self.goal_sent_for_step = False
            self.climb_sent_for_step = False
            self.last_goal_resend_s = 0.0
            actions['phase_changed'] = True

            nxt = self.current_step
            if nxt is None:
                self.completed = True
                self._plan_tr_route('park_ground')
            else:
                self._configure_tr_for_step(nxt)

        return actions


class MatchScenarioDirectorNode(Node):
    """
    ROS 2 Jazzy node executing the full ABU Robocon 2027 gameplay loop and
    publishing live 3D game object markers (`/br_mission_objects`) & scoreboard
    status (`/br_mission_status`).
    """

    def __init__(self) -> None:
        super().__init__('match_scenario_director')
        self.declare_parameter('auto_start_mission', False)
        self.declare_parameter('map_frame', 'map')

        self.logic = MatchMissionLogic()
        self._br_pose: Tuple[float, float, float] = BR_START
        self._climb_idle: bool = True
        self._last_tick = self.get_clock().now()

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.climb_pub = self.create_publisher(String, '/br_climb_command', 10)
        self.tr_override_pub = self.create_publisher(PointStamped, '/br_tr_pose_override', 10)
        self.objects_pub = self.create_publisher(MarkerArray, '/br_mission_objects', latched_qos)
        self.status_pub = self.create_publisher(String, '/br_mission_status', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.climb_status_sub = self.create_subscription(
            String, '/br_climb_status', self._climb_status_cb, 10
        )
        self.cmd_sub = self.create_subscription(
            String, '/br_mission_command', self._mission_cmd_cb, 10
        )

        self.timer = self.create_timer(0.05, self._tick)

        if bool(self.get_parameter('auto_start_mission').value):
            self.create_timer(2.5, self._auto_start_once)

        self._auto_started = False
        self.get_logger().info(
            'MatchScenarioDirectorNode ready! Press G (or click "Run Full Match Demo") '
            'to simulate the complete ABU Robocon 2027 TR + BR + 2 Towers + Mustika gameplay.'
        )

    def _auto_start_once(self) -> None:
        if not self._auto_started:
            self._auto_started = True
            self._start_mission()

    def _odom_cb(self, msg: Odometry) -> None:
        self._br_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )

    def _climb_status_cb(self, msg: String) -> None:
        self._climb_idle = 'Wheel-Lift: IDLE' in msg.data

    def _send_climb(self, cmd: str) -> None:
        m = String()
        m.data = cmd
        self.climb_pub.publish(m)

    def _send_goal(self, gx: float, gy: float, gz: float) -> None:
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = str(self.get_parameter('map_frame').value)
        g.pose.position.x = float(gx)
        g.pose.position.y = float(gy)
        g.pose.position.z = float(gz)
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    def _release_tr(self) -> None:
        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'release'
        self.tr_override_pub.publish(p)

    def _start_mission(self) -> None:
        self.logic.start()
        self._send_climb('mission_reset')
        self._br_pose = BR_START
        self.get_logger().info(
            '=== FULL ABU ROBOCON 2027 MATCH SIMULATION STARTED (TR + BR + 2 Towers + Mustika) ==='
        )

    def _mission_cmd_cb(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd in ('start', 'restart', 'toggle'):
            if cmd == 'toggle' and self.logic.active and not self.logic.completed:
                self.logic.stop()
                self._release_tr()
                self._send_climb('mode_manual')
                self.get_logger().info('Full Match Simulation STOPPED -> Returned to Manual Control.')
            else:
                self._start_mission()
        elif cmd in ('stop', 'reset'):
            self.logic.stop()
            self._release_tr()
            if cmd == 'reset':
                self.logic.reset()
            self._send_climb('mode_manual')
            self.get_logger().info('Full Match Simulation STOPPED.')

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05

        if self.logic.active:
            actions = self.logic.advance_mission(self._br_pose, self._climb_idle, dt)

            # Publish TR override pose so OUR TR moves along exact Ground + Ramp + Transfer route
            tr_msg = PointStamped()
            tr_msg.header.stamp = now.to_msg()
            tr_msg.header.frame_id = str(self.get_parameter('map_frame').value)
            tr_msg.point.x = float(self.logic.tr_pos[0])
            tr_msg.point.y = float(self.logic.tr_pos[1])
            tr_msg.point.z = float(self.logic.tr_pos[2])
            self.tr_override_pub.publish(tr_msg)

            if actions['send_climb'] is not None:
                self._send_climb(str(actions['send_climb']))
            if actions['send_goal'] is not None:
                gx, gy, gz = actions['send_goal']
                self._send_goal(gx, gy, gz)
            if actions['phase_changed']:
                step = self.logic.current_step
                if step is not None:
                    self.get_logger().info(f'[MISSION] {step.name} | {step.description}')
                else:
                    self.get_logger().info(
                        f'[MISSION COMPLETE!] Total Red Score = {self.logic.total_score} pts '
                        f'(Transfer={self.logic.score_transfer}, Towers={self.logic.score_towers}, '
                        f'Mustika={self.logic.score_mustika})'
                    )

        self._publish_objects_and_status(now.to_msg())

    def _make_object_marker(
        self, stamp, frame_id: str, mid: int, kind: str, x: float, y: float, z: float
    ) -> Marker:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = 'mission_objects'
        m.id = int(mid)
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        if kind == 'earth':
            m.type = Marker.CUBE
            m.scale.x = 0.35
            m.scale.y = 0.35
            m.scale.z = 0.35
            m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.22, 0.14, 0.98
        elif kind == 'sky':
            m.type = Marker.CUBE
            m.scale.x = 0.20
            m.scale.y = 0.20
            m.scale.z = 0.20
            m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.10, 0.15, 0.98
        else:  # mustika
            m.type = Marker.SPHERE
            m.scale.x = 0.22
            m.scale.y = 0.22
            m.scale.z = 0.22
            m.color.r, m.color.g, m.color.b, m.color.a = 1.00, 0.84, 0.05, 1.00
        return m

    def _publish_objects_and_status(self, stamp) -> None:
        frame_id = str(self.get_parameter('map_frame').value)
        m_array = MarkerArray()

        # Clear any transient slots by default
        def _del(mid: int) -> Marker:
            dm = Marker()
            dm.header.stamp = stamp
            dm.header.frame_id = frame_id
            dm.ns = 'mission_objects'
            dm.id = mid
            dm.action = Marker.DELETE
            return dm

        # Slot 1: Object carried by OUR TR
        if self.logic.tr_carrying is not None:
            tx, ty, tz = self.logic.tr_pos
            m_array.markers.append(
                self._make_object_marker(stamp, frame_id, 1, self.logic.tr_carrying, tx, ty, tz + 0.72)
            )
        else:
            m_array.markers.append(_del(1))

        # Slot 2: Block resting in Red Transfer Area loading zone
        if self.logic.transfer_block is not None:
            z_off = 0.175 if self.logic.transfer_block == 'earth' else 0.10
            m_array.markers.append(
                self._make_object_marker(
                    stamp,
                    frame_id,
                    2,
                    self.logic.transfer_block,
                    TRANSFER_DROP[0] + 0.18,
                    TRANSFER_DROP[1],
                    TRANSFER_DROP[2] + z_off,
                )
            )
        else:
            m_array.markers.append(_del(2))

        # Slot 3: Object carried by OUR BR
        if self.logic.br_carrying is not None:
            bx, by, bz = self._br_pose
            m_array.markers.append(
                self._make_object_marker(stamp, frame_id, 3, self.logic.br_carrying, bx, by, bz + 0.84)
            )
        else:
            m_array.markers.append(_del(3))

        # Placed Tower Blocks & Enshrined Mustika
        for obj in self.logic.placed_objects:
            m_array.markers.append(
                self._make_object_marker(stamp, frame_id, obj.obj_id, obj.kind, obj.x, obj.y, obj.z)
            )

        # Floating 3D Scoreboard & Phase Banner above the arena
        banner = Marker()
        banner.header.stamp = stamp
        banner.header.frame_id = frame_id
        banner.ns = 'mission_banner'
        banner.id = 999
        banner.type = Marker.TEXT_VIEW_FACING
        banner.pose.position.x = 0.0
        banner.pose.position.y = 0.0
        banner.pose.position.z = 2.55
        banner.pose.orientation.w = 1.0
        banner.scale.z = 0.28
        banner.color.r = 1.0
        banner.color.g = 0.96
        banner.color.b = 0.45
        banner.color.a = 1.0

        if not self.logic.active and not self.logic.completed:
            banner.action = Marker.DELETE
            status_text = 'Mission: IDLE (Press G or click "Run Full Match Demo" to simulate full gameplay)'
        elif self.logic.completed:
            banner.action = Marker.ADD
            status_text = (
                f'MATCH DEMO COMPLETE! | RED SCORE: {self.logic.total_score} pts '
                f'(Transfer:{self.logic.score_transfer} + Towers:{self.logic.score_towers} + '
                f'Mustika:{self.logic.score_mustika}) | Sanctuary Mandate: FULFILLED | Mustika: ENSHRINED!'
            )
            banner.text = f'RED SCORE: {self.logic.total_score} PTS | MUSTIKA ENSHRINED ON CENTRAL PILLAR!'
        else:
            step = self.logic.current_step
            phase_name = step.name if step else 'COMPLETE'
            mandate = 'UNLOCKED' if self.logic.sanctuary_mandate_fulfilled else (
                f'{self.logic.complete_towers_exclusive + self.logic.complete_towers_shared}/2 Towers'
            )
            banner.action = Marker.ADD
            banner.text = f'{phase_name}\nRED SCORE: {self.logic.total_score} pts | Mandate: {mandate}'
            status_text = (
                f'{phase_name} | Score: {self.logic.total_score} pts '
                f'(Tr:{self.logic.score_transfer} Tw:{self.logic.score_towers} Mu:{self.logic.score_mustika}) | '
                f'Mandate: {mandate}'
            )

        m_array.markers.append(banner)
        self.objects_pub.publish(m_array)

        s_msg = String()
        s_msg.data = status_text
        self.status_pub.publish(s_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MatchScenarioDirectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
