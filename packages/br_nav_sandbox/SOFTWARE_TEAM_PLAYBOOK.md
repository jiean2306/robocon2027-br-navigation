# ABU Robocon 2027 — BR Navigation & Gameplay Software Guide

**Scope:** LiDAR Perception, Multi-Level Occupancy Mapping, Traversability Costmap, A\* Path Planning & Match Flow  
**Rulebook Reference:** ABU Robocon 2027 Official Rulebook V1.1 ("The Pursuit of Mustika Nusantara")

---

## 1. Competition Rules & Software Constraints (Rulebook V1.1)

The software architecture in `br_nav_sandbox` enforces the following competition rules directly at the perception, costmap, planning, and state-machine layers:

| Domain | Rulebook Requirement | Software Implementation & Constraint |
| :--- | :--- | :--- |
| **TR Operating Area** | Rules 3.2.1, 4.3.1, 6.1.1 | Transporter Robot (TR) operates only in the Ground Area and enters the `1000 × 1000 mm` Transfer Area via the Ramp. While touching the Ground Area, TR cannot place objects on the Transfer Area (Rule 3.4.5). Any entry into Level 1 beyond the Transfer Area or Level 2 incurs a Forced Retry. |
| **BR Operating Area** | Rules 3.2.2, 4.3.2, 11.3 | Builder Robot (BR) starts in a Ground Start/Retry Zone, climbs to Level 1 via Stairs/Ramp, operates across the Transfer Area, Level 1, and Level 2, and must run fully autonomously without manual control. |
| **Carry Limits** | Rules 3.4.3, 3.4.4, 3.4.6 | TR may carry at most 3 blocks; BR may carry at most 2 blocks. A robot carrying the Mustika may not carry any block. |
| **Tower Sequence & Scoring** | Rules 3.5.2, 6.3, 8.2 | Every Complete Tower must be stacked in order: `Earth Block (bottom: +10 L1 / +20 L2)` → `Earth Block (middle: +20 L1 / +40 L2)` → `Sky Block (top: +40 L1 / +80 L2, team color facing up)`. |
| **Sanctuary Mandate** | Rules 3.6.1, 3.6.2, 4.5.1 | Retrieving the Mustika requires at least two Complete Towers standing simultaneously, with at least one located in a Shared Area (`L1 Shared Area` or `L2 Shared Area`). Once achieved, the mandate remains permanent. |
| **Mustika Handover & Enshrinement** | Rules 3.4.5, 3.6.3, 3.6.8, 6.2 | Unlike blocks, the Mustika must be handed directly from TR to BR inside the Transfer Area without touching the floor. BR then climbs to Level 2 (`+0.90 m`) and enshrines it on the Central Pillar (`+250 pts`). Match continues for the full 3 minutes (Rule 9.1.2). |
| **Zero Robot-to-Robot Comms** | Rule 11.11 | Communication between TR and BR is strictly prohibited during a match. BR must infer object deliveries and field state exclusively from onboard sensors (LiDAR, ToF, or camera). |
| **Freeze / Resume & E-Stop** | Rules 11.4, 12.2 | BR must support a referee-authorized remote Freeze/Resume actuator pause and a physical hardwired emergency stop button. |

---

## 2. System Architecture & Data Flow

```text
  [2D LiDAR /scan] + [Wheel Encoders + IMU /odom]
                 │
                 ▼
  1. Localization (Static Map Scan-Match / AMCL)  ──►  map -> odom -> base_link
                 │
                 ▼
  2. Multi-Level Occupancy Mapper (occupancy_mapper.py)
     • Preloaded exact Ground / L1 / L2 static wall grids
     • Bayesian log-odds updates + Bresenham raycasting
     • Static-vs-Dynamic Diff layer isolating moving robots & delivered blocks
                 │
                 ▼
  3. Vectorized EDT Traversability Costmap (traversability_costmap.py)
     • Exact Euclidean Distance Transform (~3-6 ms per 228x228 update)
     • Lethal (254) = walls, cliffs, opponent zone, built towers, dynamic obstacles
     • Inscribed (253) = 0.35 m collision ring (half-width of 700 x 700 mm chassis)
     • Exponential Decay (1..252) = 0.35 m .. 0.80 m clearance gradient
                 │
                 ▼
  4. Cost-Aware A* + LOS Smoothing (astar_pathfinder.py)
     • f(n) = g(n) + h(n) + 0.02 * cell_cost (prefers corridor centerlines)
     • Standoff goal projection for Building Spots & Transfer Area edges
     • Periodic replanning (~0.45 s) for dynamic obstacle avoidance
                 │
                 ▼
  5. Holonomic Mecanum Controller + Wheel-Lift Climber (l1_sim_bridge.py / real MCU)
     • Decoupled (vx, vy, wz) tracking + directional forward-cone LiDAR safety brake
     • 3-Stage Wheel-Lift Climb state machine (Ground <-> L1 <-> L2)
                 │
                 ▼
  6. Match Mission Director (match_scenario_director.py)
     • Climb L1 -> Transfer Staging -> Tower 1 (Red L1) -> Tower 2 (L1 Shared)
     • Sanctuary Mandate Check -> Direct Mustika Handover -> Climb L2 -> Enshrine
```

---

## 3. Core Modules & Design Rationale

### 3.1 Hybrid Pre-Made Map + Live Dynamic Layer
Full online SLAM from an unknown map is unnecessary for a standardized field and introduces loop-closure risk within a 3-minute match. Instead, `occupancy_mapper.py` pre-rasterizes the fixed Rulebook wall geometry onto three `228 × 228` grids (`0.05 m/cell`, covering `11.4 × 11.4 m`) and uses live `/scan` data strictly for:
1. Scan-to-map localization correction (compensating for mecanum wheel slip on painted plywood and stairs).
2. Static-vs-dynamic differencing (`/br_dynamic_obstacles`) with raytrace clearing to detect opposing robots, finished towers, and newly delivered blocks in the Transfer Area.

### 3.2 Three-Floor Map Isolation (`ground` / `level_1` / `level_2`)
Operating a single 2D grid across multiple elevations causes Level 2 LiDAR beams (`z = 0.90 m`) to overwrite Level 1 perimeter barriers (`z = 0.60 m`) or Ground Earth/Sky blocks (`z = 0.00 m`) as free space. Both `occupancy_mapper.py` and `traversability_costmap.py` maintain separate grids per level and switch instantaneously based on `/odom` elevation `z` (and climber state on hardware).

### 3.3 Vectorized EDT Traversability Costmap
`traversability_costmap.py` computes an exact Euclidean Distance Transform (`scipy.ndimage.distance_transform_edt`) from all lethal cells (static arena walls, drop-offs, opponent exclusive half, built towers, and live dynamic detections):
- `COST_LETHAL (254)` (`d = 0`): Impassable boundary or obstacle.
- `COST_INSCRIBED (253)` (`0 < d <= 0.35 m`): Hard chassis collision buffer matching BR's `700 × 700 mm` footprint radius.
- `Exponential Decay (1..252)` (`0.35 m < d <= 0.80 m`): Smooth penalty curve `floor(252 * exp(-k * (d - r_inscribed)))` that guides paths away from walls without blocking narrow doorways.

For simultaneous visualization in RViz2 and Gazebo Sim, `/br_traversability_viz` renders only the active-floor safety rings and a 1-cell obstacle rim so the underlying black-and-white occupancy grid remains clearly visible.

### 3.4 Cost-Aware A\*, Standoff Projection & Directional Safety Shield
- **Path Smoothing:** `astar_pathfinder.py` applies Theta\*-style Bresenham Line-of-Sight pruning on top of 8-connected A\* waypoints so the holonomic base travels along smooth diagonals rather than axis-aligned zigzags.
- **Smart Standoff Projection:** Corner Building Spots (such as `(-2.72, -2.72)` on L1) sit `0.25 m` from perimeter barriers, closer than `r_inscribed = 0.35 m`. When a requested goal falls inside an inscribed or lethal zone, the planner automatically projects it to the nearest traversable standoff pose facing the target.
- **Non-Interference & Safety Brake:** `l1_sim_bridge.py` decouples translation `(vx, vy)` from heading control `wz` and monitors a directional forward LiDAR cone (`±41°` around the velocity vector, threshold `< 0.38 m`) to prevent collisions and Rule 6.5 Shared Area interference.

### 3.5 Rule 11.11 Transfer Area Perception & Recovery
- **Autonomous Block Trigger (Hardware Integration Target):** Define a LiDAR/ToF/camera Region of Interest (ROI) over the team Transfer Area (`x ∈ [-4.00, -2.97], y ∈ [-1.80, -0.80]` for Red) to classify empty floor vs. Earth Block (`350 mm`) vs. Sky Block (`200 mm`) vs. TR holding the Mustika without any wireless link.
- **Tower Protection (Rule 6.6):** Once a tower is placed on a Building Spot, that spot should be stamped into the live occupancy grid as `COST_LETHAL (254)` so subsequent paths never brush against it.
- **Retry Localization Preset (Rule 5.3.1):** If a Retry is called after BR has entered Level 1, an L1 Retry Zone pose preset (`(-1.85, +2.62, z = 0.60 m)` for Red) allows instant map re-alignment without re-climbing from Ground.

---

## 4. Hardware Target & Deployment Notes

| Component | Target Specification & Integration Notes |
| :--- | :--- |
| **Onboard Compute** | NVIDIA Jetson Orin Nano or Raspberry Pi 5 running **Ubuntu 24.04 + ROS 2 Jazzy** (runs the identical `occupancy_mapper`, `traversability_costmap`, `astar_pathfinder`, and mission logic nodes). |
| **Low-Level MCU** | STM32 / Teensy (micro-ROS or UART/CAN) handling 1 kHz mecanum wheel PID, encoder + IMU (e.g., BNO085) odometry publishing on `/odom`, wheel-lift actuator sequencing, hardwired E-Stop (Rule 12.2), and remote Freeze/Resume (Rule 11.4). |
| **2D LiDAR Sensor** | IEC 60825-1 Class 1 laser scanner (Rule 11.16.2), such as RPLIDAR S2/A3, Hokuyo UST-10LX, or YDLIDAR TG30. Note the field barrier heights: L1 perimeter barriers and Center Dividers are `50 mm` tall, L2 walls are `300 mm`, and Earth Blocks are `350 mm`. A scanner mounted at `z ≈ 0.20–0.25 m` reliably detects L2 walls, pillars, Earth Blocks, and opposing robots while static preloaded barriers protect the `50 mm` rims. |
| **Fine Alignment** | Global LiDAR A\* navigation brings BR to the standoff pose (`±2–3 cm`). Pair with short-range ToF, line sensors, or a downward camera for the final `15 cm` gripper alignment on Building Spots and the Central Pillar. |

---

## 5. Development Roadmap

1. **Simulation Validation (`br_nav_sandbox`):** Tune `inflation_radius`, `cost_scaling_factor`, and A\* `cost_weight` against moving match robots (`ENEMY BR`, `ENEMY TR`); add dynamic contingency Building Spot selection (`South L1 Shared` / `L2 Corner`) when `North L1 Shared` is occupied.
2. **Real Chassis Bring-Up:** Replace `l1_sim_bridge` with the physical MCU driver (`/cmd_vel` → motors, encoders + IMU → `/odom`), physical Class 1 LiDAR driver (`/scan`), and static-map scan matching / AMCL (`map -> odom`).
3. **Transfer Area & Gripper Docking:** Implement the Rule 11.11 Transfer Area ROI detector (Earth vs. Sky vs. Mustika handover) and closed-loop final `15 cm` alignment.
4. **Full Match & Retry Drills:** Validate full 3-minute continuous operation alongside a manually driven TR, verifying zero Area/Tower violations and `< 0.5 s` L1 Retry recovery.
