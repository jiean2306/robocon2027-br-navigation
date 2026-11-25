# ABU Robocon 2027 — Builder Robot (BR) Autonomous Navigation & Simulation Workspace

ROS 2 **Jazzy** + Gazebo **Harmonic** (`gz sim 8.x`) multi-package simulation, LiDAR mapping, traversability costmap, A\* pathfinding, and full Rulebook V1.1 match gameplay workspace for **ABU Robocon 2027: *"The Pursuit of Mustika Nusantara"* **.

> **Active Development Branch:** `feature/br-nav-sandbox`  
> Make sure you clone or check out **`feature/br-nav-sandbox`** (not `main`, which only holds the early initial commit).

---

## 1. Key Features

- **Exact Rulebook V1.1 & Appendix V1.1 Arena (146 3D Primitives):** Full `11 m × 11 m` Ground Area, `6 m × 6 m` Level 1 (`+0.60 m`), `3 m × 3 m` Level 2 (`+0.90 m`), `3.5 m` Ramps, Stairs, Transfer Areas, Building Spots, 40 Earth Blocks, Mustika Pillar + Mustika Ball, Central Pillar, and all **12 pixel-verified Figure 4 Sky Blocks**.
- **Fusion 360 BR v7 CAD Mesh (`0.7025 × 0.6965 × 0.6960 m`):** Cleaned and scaled in `packages/br_nav_sandbox/meshes/br_robot_v7.stl` with portable relative Gazebo URIs (`../meshes/br_robot_v7.stl`).
- **3-Floor Isolated LiDAR Occupancy Mapping & Dynamic Diff:** Bayesian log-odds + Bresenham raycasting across independent `ground`, `level_1`, and `level_2` grids (`0.05 m/cell`, `228 × 228`), plus Static-vs-Dynamic obstacle isolation (`/br_dynamic_obstacles`).
- **Vectorized EDT Traversability Costmap (`~3–6 ms`):** Enforces Rulebook boundaries, cliffs, opponent-side restrictions, and `r_inscribed = 0.35 m` + exponential decay halo (`0.80 m`), rendered as a clean non-obscuring overlay in **both RViz2 (`/br_traversability_viz`) and Gazebo Sim (`/marker_array`)**.
- **Cost-Aware A\* + Line-of-Sight Smoothing + Standoff Projection:** Auto-snaps Building Spot goals next to barriers to collision-free chassis poses, dynamically replans around moving robots (`OUR TR`, `ENEMY BR`, `ENEMY TR`), and pairs with a decoupled holonomic mecanum follower + directional forward-cone LiDAR safety shield.
- **3-Stage Wheel-Lift Step Climber:** Simulates climbing `Ground ↔ Level 1 (+0.60 m) ↔ Level 2 (+0.90 m)` and syncs pose directly into Gazebo Sim.
- **Full End-to-End Match Gameplay Demo:** Orchestrates BR + TR logistics (strictly Ground + Ramp + Transfer for TR), Tower #1 (Red L1) + Tower #2 (L1 Shared) → unlocks **Sanctuary Mandate** → **Direct Mustika Handover** → Level 2 Climb → **Central Pillar Enshrinement (`420 pts`)**.

---

## 2. Repository Map

```text
robocon2027-br-navigation/
├── README.md                              # Team onboarding, install & run guide (this file)
├── docs/
│   ├── architecture.md                    # High-level interfaces & staged integration plan
│   └── field_dimensions.md                # Reference field measurements
└── packages/
    ├── br_nav_sandbox/                    # ACTIVE full simulation & navigation stack
    │   ├── SOFTWARE_TEAM_PLAYBOOK.md      # Rulebook V1.1 audit + What Can/Should/How To guide
    │   ├── LEARNING_NOTES.md              # Design notes on wheel-lift climbing, A*, and costmaps
    │   ├── package.xml & setup.py         # ROS 2 ament_python package manifest & entry points
    │   ├── config/
    │   │   ├── br_nav_sandbox_params.yaml # All tunable LiDAR, costmap, A* & robot parameters
    │   │   └── ros_gz_bridge.yaml         # ROS 2 <-> Gazebo Harmonic topic bridge config
    │   ├── launch/
    │   │   └── br_nav_sandbox.launch.py   # Main unified launcher
    │   ├── br_nav_sandbox/
    │   │   ├── arena_geometry.py          # Single-source-of-truth 146 arena primitives & SDF builder
    │   │   ├── l1_sim_bridge.py           # Mecanum kinematics, wheel-lift climber, TF, 2D LiDAR & match robots
    │   │   ├── occupancy_mapper.py        # 3-floor Bayesian log-odds mapper + dynamic obstacle diff
    │   │   ├── traversability_costmap.py  # Vectorized SciPy EDT costmap + clean /br_traversability_viz
    │   │   ├── astar_pathfinder.py        # Cost-aware 8-connected A*, LOS smoothing & standoff projection
    │   │   ├── match_scenario_director.py # Full 31-step Rulebook V1.1 match demo director
    │   │   ├── gz_overlay_bridge.py       # Streams traversability, A*, dynamic obstacles & towers into Gazebo
    │   │   ├── teleop_gui.py              # Compact Tkinter Joystick + WASD/Arrows + Match Demo GUI
    │   │   └── teleop_keyboard.py         # Lightweight terminal keyboard teleop alternative
    │   ├── meshes/br_robot_v7.stl         # Fusion 360 BR v7 CAD mesh (meters)
    │   ├── urdf/br_robot.urdf             # Robot URDF (base_link -> lidar_link)
    │   ├── worlds/robocon_2027_l1_sandbox.sdf # Complete Gazebo Harmonic world
    │   ├── rviz/
    │   │   ├── br_unified_view.rviz       # Default single RViz window (2D Mapping + 3D Orbit presets)
    │   │   ├── br_simulation_view.rviz    # Optional separate 3D Simulation view (dual_views:=true)
    │   │   └── br_lidar_mapping_view.rviz # Optional separate 2D Mapping view (dual_views:=true)
    │   └── test/                          # Automated pytest verification suite (26 tests)
    └── br_nav_sim/                        # Preserved initial starter package (2 tests)
```

---

## 3. Prerequisites & One-Command Install (Ubuntu 24.04 / WSL2)

Install **ROS 2 Jazzy**, **Gazebo Harmonic (`ros-jazzy-ros-gz`)**, build tools, and the required Python libraries (`numpy`, `scipy`, `tkinter`, `pytest`):

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-scipy \
  python3-tk \
  python3-pytest
```

Verify Gazebo is installed:

```bash
source /opt/ros/jazzy/setup.bash
gz sim --version
```

---

## 4. Quick Start: Clone, Build & Run

### Step 1 — Clone the `feature/br-nav-sandbox` branch & build

```bash
mkdir -p ~/robocon_ws/src && cd ~/robocon_ws/src
git clone -b feature/br-nav-sandbox https://github.com/jiean2306/robocon2027-br-navigation.git
cd ~/robocon_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --symlink-install
source install/setup.bash
```

*(If `colcon` is installed via `pip --user` instead of `apt`, run `export PATH="$HOME/.local/bin:$PATH"` before `colcon build`.)*

### Step 2 — Run the Automated Test Suite (28 tests)

```bash
cd ~/robocon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
pytest -v src/robocon2027-br-navigation/packages/br_nav_sandbox/test src/robocon2027-br-navigation/packages/br_nav_sim/test
```

### Step 3 — Launch Modes (Clean Minimal Window Layouts)

> Press **`Ctrl + C`** in the launch terminal to stop a running session before launching another mode.

1. **Fast 2-Window Match Demo (Recommended for rapid algorithm/navigation testing — Unified 2D+3D RViz + Controller, no Gazebo overhead):**
   ```bash
   cd ~/robocon_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch br_nav_sandbox br_nav_sandbox.launch.py use_gazebo:=false auto_start_mission:=true
   ```

2. **Full 3D Physics Simulation (Gazebo Sim + Unified RViz Mapping View + Controller):**
   ```bash
   cd ~/robocon_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch br_nav_sandbox br_nav_sandbox.launch.py auto_start_mission:=true
   ```

3. **Gazebo-Only 2-Window Mode (Traversability, A\*, Towers & Mustika streamed directly into Gazebo Sim + Controller):**
   ```bash
   cd ~/robocon_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch br_nav_sandbox br_nav_sandbox.launch.py use_rviz:=false auto_start_mission:=true
   ```

4. **Interactive Manual Exploration Mode (press `G` anytime to start the Match Demo):**
   ```bash
   cd ~/robocon_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch br_nav_sandbox br_nav_sandbox.launch.py use_gazebo:=false
   ```

*(Useful launch arguments: `team_side:=red|blue`, `dual_views:=true|false`, `inflation_radius:=0.80`, `cost_scaling_factor:=5.0`, `use_teleop_gui:=false` with `ros2 run br_nav_sandbox teleop_keyboard`.)*

---

## 5. Controls, Camera Views & Map Color Legend

### Controller GUI & Keyboard Shortcuts (when Controller window is focused)

| Input | Action |
| :--- | :--- |
| **Virtual Joystick** / **`W A S D`** / **Arrow Keys** | Holonomic mecanum drive (`+vx`, `-vx`, `+vy`, `-vy`) |
| **`Q` / `E`** | Rotate Counter-Clockwise (`+wz`) / Clockwise (`-wz`) |
| **`Space`** | Immediate full wheel stop |
| **`L` / `K`** | Trigger 3-Stage Wheel-Lift Climb **UP** (`Ground -> L1 -> L2`) / **DOWN** |
| **`0` / `1` / `2` / `R`** | Transition to Ground Start (`0`), Level 1 (`1`), Level 2 (`2`), L1 Retry Zone (`R`) |
| **`M`** | Toggle Manual mode <-> Autonomous **2D Goal Pose** A\* follower |
| **`G`** | Start / Stop the **Full Match Gameplay Demo** |
| **`O` / `P`** | Toggle / Pause Match Robots (`OUR TR`, `ENEMY BR`, `ENEMY TR`) |

### RViz2 Saved Camera Views (Unified Window)

In the left **Views** panel of `br_unified_view.rviz`, double-click either preset:
- **`2D Top-Down Mapping (North-Up)`** — Crisp overhead view of `/br_occupancy_grid`, `/br_traversability_viz`, `/scan`, and A\* planning.
- **`3D Arena + Traversability Orbit`** — Perspective 3D view of the arena, BR v7 mesh, stacked towers, Mustika, and match robots.
- *(Tip: Use RViz's toolbar **`2D Goal Pose`** tool to click any destination for BR to plan & drive to autonomously.)*

### Occupancy & Traversability Color Legend

| Color | Layer & Meaning |
| :--- | :--- |
| **Black / White / Grey Grid** | Bayesian Occupancy Grid (`/br_occupancy_grid`: occupied / free / unknown) |
| **Crimson 1-Cell Rim** | Lethal obstacle border (`cost = 254`: walls, cliffs, opponent zone, built towers) |
| **Bright Cyan Halo** | Inscribed collision ring (`cost = 253`, `0 < d <= 0.35 m` — BR center must never enter) |
| **Red -> Orange -> Yellow** | Exponential cost decay buffer (`0.35 m < d <= 0.80 m` — A\* centers along low cost) |
| **Glowing Magenta** | Live LiDAR Dynamic Obstacles (`/br_dynamic_obstacles`: diff vs pre-made static map) |
| **Bright Green Path** | Cost-aware A\* + Line-of-Sight smoothed trajectory (`/br_planned_path`) |

---

## 6. ROS 2 Pipeline & Topic Architecture

```text
  [/scan + /odom]
        │
        ▼
  occupancy_mapper (/br_occupancy_grid, /br_dynamic_obstacles)
        │
        ▼
  traversability_costmap (/br_traversability_costmap, /br_traversability_viz, /br_footprint_polygon)
        │
        ▼
  astar_pathfinder <-- /goal_pose --> (/br_planned_path, /br_explored_nodes)
        │
        ▼
  l1_sim_bridge + match_scenario_director + gz_overlay_bridge
```

| Topic | Type | Publisher | Purpose |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/LaserScan` | `l1_sim_bridge` | Level-aware 360° 2D LiDAR scan including Earth/Sky blocks & match robots |
| `/odom` | `nav_msgs/Odometry` | `l1_sim_bridge` | Robot pose + floor elevation (`z`) used for automatic Ground/L1/L2 switching |
| `/br_occupancy_grid` | `nav_msgs/OccupancyGrid` | `occupancy_mapper` | Active-floor Bayesian log-odds occupancy map |
| `/br_dynamic_obstacles` | `visualization_msgs/MarkerArray` | `occupancy_mapper` | Moving obstacles isolated from static arena walls |
| `/br_traversability_viz` | `visualization_msgs/MarkerArray` | `traversability_costmap` | Clean non-pink cyan + red-to-yellow safety halo overlay |
| `/br_traversability_costmap` | `nav_msgs/OccupancyGrid` | `traversability_costmap` | Canonical inflated costmap (`0..254` mapped to `-1..100`) |
| `/br_planned_path` | `nav_msgs/Path` | `astar_pathfinder` | LOS-smoothed cost-aware A\* path (`[]` triggers safety stop) |
| `/br_mission_objects` | `visualization_msgs/MarkerArray` | `match_scenario_director` | Live carried/stacked Earth Blocks, Sky Blocks, Mustika & scoreboard banner |

---

## 7. Where to Continue Next

Read [`packages/br_nav_sandbox/SOFTWARE_TEAM_PLAYBOOK.md`](packages/br_nav_sandbox/SOFTWARE_TEAM_PLAYBOOK.md) for the complete Rulebook V1.1 breakdown and engineering roadmap. Priority next tasks for the team:

1. **Dynamic Contingency Spots:** If `ENEMY BR` blocks the North L1 Shared Spot, fall back to South L1 Shared or an L2 corner Building Spot, and stamp finished towers as permanent lethal obstacles (Rule 6.6).
2. **Rule 11.11 Transfer Area Sensor Trigger:** Detect *empty vs Earth Block vs Sky Block vs TR-holding-Mustika* via a LiDAR/ToF/camera ROI box inside the Red Transfer Area (zero TR↔BR radio communication).
3. **Real Hardware Integration:** Swap `l1_sim_bridge` for your STM32/Teensy mecanum + wheel-lift driver (`/cmd_vel` ↔ `/odom`), Class 1 2D LiDAR (`/scan`), and static-map scan-matching/AMCL localization.

---

## Legacy Starter Package (`packages/br_nav_sim`)

The original minimal starter package (`field_map_node`, `astar_planner`, `mecanum_kinematics`, schematic SDF) is preserved untouched inside `packages/br_nav_sim/`. All new development targets `packages/br_nav_sandbox/`.
