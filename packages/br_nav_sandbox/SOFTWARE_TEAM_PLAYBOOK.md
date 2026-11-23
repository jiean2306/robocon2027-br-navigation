# ABU Robocon 2027 — BR Software Team Playbook
**Owner:** Navigation, LiDAR, Occupancy Mapping, Traversability & Path Planning Sub-Team  
**Rulebook Baseline:** ABU Robocon 2027 Official Rulebook V1.1 ("The Pursuit of Mustika Nusantara")

---

## 1. Rulebook V1.1 Alignment Check (Critical Constraints for Software!)

| Gameplay Assumption | Rulebook V1.1 Verdict | Exact Rule & Software Impact |
| :--- | :--- | :--- |
| **TR stays on Ground Floor and transports Earth & Sky Blocks to BR** | **Aligned + 1 Crucial Catch** | **Rules 3.2.1, 3.4.5, 4.3.1, 6.1.1**: TR operates on Ground Area **and enters the `1000 × 1000 mm` Transfer Area** (via the 3.5 m Ramp). *While touching the Ground Area, TR cannot place objects on the Transfer Area* (3.4.5), and TR must **never** cross past Transfer Area onto L1/L2 (6.1.1 = Forced Retry). TR may carry up to **3 blocks** at once (3.4.3). |
| **BR starts on Ground, climbs to L1 first, waits at Transfer Area, builds on Building Spots** | **100% Aligned** | **Rules 3.2.2, 3.3.1, 4.3.2**: BR starts in Ground Start/Retry Zone, climbs to L1 via Stairs/Ramp, operates on Transfer Area, L1, and L2. BR is **100% autonomous** (Rule 11.3) and may carry up to **2 blocks** at once (3.4.4). |
| **Stack 2 Earth Blocks → then place 1 Sky Block on top** | **100% Aligned** | **Rules 3.5.2, 6.3, 8.2**: Strictly `Earth (bottom, +10/+20) → Earth (middle, +20/+40) → Sky (top, +40/+80, team color UP)`. Out-of-sequence placement or dragging blocks = **Failure / Forced Retry**. |
| **After 2 Complete Towers, TR transports Mustika to BR** | **Almost Exact — 1 Crucial Catch!** | **Sanctuary Mandate (Rules 3.6.1 & 4.5.1)**: You need **≥ 2 Complete Towers standing simultaneously**, AND **at least 1 of them MUST be in a Shared Area (`L1 Shared Area` or `L2 Shared Area`)!** Once fulfilled, the mandate is **permanent** even if the opponent later flips a Sky Block (3.6.2). |
| **TR passes Mustika to BR** | **100% Aligned + Handover Rule** | **Rules 3.4.5, 3.6.3, 6.2**: Unlike blocks, the **Mustika MUST be handed DIRECTLY from TR to BR in the Transfer Area and NEVER touch the floor** (touching floor = dropped/invalid!). Neither robot may hold any block while carrying the Mustika (3.4.6). BR then climbs to **Level 2 (+0.90 m)** and enshrines it on the **Central Pillar** for **+250 pts** (match still continues until 3:00, Rule 9.1.2). |
| **How does BR know TR delivered a block or Mustika?** | **CRITICAL SOFTWARE RULE (11.11)!** | **Rule 11.11 strictly bans ANY wireless/encoded communication between TR and BR!** BR must **autonomously detect** blocks in the Transfer Area and TR's presence using **onboard sensors** (LiDAR ROI scan, ToF array, or depth camera). |

---

## 2. What You **CAN** Do (Capabilities of Your LiDAR + Occupancy + Traversability + Pathing Stack)

As the software lead owning perception-to-planning on BR, your stack unlocks six competition-winning powers:

1. **Millimeter-Grade Localization on a Known Field (`map → odom → base_link`)**:
   - Because the arena dimensions (`11 × 11 m`, L1 `6 × 6 m`, L2 `3 × 3 m`, Central Pillar) are fixed by the Rulebook, you can match live 2D LiDAR scans against the **pre-built known CAD/Rulebook wall map** (AMCL / scan-to-map ICP) fused with wheel encoders + IMU/gyro.
   - Result: `< 1.5 cm` pose drift across the entire 3-minute match — critical for landing `350 mm` Earth Blocks cleanly inside `500 mm` green Building Spots and docking at the Transfer Area!
2. **Static-vs-Dynamic Obstacle Differentiation (Our Static Map + Live LiDAR Diff)**:
   - Subtracting the known arena walls from the live `/scan` immediately isolates **things that were NOT in the CAD map**:
     - The **Enemy BR** entering L1 Shared Area or Level 2 Shared Area
     - **Blocks newly placed by OUR TR** inside the Transfer Area *(legal, zero-comms Rule 11.11 trigger!)*
     - **Already-built Towers** (so BR never bumps a tower and triggers a Rule 6.6 Forced Retry!)
3. **Hard Rulebook Safety & Cliff Protection via Traversability Costmap**:
   - Bake **every rulebook forbidden zone** directly into the costmap as `COST_LETHAL (254)`:
     - Level 1 outer drop-offs & `30 × 50 mm` perimeter barriers
     - Opponent exclusive half of L1 (preventing Rule 6.1.2 Area Violation!)
     - Level 2 `+300 mm` step walls (while on L1) and Level 2 outer cliffs + Central Pillar (while on L2)
   - Inflate every obstacle by BR's exact **inscribed radius (`r_inscribed = 0.35 m`)** + **exponential decay halo (`0.80 m`)**, mathematically guaranteeing the `700 × 700 mm` chassis never clips a wall, tower, or opponent!
4. **Dynamic Replanning & Non-Interference (Rule 6.5)**:
   - When `ENEMY BR` enters an L1/L2 Shared Area, your costmap immediately inflates a cyan no-go bubble around `ENEMY BR`, and **cost-aware A\*** routes cleanly around it (or pauses cleanly with a LiDAR safety shield if the corridor is temporarily blocked) — avoiding **Rule 6.5 Blocking/Interference Forced Retries**!
5. **Multi-Floor Map Isolation (`ground` / `level_1` / `level_2`)**:
   - Maintain **3 separate floor maps** switched by wheel-lift climber state + IMU/elevation estimate so beams on Level 2 never erase Level 1 barriers or Ground blocks.
6. **Smart Standoff Goal Projection**:
   - Because Building Spots sit right next to barriers (`(-2.72, -2.72)` is only `0.25 m` from the L1 wall, less than `r_inscribed = 0.35 m`), your planner automatically projects any Building Spot target to the **nearest safe chassis standoff pose** matching your mechanical arm/gripper reach!

---

## 3. What You **SHOULD** Do (Priority Roadmap to Win Matches)

Don't fall into the trap of running heavy general-purpose SLAM (like GMapping/SLAM Toolbox from scratch) during a 3-minute match! Follow this competition-tested priority order:

### Priority 1 (Must-Have Before Any Match) — **Hybrid Pre-Made Map + Live Dynamic Layer**
- **DO NOT** rely on exploring the arena from scratch with SLAM during a match — you already know the exact arena geometry from the Rulebook!
- **DO** preload the exact static floor maps (`ground`, `level_1`, `level_2`) + fuse live LiDAR strictly for:
  1. **Localization correction** (fixing mecanum wheel slip on plywood/stairs!)
  2. **Dynamic obstacle overlay** (`ENEMY BR`, placed towers, Transfer Area blocks)

### Priority 2 — **Level-Specific Tight Localization + Mechanical Alignment**
- Mecanum wheels slip heavily on painted plywood and after wheel-lift climbing!
- Fuse **Dead-Reckoning Odometry (4-wheel encoders or 2-3 dead-wheel tracking pods)** + **BNO085/IMU yaw** + **2D LiDAR wall-matching (AMCL / scan-matching)** at 20–50 Hz.
- For final precision (`±5 mm`) when stacking blocks on a Building Spot or docking at Transfer Area, combine your global LiDAR pose with a short-range sensor (line sensor / ToF / downward camera) on the final `15 cm` approach.

### Priority 3 — **Transfer Area Autonomous Trigger (Rule 11.11 Compliant)**
- Since TR and BR **cannot talk over Wi-Fi/Bluetooth** (Rule 11.11), define a **LiDAR Region of Interest (ROI)** box covering the Red Transfer Area `x ∈ [-4.00, -2.97], y ∈ [-1.80, -0.80]`:
  - Cluster LiDAR points inside that ROI:
    - Width ~`350 mm` → **Earth Block** ready!
    - Width ~`200 mm` → **Sky Block** ready!
    - Large cluster (`~600 mm`) while Sanctuary Mandate is fulfilled → **OUR TR holding the Mustika** → enter Direct Handover alignment!
  - *(Backup: add a small depth/RGB camera on BR facing the Transfer Area to distinguish Earth vs Sky vs Mustika color reliably!)*

### Priority 4 — **Fast Cost-Aware A\* + Line-of-Sight Smoothing + Holonomic Trajectory Tracker**
- Keep grid resolution at **`0.05 m/cell`** (`228 × 228` cells = `< 5 ms` EDT costmap + `< 8 ms` A\* replan on a Jetson Orin Nano / Raspberry Pi 5!).
- Always run **Theta\*-style Line-of-Sight (LOS) path smoothing** so BR moves in crisp straight holonomic diagonals instead of jagged 45° stair-steps.
- Decouple **translation `(vx, vy)`** from **heading `wz`** so BR can glide sideways while already turning its gripper to face the next Building Spot!

### Priority 5 — **Tower Protection & Retry Recovery (Rules 5.3 & 6.6)**
- Every time BR places a tower, **mark that Building Spot as a permanent lethal obstacle** in your live occupancy grid so BR never swings through its own finished tower!
- Pre-program an instant **L1 Retry Zone (`(-1.85, +2.62, z=0.60)`) initial-pose preset** (Rule 5.3.1): if you call a Retry after entering L1, pressing one button resets localization to the L1 Retry Zone in `< 0.5 s` instead of climbing from Ground again!

---

## 4. How You **CAN** Do It (Step-by-Step Engineering Implementation)

### A. Recommended Hardware & Compute Stack on Real BR
- **Compute**: NVIDIA Jetson Orin Nano / Raspberry Pi 5 running **Ubuntu 24.04 + ROS 2 Jazzy** (exact same code as this workspace!).
- **Low-Level MCU**: STM32 / Teensy running 1 kHz wheel PID + mecanum kinematics + wheel-lift state machine + hardwired E-Stop (Rule 12.2) + Freeze/Resume receiver (Rule 11.4), connected via micro-ROS or fast UART/CAN.
- **2D LiDAR**: Class 1 IEC 60825-1 compliant (Rule 11.16.2!) — e.g. **RPLIDAR S2/A3**, **Hokuyo UST-10LX**, or **LD19/YDLIDAR TG30** mounted at `z ≈ 0.20–0.25 m` above `base_link` (high enough to clear the `50 mm` L1 perimeter barriers or angled/dual-height so it sees both barriers AND Transfer Area blocks!).
  - *Tip*: Notice L1 perimeter barriers are `50 mm` tall and Center Dividers are `50 mm` tall, while Earth Blocks are `350 mm` tall and L2 walls are `300 mm` tall! Mounting a low LiDAR (`z ≈ 0.04 m`) sees barriers/dividers; mounting a mid LiDAR (`z ≈ 0.22 m`) sees L2 walls, Earth Blocks, Central Pillar, and Enemy BR cleanly over the `50 mm` rim. Using **pre-made static barriers + mid-height LiDAR** is the cleanest solution!

### B. Software Pipeline (Already Built in `br_nav_sandbox` — Ready to Port to Real Robot!)

```text
  [Real 2D LiDAR /scan] + [Wheel Odom + IMU /odom]
                 │
                 ▼
  1. Localization (Static Map Scan-Match / AMCL)  ──►  map -> odom -> base_link
                 │
                 ▼
  2. Multi-Level Occupancy Mapper (occupancy_mapper.py)
     • Preloaded exact Ground / L1 / L2 static walls
     • Bayesian log-odds + Bresenham raycasting
     • Static-vs-Dynamic Diff layer -> isolates Enemy BR & Transfer Area blocks
                 │
                 ▼
  3. Vectorized EDT Traversability Costmap (traversability_costmap.py)
     • Exact Euclidean Distance Transform (~3-6 ms/frame)
     • Lethal (254) = walls, cliffs, opponent zone, built towers, dynamic obstacles
     • Inscribed (253) = 0.35 m hard collision ring (BR 700x700 mm radius)
     • Exponential Decay (1..252) = 0.35 m .. 0.80 m centering gradient
                 │
                 ▼
  4. Cost-Aware A* + LOS Smoothing (astar_pathfinder.py)
     • f(n) = g(n) + h(n) + 0.02 * cell_cost (prefers corridor centers!)
     • Smart Standoff Projection for Building Spots & Transfer Area
     • Dynamic replan every ~0.45 s if Enemy BR blocks path
                 │
                 ▼
  5. Holonomic Mecanum Controller + Safety Shield (l1_sim_bridge.py / real MCU)
     • Decoupled (vx, vy, wz) tracking + <0.42 m LiDAR emergency brake
     • 3-Stage Wheel-Lift Climber state machine (Ground <-> L1 <-> L2)
                 │
                 ▼
  6. High-Level Match Behavior Tree / State Machine (match_scenario_director.py)
     • Climb L1 -> Wait/Detect Block -> Build Tower 1 (Red L1) ->
       Build Tower 2 (L1 Shared) -> Sanctuary Mandate Unlocked! ->
       Direct Mustika Handover -> Climb L2 -> Enshrine on Central Pillar!
```

### C. Weekly Development Plan for Your Software Team
1. **Week 1–2 (Simulation — Right Here in `br_nav_sandbox`)**:
   - Tune `inflation_radius`, `cost_scaling_factor`, and A\* `cost_weight` while running `match_scenario_director` with `ENEMY BR` moving across Shared Areas.
   - Test failure cases: what happens if `ENEMY BR` stands directly on the North L1 Shared Spot? Add a fallback goal selector (`North Shared Spot` vs `South Shared Spot` vs `Level 2 Corner Spot`)!
2. **Week 3–4 (Odometry + LiDAR Localization on Real Chassis)**:
   - Swap `l1_sim_bridge` with your real MCU driver (`/cmd_vel` → STM32, STM32 encoders+IMU → `/odom`) and real LiDAR driver (`/scan`).
   - Run static-map AMCL / scan-matcher and verify `< 2 cm` repeatability across 20 runs on plywood.
3. **Week 5–6 (Transfer Area Perception + Building Spot Final Docking)**:
   - Implement the Rule 11.11 Transfer Area LiDAR/Camera ROI detector (Earth vs Sky vs TR holding Mustika).
   - Add closed-loop gripper alignment for the final `15 cm` into Building Spots and the Central Pillar.
4. **Week 7+ (Full 3-Minute Match Stress Testing & Retry Drills)**:
   - Practice full 3-minute runs with a real teammate driving TR manually, intentionally triggering Retries (Rule 5.3) and Freeze/Resume (Rule 11.4), and verifying zero Area/Tower Violations!
