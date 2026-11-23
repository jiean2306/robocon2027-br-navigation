# ABU Robocon 2027 Builder Robot (BR) — Autonomous Navigation Mathematics & Learning Notes

This guide derives step by step every mathematical model and algorithm implemented in the `br_nav_sandbox` ROS 2 Jazzy package for the **ABU Robocon 2027 ("The Pursuit of Mustika Nusantara") Builder Robot (BR)** operating on **Level 1 (L1)**.

---

## 1. Coordinate Frames, Arena Geometry & TF2 Tree

### 1.1 Arena Coordinate Convention
Following the ABU Robocon 2027 Rulebook V1.1 and Appendix V1.1 plan views (Figures 3.1, 3.3, 3.9):
* The `map` frame origin $(0, 0, 0)$ sits at the geometric center of the $11000 \times 11000\text{ mm}$ ($11.0 \times 11.0\text{ m}$) Ground Area floor:
  * $+X$ points **East** (Red Team side is $x < 0$; Blue Team side is $x > 0$).
  * $+Y$ points **North** (Start/Retry Zones & $1200 \times 1200\text{ mm}$ Sky Block Ground Shared Area are at $y < 0$; Storage Areas & Mustika Pillar are at $y > 0$).
  * $+Z$ points **Up** (Level 1 top surface is at $z = +0.60\text{ m}$; Level 2 top surface is at $z = +0.90\text{ m}$).

### 1.2 Rigid-Body Homogeneous Transformations (`map -> odom -> base_link -> lidar_link`)
In planar 2D navigation at fixed elevation $z_{\text{L1}} = 0.60\text{ m}$, a rigid-body pose $(x, y, \theta) \in SE(2)$ is represented by the $3 \times 3$ homogeneous transformation matrix:

$$
T = \begin{bmatrix}
\cos\theta & -\sin\theta & x \\
\sin\theta & \cos\theta & y \\
0 & 0 & 1
\end{bmatrix}
$$

The ROS 2 TF2 tree chains three transforms:

$$
{}^{m}T_{\ell} = {}^{m}T_{o} \cdot {}^{o}T_{b} \cdot {}^{b}T_{\ell}
$$

1. **`map -> odom` (${}^{m}T_{o}$)**: Corrects long-term odometry drift using global localization (identity $I_3$ when initialized without drift).
2. **`odom -> base_link` (${}^{o}T_{b}$)**: Continuously integrated from wheel encoder / mecanum odometry (`/odom`) at 20–50 Hz.
3. **`base_link -> lidar_link` (${}^{b}T_{\ell}$)**: Fixed rigid mounting offset $(\Delta x_\ell, \Delta y_\ell, \Delta z_\ell) = (0.0, 0.0, +0.22\text{ m})$ above `base_link`.

Given a 2D LiDAR range measurement $r_i$ at beam angle $\alpha_i = \alpha_{\min} + i \cdot \Delta\alpha$ in the `lidar_link` frame, its Cartesian hit coordinate in the `map` frame is:

$$
\begin{bmatrix} x_{\text{hit}} \\ y_{\text{hit}} \end{bmatrix}
=
\begin{bmatrix} x_s \\ y_s \end{bmatrix}
+
r_i \begin{bmatrix} \cos(\theta_s + \alpha_i) \\ \sin(\theta_s + \alpha_i) \end{bmatrix}
$$

where $(x_s, y_s, \theta_s)$ is the pose of `lidar_link` in `map`.

---

## 2. Four-Wheel Mecanum Drive Kinematics

### 2.1 Geometry & Roller Layout
The Builder Robot uses four mecanum wheels with roller axes angled at $\pm 45^\circ$ to the wheel hub axis (standard X-configuration):
* Wheel radius: $r = 0.075\text{ m}$
* Half-wheelbase (longitudinal distance from `base_link` center to wheel axle): $a = L_{\text{wb}} / 2 = 0.25\text{ m}$
* Half-track width (lateral distance from `base_link` center to wheel center): $b = W_{\text{tr}} / 2 = 0.28\text{ m}$
* Geometric lever arm sum: $k = a + b$

Using the ROS 2 body-frame convention ($v_x$ forward, $v_y$ left, $\omega_z$ counter-clockwise yaw rate) and wheel ordering $(\omega_{\text{FL}}, \omega_{\text{FR}}, \omega_{\text{RL}}, \omega_{\text{RR}})$:

### 2.2 Inverse Kinematics (Body Twist $\to$ Wheel Angular Velocities)

$$
\begin{bmatrix}
\omega_{\text{FL}} \\
\omega_{\text{FR}} \\
\omega_{\text{RL}} \\
\omega_{\text{RR}}
\end{bmatrix}
=
\frac{1}{r}
\begin{bmatrix}
1 & -1 & -(a + b) \\
1 & +1 & +(a + b) \\
1 & +1 & -(a + b) \\
1 & -1 & +(a + b)
\end{bmatrix}
\begin{bmatrix}
v_x \\
v_y \\
\omega_z
\end{bmatrix}
$$

* **Pure forward motion** ($v_x > 0, v_y = 0, \omega_z = 0$): all four wheels spin forward at $\omega = v_x / r$.
* **Pure left strafe** ($v_x = 0, v_y > 0, \omega_z = 0$): $\omega_{\text{FL}} = \omega_{\text{RR}} = -v_y / r$ and $\omega_{\text{FR}} = \omega_{\text{RL}} = +v_y / r$; longitudinal roller force components cancel while lateral components sum to $+v_y$.
* **Pure CCW rotation in place** ($v_x = 0, v_y = 0, \omega_z > 0$): left wheels spin backward ($\omega_{\text{FL}} = \omega_{\text{RL}} = -k\omega_z / r$) and right wheels spin forward ($\omega_{\text{FR}} = \omega_{\text{RR}} = +k\omega_z / r$).

### 2.3 Forward Kinematics (Wheel Angular Velocities $\to$ Body Twist)
Taking the Moore-Penrose pseudoinverse $J^+ = (J^T J)^{-1} J^T$ of the $4 \times 3$ inverse kinematics matrix yields:

$$
\begin{bmatrix}
v_x \\
v_y \\
\omega_z
\end{bmatrix}
=
\frac{r}{4}
\begin{bmatrix}
1 & 1 & 1 & 1 \\
-1 & 1 & 1 & -1 \\
-\frac{1}{a+b} & \frac{1}{a+b} & -\frac{1}{a+b} & \frac{1}{a+b}
\end{bmatrix}
\begin{bmatrix}
\omega_{\text{FL}} \\
\omega_{\text{FR}} \\
\omega_{\text{RL}} \\
\omega_{\text{RR}}
\end{bmatrix}
$$

In world/odometry coordinates, integrating over timestep $\Delta t$ updates the robot pose:

$$
\begin{aligned}
x_{t+1} &= x_t + (v_x \cos\theta_t - v_y \sin\theta_t)\,\Delta t \\
y_{t+1} &= y_t + (v_x \sin\theta_t + v_y \cos\theta_t)\,\Delta t \\
\theta_{t+1} &= \theta_t + \omega_z\,\Delta t
\end{aligned}
$$

---

## 3. Bayesian Log-Odds LiDAR Occupancy Grid Mapping

### 3.1 Binary Bayes Filter in Log-Odds Form
We partition the 2D arena plane into a regular grid of independent binary random variables $m_i \in \{0, 1\}$ with resolution $\Delta_{\text{grid}} = 0.05\text{ m/cell}$, where $m_i = 1$ denotes an occupied cell and $m_i = 0$ denotes a free cell.

Given sensor measurements $z_{1:t}$ and known robot poses $x_{1:t}$, Bayes' rule for cell $m_i$ states:

$$
p(m_i = 1 \mid z_{1:t}, x_{1:t})
=
\frac{p(z_t \mid m_i = 1, x_t)\,p(m_i = 1 \mid z_{1:t-1}, x_{1:t-1})}{p(z_t \mid z_{1:t-1}, x_{1:t})}
$$

Applying Bayes' rule to the inverse sensor model $p(z_t \mid m_i = 1, x_t) = \frac{p(m_i = 1 \mid z_t, x_t)\,p(z_t \mid x_t)}{p(m_i = 1)}$ and dividing $p(m_i = 1 \mid z_{1:t}, x_{1:t})$ by $p(m_i = 0 \mid z_{1:t}, x_{1:t}) = 1 - p(m_i = 1 \mid z_{1:t}, x_{1:t})$ cancels the intractable marginal observation probabilities:

$$
\frac{p(m_i = 1 \mid z_{1:t}, x_{1:t})}{1 - p(m_i = 1 \mid z_{1:t}, x_{1:t})}
=
\underbrace{\frac{p(m_i = 1 \mid z_{1:t-1}, x_{1:t-1})}{1 - p(m_i = 1 \mid z_{1:t-1}, x_{1:t-1})}}_{\text{prior odds at } t-1}
\cdot
\underbrace{\frac{p(m_i = 1 \mid z_t, x_t)}{1 - p(m_i = 1 \mid z_t, x_t)}}_{\text{inverse sensor model odds}}
\cdot
\underbrace{\frac{1 - p(m_i = 1)}{p(m_i = 1)}}_{\text{initial prior odds}^{-1}}
$$

Defining the **log-odds ratio** $L(m_i) = \ln\left(\frac{p(m_i = 1)}{1 - p(m_i = 1)}\right)$ turns multiplication into simple addition:

$$
L(m_i \mid z_{1:t}, x_{1:t}) = L(m_i \mid z_{1:t-1}, x_{1:t-1}) + l_{\text{inv}}(m_i \mid z_t, x_t) - l_0
$$

With a uniform initial prior $p_0 = 0.5$, the prior log-odds is $l_0 = \ln(0.5 / 0.5) = 0.0$. At any time, the posterior probability is recovered via the logistic sigmoid function:

$$
p(m_i = 1 \mid z_{1:t}, x_{1:t}) = \frac{1}{1 + \exp\big(-L(m_i \mid z_{1:t}, x_{1:t})\big)}
$$

### 3.2 Inverse Sensor Model & Clamping
For each LiDAR beam traced from sensor cell $(c_{sx}, c_{sy})$ to endpoint cell $(c_{ex}, c_{ey})$:
* **Free cells along the beam**: $p_{\text{free}} = 0.30 \implies l_{\text{free}} = \ln(0.30 / 0.70) \approx -0.8473$.
* **Hit endpoint cell** (when $r_{\min} \le r < r_{\max}$): $p_{\text{occ}} = 0.70 \implies l_{\text{occ}} = \ln(0.70 / 0.30) \approx +0.8473$.
* **Saturation clamping**: each update is clamped to $[L_{\min}, L_{\max}] = [-4.0, +4.0]$ ($p \in [0.018, 0.982]$) so the map never becomes infinitely confident and can adapt if dynamic game objects (Earth/Sky blocks or the opponent BR in Shared Areas) move.

### 3.3 Bresenham's 2D Integer Raycasting Algorithm
To trace a ray from integer cell $(x_0, y_0)$ to $(x_1, y_1)$ without floating-point division or missed diagonal pixels:
1. Compute $\Delta x = |x_1 - x_0|$, $\Delta y = |y_1 - y_0|$, step signs $s_x, s_y \in \{-1, +1\}$, and initial error $e = \Delta x - \Delta y$.
2. At each cell $(c_x, c_y)$, double the error $e_2 = 2e$:
   * If $e_2 > -\Delta y$, decrement $e \leftarrow e - \Delta y$ and step $c_x \leftarrow c_x + s_x$.
   * If $e_2 < \Delta x$, increment $e \leftarrow e + \Delta x$ and step $c_y \leftarrow c_y + s_y$.
3. Repeat until $(c_x, c_y) = (x_1, y_1)$.

### 3.4 Quantization to `nav_msgs/OccupancyGrid` (`/br_occupancy_grid`)
* **Occupied (`100`)**: $p(L) \ge p_{\text{occ\_thresh}}$ ($0.65 \implies L \ge +0.619$)
* **Free (`0`)**: $p(L) \le p_{\text{free\_thresh}}$ ($0.35 \implies L \le -0.619$)
* **Unknown (`-1`)**: $-0.619 < L < +0.619$ (unobserved or ambiguous cells)

---

## 4. Traversability & Footprint Costmap Inflation

### 4.1 Builder Robot Footprint Radii (Minkowski Configuration Space)
Planning for a rigid $L \times W = 0.70\text{ m} \times 0.70\text{ m}$ square Builder Robot can be reduced to planning for a **point robot**$(x_r, y_r)$ by inflating obstacles by the robot's footprint radii:

1. **Inscribed Radius ($r_{\text{inscribed}}$)** — radius of the largest circle completely contained inside the $0.70 \times 0.70\text{ m}$ chassis:

$$
r_{\text{inscribed}} = \frac{\min(L, W)}{2} = \frac{0.70\text{ m}}{2} = 0.35\text{ m}
$$

   *If the robot center $(x_r, y_r)$ is at distance $d \le r_{\text{inscribed}}$ from an obstacle cell, the robot collides with the obstacle **for every possible heading angle $\theta \in [-\pi, \pi)$**.*

2. **Circumscribed Radius ($r_{\text{circumscribed}}$)** — distance from the robot center to its four outer corners:

$$
r_{\text{circumscribed}} = \frac{\sqrt{L^2 + W^2}}{2} = \frac{\sqrt{0.70^2 + 0.70^2}}{2} = 0.35\sqrt{2} \approx 0.495\text{ m} \approx 0.50\text{ m}
$$

   *If $r_{\text{inscribed}} < d \le r_{\text{circumscribed}}$, collision depends on the robot's yaw angle $\theta$ (corners may clip the obstacle). If $d > r_{\text{circumscribed}}$, the robot is guaranteed collision-free at **any** yaw orientation.*

3. **Inflation Radius ($r_{\text{inflation}}$)** — outer buffer radius ($r_{\text{inflation}} = 0.80\text{ m} \ge r_{\text{circumscribed}}$) over which repulsive cost decays smoothly to zero.

### 4.2 Exponential Cost Decay Curve ($0\text{–}254$ Canonical Scale)
Given the Euclidean distance $d = \sqrt{(\Delta c_x)^2 + (\Delta c_y)^2} \cdot \Delta_{\text{grid}}$ from a cell to the nearest lethal obstacle/barrier/boundary cell, the canonical $0\text{–}254$ cost $C_{254}(d)$ is:

$$
C_{254}(d) = \begin{cases}
254 & \text{if } d = 0 \quad (\text{Lethal Obstacle / Barrier / Cliff / Restricted Zone}) \\
253 & \text{if } 0 < d \le r_{\text{inscribed}} \quad (\text{Inscribed Footprint Collision}) \\
\left\lfloor 252 \cdot \exp\!\Big(-k_{\text{scale}} \cdot (d - r_{\text{inscribed}})\Big) \right\rfloor & \text{if } r_{\text{inscribed}} < d \le r_{\text{inflation}} \\
0 & \text{if } d > r_{\text{inflation}} \quad (\text{Zero-Cost Free Space})
\end{cases}
$$

where $k_{\text{scale}} = \text{cost\_scaling\_factor}$ (default $5.0\text{ m}^{-1}$):
* Larger $k_{\text{scale}}$ makes the exponential penalty drop off steeply outside $r_{\text{inscribed}}$ (allowing tighter cornering near walls).
* Smaller $k_{\text{scale}}$ spreads a broader high-cost cushion across L1 corridors (pushing paths closer to the corridor centerline).

### 4.3 Mapping to `nav_msgs/OccupancyGrid` (`int8` $[-1..100]$)
Because ROS 2 `nav_msgs/msg/OccupancyGrid.data` uses signed 8-bit integers (`int8`, $[-128, 127]$) and RViz2's `Costmap` color palette expects values in $[-1..100]$, we map $C_{254}$ linearly while preserving exact inscribed/lethal semantics:
* $C_{254} = 254 \text{ (Lethal)} \longmapsto 100$ (rendered purple/yellow lethal in RViz2)
* $C_{254} = 253 \text{ (Inscribed)} \longmapsto 99$ (rendered bright cyan inscribed contour in RViz2)
* $C_{254} \in [1..252] \longmapsto \text{round}\!\left(\frac{C_{254} \cdot 98}{252}\right) \in [1..98]$ (rendered red-to-blue gradient in RViz2)
* $C_{254} = 0 \longmapsto 0$ (free space)
* $C_{254} = 255 \text{ (Unknown)} \longmapsto -1$

### 4.4 ABU Robocon 2027 Level 1 Restricted & Non-Traversable Zones
In addition to LiDAR-detected obstacles, `is_point_restricted_on_l1(x, y, team_side)` marks the competition-specific Level 1 boundaries as lethal ($C_{254} = 254$) so they are automatically inflated by $r_{\text{inscribed}} = 0.35\text{ m}$ (satisfying Rule 4.3.3 vertical projection constraints):
1. **L1 Outer Perimeter & Drop-Offs**: Only the $5940 \times 5940\text{ mm}$ L1 inner floor ($|x| \le 2.97\text{ m}, |y| \le 2.97\text{ m}$) plus the team's $1000 \times 1000\text{ mm}$ Transfer Area ($y \in [-1.80, -0.80]\text{ m}$, $x \in [-4.00, -2.97]\text{ m}$ for Red or $[+2.97, +4.00]\text{ m}$ for Blue) are valid L1 surfaces.
2. **Central Level 2 Platform Step**: $|x| \le 1.50\text{ m}, |y| \le 1.50\text{ m}$ ($+300\text{ mm}$ vertical step wall).
3. **L1 Center Divider Fences**: $|x| \le 0.015\text{ m}, |y| \in [1.50, 2.22]\text{ m}$ (Appendix Figures 3.3, 3.9, 3.10d).
4. **Opponent Team Exclusive L1 Area (Rule 6.1.2)**: For Red Team, $x > 0$ is restricted everywhere except inside the North and South **L1 Shared Areas** ($x \in [-0.50, +0.50]\text{ m}, |y| \in [2.22, 2.97]\text{ m}$).

---

## 5. Cost-Aware A* Pathfinding (`/br_planned_path`)

### 5.1 Graph Search Objective
On the 8-connected traversability costmap grid, A* expands nodes from a priority queue ordered by:

$$
f(n) = g(n) + h(n)
$$

* **$g(n)$**: accumulated cost-to-come from the start cell $n_{\text{start}}$ to cell $n$.
* **$h(n)$**: admissible heuristic estimate of the remaining cost-to-go from $n$ to $n_{\text{goal}}$.

### 5.2 Traversability-Penalized Edge Cost
Transitioning from cell $u = (x_u, y_u)$ to an adjacent 8-connected neighbor $v = (x_v, y_v)$ is permitted only if:
1. $C_{254}(v) < 253$ (blocks both lethal $254$ and inscribed $253$ cells), and
2. For diagonal steps ($|x_v - x_u| = 1$ and $|y_v - y_u| = 1$), both cardinal neighbors $(x_v, y_u)$ and $(x_u, y_v)$ also satisfy $C_{254} < 253$ (preventing diagonal corner cutting).

The step cost $\Delta g(u, v)$ combines geometric distance with a traversability cost penalty:

$$
\Delta g(u, v) = d_{\text{step}}(u, v) \cdot \left(1.0 + w_{\text{cost}} \cdot \frac{C_{254}(v)}{252.0}\right)
$$

where:
* $d_{\text{step}}(u, v) = 1.0$ for cardinal moves and $\sqrt{2} \approx 1.4142$ for diagonal moves,
* $w_{\text{cost}} \ge 0$ (`cost_penalty_weight`, default $4.0$) controls the trade-off between **shortest geometric path** ($w_{\text{cost}} = 0$) and **safest high-clearance path** ($w_{\text{cost}} > 0$).

### 5.3 Octile Distance Heuristic & Proof of Admissibility
In an 8-connected grid where cardinal steps cost at least $1$ and diagonal steps cost at least $\sqrt{2}$, the minimum geometric distance between $n = (x_n, y_n)$ and $n_{\text{goal}} = (x_g, y_g)$ with $\Delta x = |x_n - x_g|$ and $\Delta y = |y_n - y_g|$ is achieved by taking $\min(\Delta x, \Delta y)$ diagonal steps followed by $|\Delta x - \Delta y| = \max(\Delta x, \Delta y) - \min(\Delta x, \Delta y)$ cardinal steps:

$$
h_{\text{octile}}(n, n_{\text{goal}})
=
\sqrt{2}\,\min(\Delta x, \Delta y) + \big(\max(\Delta x, \Delta y) - \min(\Delta x, \Delta y)\big)
=
\max(\Delta x, \Delta y) + (\sqrt{2} - 1)\min(\Delta x, \Delta y)
$$

**Admissibility & Consistency Proof:**
Because $C_{254}(v) \ge 0$ and $w_{\text{cost}} \ge 0$, every edge satisfies:

$$
\Delta g(u, v) = d_{\text{step}}(u, v)\left(1 + w_{\text{cost}}\frac{C_{254}(v)}{252}\right) \ge d_{\text{step}}(u, v)
$$

By the triangle inequality on the octile metric, for any edge $(u, v)$:

$$
h_{\text{octile}}(u, n_{\text{goal}}) \le d_{\text{step}}(u, v) + h_{\text{octile}}(v, n_{\text{goal}}) \le \Delta g(u, v) + h_{\text{octile}}(v, n_{\text{goal}})
$$

Thus $h_{\text{octile}}$ is both **consistent (monotonic)** and **admissible** ($h(n) \le h^*(n)$), guaranteeing that A* expands each closed-set cell at most once and returns the **globally optimal** cost-weighted path.
