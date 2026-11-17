# ABU Robocon 2027 — Builder Robot (BR) Navigation Simulation Starter

This is the first implementation baseline for a mecanum-drive BR using ROS 2 Jazzy,
Gazebo Harmonic and Python (`rclpy`). It is deliberately parameterised because the
team's mechanical/electrical design is not final.

## What is included

- `br_nav_sim/field_map_node.py`: publishes a static `nav_msgs/OccupancyGrid` for
  an L1 navigation test map (6 m × 6 m, 5 cm cells).
- `br_nav_sim/astar_planner.py`: subscribes to `/map` and `/goal_pose`, plans with
  8-connected A*, publishes `/planned_path`.
- `br_nav_sim/mecanum_kinematics.py`: pure Python mecanum inverse/forward
  kinematics functions, with tests. These are independent of any specific motor.
- `config/robot.yaml`: provisional geometry and speed parameters. Replace these
  with measured values once the chassis is final.
- `worlds/robocon_2027_schematic.sdf`: schematic Gazebo environment with the
  11 m × 11 m ground field, raised L1/L2 platforms and an illustrative ramp.
- `docs/architecture.md`: interfaces and staged integration plan.

## Important accuracy note

The Rulebook and Appendix specify the main field dimensions, heights and several
area dimensions. This first SDF is a **navigation-development schematic**, not a
certified exact replica of every barrier, opening, object, color, or team-side
coordinate. Do not use it to validate competition legality or final robot
clearances. The L1 occupancy grid is intentionally a clean baseline; detailed
obstacles and restricted areas will be added from the Appendix drawing after the
team chooses its coordinate convention and team side.

## Target platform

Develop and run Gazebo on an Ubuntu desktop/laptop first. Raspberry Pi is the
intended onboard target, not the preferred machine for full Gazebo GUI simulation.
Once the robot computer model and Ubuntu/ROS image are confirmed, deploy the ROS
navigation nodes to the Pi and keep the simulator on the development PC.

Recommended ROS pairing: ROS 2 Jazzy + Gazebo Harmonic. The official
`gz_ros2_control_demos` package provides a mecanum example to use as the initial
actuation reference. The robot model in this starter is not yet connected to
that demo's controller; do that in the next integration stage after confirming
the wheel layout and the exact controller parameters for the installed package.

## 1. Install ROS 2 Jazzy and Gazebo Harmonic

Follow the official ROS 2 Jazzy and Gazebo installation instructions for Ubuntu
24.04. Then install:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-ros2-control-demos \
  python3-colcon-common-extensions
```

Source ROS in each terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

Check Gazebo:

```bash
gz sim --version
```

## 2. Build this ROS package

Unzip the archive, then:

```bash
mkdir -p ~/robocon_ws/src
cp -r robocon2027_br_navigation ~/robocon_ws/src/br_nav_sim
cd ~/robocon_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3. Run the L1 map and A* planner

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robocon_ws/install/setup.bash
ros2 run br_nav_sim field_map
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robocon_ws/install/setup.bash
ros2 run br_nav_sim astar_planner
```

Terminal 3 (RViz2):

```bash
source /opt/ros/jazzy/setup.bash
source ~/robocon_ws/install/setup.bash
rviz2
```

In RViz2:
1. Set Fixed Frame to `map`.
2. Add a Map display, topic `/map`.
3. Add a Path display, topic `/planned_path`.
4. Add a Pose display or use the **2D Goal Pose** tool on `/goal_pose`.
5. A* publishes a path if both the start and goal are in free cells.

For a quick command-line goal, use a PoseStamped publisher (example: goal x=1.5 m,
y=1.0 m):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 1.5, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

The planner's initial start pose is a parameter (`start_x`, `start_y`). This is
temporary. Later it will subscribe to the robot's estimated pose from TF/odometry.

## 4. Open the schematic Gazebo world

```bash
gz sim worlds/robocon_2027_schematic.sdf
```

This opens the field geometry for visual inspection. It does not yet spawn a
fully actuated BR. That is an intentional separation: first verify map/planner
logic, then integrate the mecanum robot and physics controller.

## 5. Validate mecanum mathematics

```bash
python3 -m pytest test
```

The convention used is:
- `x`: forward
- `y`: left
- `z`: counter-clockwise yaw
- wheel order: front-left, front-right, rear-left, rear-right

For wheel radius `r`, half-length `a` and half-width `b`, let `k = a + b`.
The inverse kinematics are:

```text
w_FL = (vx - vy - k*wz) / r
w_FR = (vx + vy + k*wz) / r
w_RL = (vx + vy - k*wz) / r
w_RR = (vx - vy + k*wz) / r
```

**Check wheel roller orientation and motor polarity with the mechanical team.**
Different mecanum roller handedness / motor mounting may require sign changes.

## 6. What we implement next

1. Confirm chassis dimensions, wheel radius, wheel centre spacing, wheel roller
   orientation, motor/encoder limits, sensor list and mounting positions.
2. Connect the official mecanum ros2_control controller to the BR Gazebo model.
3. Publish simulated `/odom`, `/tf`, `/scan` and `/imu/data`.
4. Replace the temporary fixed start pose with odometry/localisation.
5. Add map obstacles and robot-footprint inflation.
6. Add a floor/level manager for Ground ↔ L1 ramp and L1 ↔ L2 transitions.
7. Add Nav2/path following and replanning only after the custom map/planner pipeline
   is understood and tested.
