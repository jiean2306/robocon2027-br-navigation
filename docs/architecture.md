# BR Navigation Architecture (initial design)

```text
Mission / waypoint request
          |
          v
   Level & mode manager  <---- field transition state (later)
          |
          v
 Occupancy / traversability map
          |
          v
       A* planner
          |
          v
 Path follower / velocity controller (later)
          |
          v
   /cmd_vel (body frame)
          |
          v
 Mecanum drive controller
          |
          v
 Four wheel motors / encoders
```

## ROS interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | L1 static occupancy map |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Requested navigation goal |
| `/planned_path` | `nav_msgs/Path` | A* output |
| `/cmd_vel` | `geometry_msgs/Twist` (controller-dependent) | Body-frame velocity command |
| `/odom` | `nav_msgs/Odometry` | Simulated/estimated odometry (next integration) |
| `/scan` | `sensor_msgs/LaserScan` | Simulated LiDAR (next integration) |
| `/imu/data` | `sensor_msgs/Imu` | Simulated IMU (next integration) |

## Map coordinate convention

Use `map` as a local coordinate frame for one surface at a time. For the initial L1
map, `(0,0)` is the lower-left floor corner in the team's chosen field coordinate
system. Do not hard-code Red/Blue mirroring into the planner. Instead, define one
team-side transform when the exact coordinate convention is agreed.

A single 2D occupancy grid cannot safely represent the Ground, L1 and L2 floors
because the same x-y coordinate can refer to different heights. Use separate
level maps and a level-transition manager. The manager should only permit a
transition when the robot is at a valid ramp/step/L2-climb transition and the
transition state is confirmed.

## Provisional chassis parameters

The values in `config/robot.yaml` are placeholders for software bring-up only.
They are not inferred from the competition rules. In particular, the maximum
starting footprint in the rules is not a recommended final chassis dimension.
