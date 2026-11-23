#!/usr/bin/env python3
"""
Unified ROS 2 Jazzy Launch File for ABU Robocon 2027 Builder Robot (BR v7 Fusion 360)
Multi-Level Simulation, LiDAR Mapping, A* Navigation & Joystick/WASD Controller.

Brings up:
  * View 1 (3D Simulation View):
      - Gazebo Harmonic (`gz sim`) with the exact ABU Robocon 2027 arena & BR v7 mesh
      - 3D Simulation RViz2 Window (`br_simulation_view.rviz`) showing the full 3D arena
        and the Fusion 360 BR v7 robot starting at the Ground Start/Retry Zone (-4.35, -5.15, z=0.0m)
  * View 2 (2D LiDAR Mapping & Navigation View):
      - 2D Top-Down Orthographic RViz2 Window (`br_lidar_mapping_view.rviz`) showing the exact
        black-and-white LiDAR Occupancy Grid Map (`/br_occupancy_grid`), live 2D LiDAR scan (`/scan`),
        BR footprint (`/br_footprint_polygon`), A* planned path (`/br_planned_path`),
        and toggleable Traversability Costmap (`/br_traversability_costmap`)
  * Interactive Controller Window (`teleop_gui`):
      - Virtual 2D Holonomic Mecanum Joystick + WASD + Arrow Keys + 3-Stage Wheel-Lifting
        Climb Controls (`L` / `K` / `0` / `1` / `2` / `R` / `M`)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('br_nav_sandbox')
    default_params_file = os.path.join(pkg_share, 'config', 'br_nav_sandbox_params.yaml')
    default_bridge_file = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    default_world_file = os.path.join(pkg_share, 'worlds', 'robocon_2027_l1_sandbox.sdf')
    unified_rviz_file = os.path.join(pkg_share, 'rviz', 'br_unified_view.rviz')
    sim_rviz_file = os.path.join(pkg_share, 'rviz', 'br_simulation_view.rviz')
    lidar_rviz_file = os.path.join(pkg_share, 'rviz', 'br_lidar_mapping_view.rviz')
    default_urdf_file = os.path.join(pkg_share, 'urdf', 'br_robot.urdf')

    with open(default_urdf_file, 'r', encoding='utf-8') as urdf_f:
        robot_description_content = urdf_f.read()

    inflation_radius_arg = DeclareLaunchArgument(
        'inflation_radius',
        default_value='0.80',
        description='Costmap exponential inflation radius in meters (>= inscribed_radius)',
    )
    cost_scaling_factor_arg = DeclareLaunchArgument(
        'cost_scaling_factor',
        default_value='5.0',
        description='Exponential cost decay rate k_scale in m^-1',
    )
    grid_resolution_arg = DeclareLaunchArgument(
        'grid_resolution',
        default_value='0.05',
        description='Occupancy grid and costmap cell resolution in meters/cell',
    )
    inscribed_radius_arg = DeclareLaunchArgument(
        'inscribed_radius',
        default_value='0.35',
        description='Inscribed radius of the 0.70m x 0.70m Builder Robot in meters',
    )
    team_side_arg = DeclareLaunchArgument(
        'team_side',
        default_value='red',
        description='Team side for L1 restricted area enforcement (red or blue)',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the br_nav_sandbox parameter YAML file',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_file,
        description='Full path to the Gazebo Harmonic SDF world file',
    )
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Whether to launch Gazebo Harmonic 3D physics simulator',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo Harmonic in headless server mode (-s -r)',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2 visualization windows',
    )
    dual_views_arg = DeclareLaunchArgument(
        'dual_views',
        default_value='false',
        description='Open two separate RViz2 windows instead of the single unified Mapping + 3D + Traversability view',
    )
    use_teleop_gui_arg = DeclareLaunchArgument(
        'use_teleop_gui',
        default_value='true',
        description='Open the Virtual Joystick + WASD/Arrows + Wheel-Lifting Climb Controller GUI',
    )
    auto_start_mission_arg = DeclareLaunchArgument(
        'auto_start_mission',
        default_value='false',
        description='Automatically launch the full ABU Robocon 2027 TR+BR+2-Towers+Mustika gameplay scenario',
    )

    inflation_radius = LaunchConfiguration('inflation_radius')
    cost_scaling_factor = LaunchConfiguration('cost_scaling_factor')
    grid_resolution = LaunchConfiguration('grid_resolution')
    inscribed_radius = LaunchConfiguration('inscribed_radius')
    team_side = LaunchConfiguration('team_side')
    params_file = LaunchConfiguration('params_file')
    world_file = LaunchConfiguration('world')
    use_gazebo = LaunchConfiguration('use_gazebo')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    dual_views = LaunchConfiguration('dual_views')
    use_teleop_gui = LaunchConfiguration('use_teleop_gui')
    auto_start_mission = LaunchConfiguration('auto_start_mission')

    shared_tunable_overrides = {
        'inflation_radius': inflation_radius,
        'cost_scaling_factor': cost_scaling_factor,
        'grid_resolution': grid_resolution,
        'inscribed_radius': inscribed_radius,
    }

    gz_gui_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", use_gazebo, "' == 'true' and '", headless, "' != 'true'"])
        ),
    )

    gz_headless_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', world_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", use_gazebo, "' == 'true' and '", headless, "' == 'true'"])
        ),
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': default_bridge_file}],
        condition=IfCondition(use_gazebo),
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    l1_sim_bridge_node = Node(
        package='br_nav_sandbox',
        executable='l1_sim_bridge',
        name='l1_sim_bridge',
        output='screen',
        parameters=[params_file],
    )

    occupancy_mapper_node = Node(
        package='br_nav_sandbox',
        executable='occupancy_mapper',
        name='occupancy_mapper',
        output='screen',
        parameters=[params_file, shared_tunable_overrides],
    )

    traversability_costmap_node = Node(
        package='br_nav_sandbox',
        executable='traversability_costmap',
        name='traversability_costmap',
        output='screen',
        parameters=[
            params_file,
            shared_tunable_overrides,
            {'team_side': team_side},
        ],
    )

    astar_pathfinder_node = Node(
        package='br_nav_sandbox',
        executable='astar_pathfinder',
        name='astar_pathfinder',
        output='screen',
        parameters=[params_file, shared_tunable_overrides],
    )

    match_scenario_director_node = Node(
        package='br_nav_sandbox',
        executable='match_scenario_director',
        name='match_scenario_director',
        output='screen',
        parameters=[{'auto_start_mission': auto_start_mission}],
    )

    gz_overlay_bridge_node = Node(
        package='br_nav_sandbox',
        executable='gz_overlay_bridge',
        name='gz_overlay_bridge',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", use_gazebo, "' == 'true' and '", headless, "' != 'true'"])
        ),
    )

    rviz_unified_view_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_unified_view',
        output='screen',
        arguments=['-d', unified_rviz_file],
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "' == 'true' and '", dual_views, "' != 'true'"])
        ),
    )

    rviz_simulation_view_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_simulation_view',
        output='screen',
        arguments=['-d', sim_rviz_file],
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "' == 'true' and '", dual_views, "' == 'true'"])
        ),
    )

    rviz_lidar_mapping_view_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_lidar_mapping_view',
        output='screen',
        arguments=['-d', lidar_rviz_file],
        condition=IfCondition(
            PythonExpression(["'", use_rviz, "' == 'true' and '", dual_views, "' == 'true'"])
        ),
    )

    teleop_gui_node = Node(
        package='br_nav_sandbox',
        executable='teleop_gui',
        name='br_teleop_gui',
        output='screen',
        condition=IfCondition(use_teleop_gui),
    )

    return LaunchDescription(
        [
            inflation_radius_arg,
            cost_scaling_factor_arg,
            grid_resolution_arg,
            inscribed_radius_arg,
            team_side_arg,
            params_file_arg,
            world_arg,
            use_gazebo_arg,
            headless_arg,
            use_rviz_arg,
            dual_views_arg,
            use_teleop_gui_arg,
            auto_start_mission_arg,
            gz_gui_cmd,
            gz_headless_cmd,
            ros_gz_bridge_node,
            robot_state_pub_node,
            l1_sim_bridge_node,
            occupancy_mapper_node,
            traversability_costmap_node,
            astar_pathfinder_node,
            match_scenario_director_node,
            gz_overlay_bridge_node,
            rviz_unified_view_node,
            rviz_simulation_view_node,
            rviz_lidar_mapping_view_node,
            teleop_gui_node,
        ]
    )
