from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'br_nav_sandbox'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LEARNING_NOTES.md', 'SOFTWARE_TEAM_PLAYBOOK.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ABU Robocon 2027 BR Team',
    maintainer_email='team@aburobocon2027.local',
    description=(
        'ABU Robocon 2027 Builder Robot (BR) autonomous navigation sandbox '
        'with Fusion 360 BR v7 mesh, wheel-lifting multi-level climbing, '
        'joystick + WASD/Arrow manual control, log-odds occupancy mapping, '
        'traversability costmap, and A* pathfinding.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_mapper = br_nav_sandbox.occupancy_mapper:main',
            'traversability_costmap = br_nav_sandbox.traversability_costmap:main',
            'astar_pathfinder = br_nav_sandbox.astar_pathfinder:main',
            'l1_sim_bridge = br_nav_sandbox.l1_sim_bridge:main',
            'gz_overlay_bridge = br_nav_sandbox.gz_overlay_bridge:main',
            'match_scenario_director = br_nav_sandbox.match_scenario_director:main',
            'teleop_gui = br_nav_sandbox.teleop_gui:main',
            'teleop_keyboard = br_nav_sandbox.teleop_keyboard:main',
        ],
    },
)
