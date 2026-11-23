from setuptools import setup
from glob import glob
import os

package_name = 'br_nav_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robocon BR Team',
    maintainer_email='team@example.com',
    description='ABU Robocon 2027 BR navigation simulation starter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'field_map = br_nav_sim.field_map_node:main',
            'astar_planner = br_nav_sim.astar_planner:main',
        ],
    },
)
