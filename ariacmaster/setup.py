from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'ariacmaster'

# Collect every file inside config/, keeping their internal structure
config_files = [
    (
        os.path.join('share', package_name, os.path.dirname(f)),
        [f],
    )
    for f in glob('config/**', recursive=True)
    if os.path.isfile(f)
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
    ] + config_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Justin Albrecht',
    maintainer_email='justin.albrecht@nist.gov',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'physical_inspection = ariacmaster.task1.physical_inspection:main',
            'conveyor_pickup = ariacmaster.task1.conveyor_pickup:main',
            'agv_placement = ariacmaster.task1.agv_placement:main',
            'move_agv = ariacmaster.task1.move_agv:main',
            'voltage_inspection = ariacmaster.task1.voltage_inspection:main',
            'move_robots = ariacmaster.examples.move_robots:main',
            'dummy_inspection = ariacmaster.examples.dummy_inspection:main',
            'pick_from_tester = ariacmaster.examples.pick_from_tester:main',
            'submit_kit = ariacmaster.examples.submit_kit:main',
            'tool_change = ariacmaster.examples.tool_change:main'
        ],
    },
)
