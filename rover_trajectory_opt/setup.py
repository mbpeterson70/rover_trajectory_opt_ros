from setuptools import setup
import os
from glob import glob

package_name = 'rover_trajectory_opt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'cfg'), glob('cfg/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'tomma',
        'robotdatapy'
    ],
    zip_safe=True,
    maintainer='masonbp',
    maintainer_email='mbpeterson70@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc.py = rover_trajectory_opt.mpc:main',
            'simple_trajectory_publisher = rover_trajectory_opt.simple_traj_publisher:main',
            # 'trajectory_generator_node.py = rover_trajectory_opt.trajectory_generator_node:main',
        ],
    },
)
