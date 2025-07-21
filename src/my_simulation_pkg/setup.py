from setuptools import setup
from glob import glob
import os

package_name = 'my_simulation_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # No Python modules yet
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models/my_map', glob('models/my_map/*.sdf')),
        ('share/' + package_name + '/models/my_map', glob('models/my_map/model.config')),
        ('share/' + package_name + '/models/my_map/meshes', glob('models/my_map/meshes/*.stl')),
        ('share/' + package_name, [os.path.join(os.path.dirname(__file__), 'package.xml')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sam',
    maintainer_email='your@email.com',
    description='Simulation package for TurtleBot3 in a custom world',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
