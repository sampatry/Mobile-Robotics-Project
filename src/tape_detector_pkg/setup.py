from setuptools import setup

package_name = 'tape_detector_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sampatry8@gmail.com',
    description='Tape detector node',
    license='MIT',
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'tape_detector = scripts.detector_node:main',
        ],
    },
)
