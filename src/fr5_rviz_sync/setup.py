from glob import glob
from setuptools import find_packages, setup

setup(
    name='fr5_rviz_sync', version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fr5_rviz_sync']),
        ('share/fr5_rviz_sync', ['package.xml', 'README.md']),
        ('share/fr5_rviz_sync/launch', glob('launch/*.launch.py')),
        ('share/fr5_rviz_sync/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='dj306', maintainer_email='dj306@example.com',
    description='Read-only FR5 feedback to RViz', license='BSD-3-Clause',
    entry_points={'console_scripts': ['joint_state_bridge = fr5_rviz_sync.bridge:main']},
)
