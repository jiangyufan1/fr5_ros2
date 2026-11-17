from glob import glob
from setuptools import find_packages, setup
setup(
    name='fr5_python', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fr5_python']),
        ('share/fr5_python', ['package.xml']),
        ('share/fr5_python/launch', glob('launch/*.launch.py')),
        ('share/fr5_python/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='FR5 developer', maintainer_email='developer@example.com',
    description='Read-only ROS2 Python state subscriber for FAIRINO FR5',
    license='Apache-2.0',
    entry_points={'console_scripts': ['state_echo = fr5_python.state_echo:main']},
)
