from setuptools import setup
import os
from glob import glob

package_name = 'sim_joy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'inputs', 'pynput'],
    zip_safe=True,
    maintainer='Soom',
    maintainer_email='soom@example.com',
    description='Joystick teleoperation and GUI for Tiangong robot simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_joy = sim_joy.gui_monitor:main_joy',
            'teleop_gui = sim_joy.gui_monitor:main',
        ],
    },
)
