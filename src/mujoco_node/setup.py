import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mujoco_node'


def _collect_share_files(src_dir, dst_subdir):
    out = []
    for root, _dirs, files in os.walk(src_dir):
        rel = os.path.relpath(root, src_dir)
        target = os.path.join('share', package_name, dst_subdir) if rel == '.' \
            else os.path.join('share', package_name, dst_subdir, rel)
        kept = [os.path.join(root, f) for f in files]
        if kept:
            out.append((target, kept))
    return out


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
]
data_files += _collect_share_files('resources', 'resources')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*', 'scripts', 'scripts.*']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Soom',
    maintainer_email='soom@example.com',
    description='MuJoCo simulator node for the EVT2 humanoid platform.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mujoco_node = mujoco_node.simulator_view_asyn:main',
        ],
    },
)
