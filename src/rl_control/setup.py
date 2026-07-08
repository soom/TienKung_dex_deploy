import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rl_control'


def _collect_share_files(src_dir, dst_subdir):
    out = []
    for root, _dirs, files in os.walk(src_dir):
        if '__pycache__' in root:
            continue
        rel = os.path.relpath(root, src_dir)
        target = os.path.join('share', package_name, dst_subdir) if rel == '.' \
            else os.path.join('share', package_name, dst_subdir, rel)
        kept = [os.path.join(root, f) for f in files
                if not f.endswith('.pyc') and not f.endswith('.py')]
        if kept:
            out.append((target, kept))
    return out


data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'),
        glob('config/*.yaml')),
]
data_files += _collect_share_files('policy', 'policy')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    py_modules=['rl_control_node', 'rl_control_node_sim'],
    package_data={
        'policy.walk_amp': ['config/*.yaml', 'model/*.onnx'],
        'policy.zero': ['config/*.yaml'],
        'policy.stop': ['config/*.yaml'],
        'policy.beyond_mimic': ['config/*.yaml', 'model/*.onnx'],
        'policy.beyondzero': ['config/*.yaml'],
        'policy.niukua': ['config/*.yaml', 'model/*.onnx'],
    },
    include_package_data=True,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Soom',
    maintainer_email='soom@example.com',
    description='xMIGCS RL control node with FSM-based policy switching.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'rl_control_node = rl_control_node:main',
            'rl_control_node_sim = rl_control_node_sim:main',
        ],
    },
)
