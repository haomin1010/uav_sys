from setuptools import setup
import os
from glob import glob

package_name = 'uav_decision_arbiter'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV Team',
    maintainer_email='your_email@example.com',
    description='无人机多源决策仲裁与同步系统',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arbiter = uav_decision_arbiter.arbiter_node:main',
            'synchronizer = uav_decision_arbiter.synchronizer_node:main',
            'rl_adapter = uav_decision_arbiter.rl_adapter:main',
            'airsim_adapter = uav_decision_arbiter.airsim_adapter:main',
            'px4_adapter = uav_decision_arbiter.px4_adapter:main',
            'formation_sync = uav_decision_arbiter.formation_sync_node:main',
        ],
    },
)

