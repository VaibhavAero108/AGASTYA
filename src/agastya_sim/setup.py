# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
import os
from glob import glob

from setuptools import setup

package_name = 'agastya_sim'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='GPL-3.0-or-later',
    maintainer='Vaibhav (VaibhavAero108)',
    entry_points={
        'console_scripts': [
            'control_pad = agastya_sim.control_pad:main',
            'world_markers = agastya_sim.world_markers:main',
            'link_sim = agastya_sim.link_sim:main',
            'acceptance_test = agastya_sim.acceptance_test:main',
        ],
    },
)
