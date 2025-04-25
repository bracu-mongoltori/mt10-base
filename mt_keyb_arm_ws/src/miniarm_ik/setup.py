from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'miniarm_ik'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        # (os.path.join('share', package_name), glob('config/*config.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duronto',
    maintainer_email='nafis.noor.202012@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyb_ik = miniarm_ik.keyb_ik:main',
            'user_input_target = miniarm_ik.user_input_target:main',
            'ik_sim_node = miniarm_ik.ik_sim_node:main',
            'miniarm_ser = miniarm_ik.miniarm_ser:main',
            'ik_feed_tuning = miniarm_ik.ik_feed_tuning:main'
        ],
    },
)
