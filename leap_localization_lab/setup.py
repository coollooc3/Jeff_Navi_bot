from glob import glob
from setuptools import find_packages, setup

setup(
    name='leap_localization_lab', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/leap_localization_lab']),
        ('share/leap_localization_lab', ['package.xml', 'README.md']),
        ('share/leap_localization_lab/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Jeff', maintainer_email='328209939+coollooc3@users.noreply.github.com',
    description='Isolated encoder odometry and SLAM data experiment.', license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': ['world_pose = leap_localization_lab.world_pose:main']},
)
