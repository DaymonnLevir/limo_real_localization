"""Setup configuration for informative_planner."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'informative_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Usado pelo postprocess_run depois da missão.
        ('share/' + package_name + '/scripts',
            ['scripts/gp_propagation_video.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='joaorafaelguimaraes@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'benchmark_planner = informative_planner.benchmark_planner:main',
            'gaussian_feeder = informative_planner.gaussian_feeder:main',
            'postprocess_run = informative_planner.postprocess_run:main',
            'limo_mission = informative_planner.limo_mission:main',
            'point_quality_node = informative_planner.point_quality_node:main',
            'bounds_map_publisher = informative_planner.bounds_map_publisher:main',
        ],
    },
)
