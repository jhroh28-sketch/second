from setuptools import setup

package_name = 'hospital_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/poses.yaml']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Mission Dev',
    maintainer_email='dev@example.com',
    description='Mission controller built on nav2_simple_commander for ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = hospital_mission.mission_controller:main',
        ],
    },
)
