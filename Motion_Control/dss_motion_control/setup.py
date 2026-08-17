from setuptools import find_packages, setup

package_name = 'dss_motion_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amap',
    maintainer_email='kukwonko@gmail.com',
    description='DSS vehicle Stanley / Pure Pursuit path tracking action servers',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dss_motion_control = dss_motion_control.dss_motion_control_node:main',
            'send_goal = dss_motion_control.send_goal:main',
            'waypoint_ui = dss_motion_control.waypoint_ui:main',
        ],
    },
)
