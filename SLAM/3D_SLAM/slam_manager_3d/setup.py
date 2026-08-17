from setuptools import find_packages, setup

package_name = 'slam_manager_3d'

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
    description='DSS 3D SLAM launch manager GUI (dss_lio_sam / HDL / RTAB-Map)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slam_manager_3d = slam_manager_3d.slam_manager_3d_node:main',
        ],
    },
)
