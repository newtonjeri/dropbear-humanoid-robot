from setuptools import find_packages, setup

package_name = 'dropbear_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newtonjeri',
    maintainer_email='newtonkaris45@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_controller = dropbear_control.hand_joints_publisher_node:main',
            'leg_controller = dropbear_control.leg_joints_publisher_node:main',
            'walking_node = dropbear_control.walking_node:main',
        ],
    },
)
