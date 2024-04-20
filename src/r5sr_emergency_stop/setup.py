from setuptools import find_packages, setup

package_name = 'r5sr_emergency_stop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy','std_msgs'],
    zip_safe=True,
    maintainer='nexisr',
    maintainer_email='nexisr2016@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r5sr_emergency_stop_node = r5sr_emergency_stop.r5sr_emergency_stop_node:main'
        ],
    },
)
