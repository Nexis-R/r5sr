from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'r5sr_image_proccesing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='hasegawa',
    maintainer_email='robohase01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_motion_detect_node = r5sr_image_proccesing.image_motion_detect_node:main',
            'qr_detector_node = r5sr_image_proccesing.qr_detector_node:main',

        ],
    },
)
