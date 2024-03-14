from setuptools import setup
import os
from glob import glob


package_name = 'r5sr_qr_detecter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))

    ],
    install_requires=['setuptools', 'opencv-python'],  #pythonの依存関係をここに記載する
    zip_safe=True,
    maintainer='Haruki Hasegawa',
    maintainer_email='robohase01@gmail.com',
    description='QR Detector for ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_detector_node = r5sr_qr_detecter.qr_detector_node:main',
        ],
    },
)
