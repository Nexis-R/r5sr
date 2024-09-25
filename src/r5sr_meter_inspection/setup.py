from setuptools import setup
import os
from glob import glob


package_name = 'r5sr_meter_inspection'

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
    install_requires=['setuptools', 'pyocr', 'opencv-python'],  #pythonの依存関係をここに記載する
    zip_safe=True,
    maintainer='あなたの名前',
    maintainer_email='your.email@example.com',
    description='ROS 2 vision analysis package in Python',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_converter_node = r5sr_meter_inspection.image_converter_node:main',
            'meter_value_calculator_node = r5sr_meter_inspection.meter_value_calculator_node:main',
            'inspection_result_node = r5sr_meter_inspection.inspection_result_node:main',
            'qr_detector_node = r5sr_meter_inspection.qr_detector_node:main',

        ],
    },
)
