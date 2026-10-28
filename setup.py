from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trsa_lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'calibration'), glob('calibration/*.yaml')),
        (os.path.join('share', package_name, 'video'), glob('video/*')),
        (os.path.join('share', package_name, 'calibration', 'images'), glob('calibration/images/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafa',
    maintainer_email='rafa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_rectifier = trsa_lab1.image_rectifier:main',
            'camera_calibration_pub = trsa_lab1.camera_calibration_pub:main',
            'camera_driver = trsa_lab1.camera_driver:main',
            'image_reader = trsa_lab1.image_reader:main',
            'image_convert = trsa_lab1.image_convert:main',
            'object_detection = trsa_lab1.object_detection:main',
        ],
    },
)
