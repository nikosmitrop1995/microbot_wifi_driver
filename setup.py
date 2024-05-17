import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'microbot_wifi_motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['platformio.ini']),
        (os.path.join('share', package_name, 'src'), glob('src/*.cpp'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikolaos Mitropoulos',
    maintainer_email='nikosmitrpo1995@gmail.com',
    description='Package to compile and upload code to microcontroller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compile = microbot_wifi_motor_driver.compile:main',
            'upload = microbot_wifi_motor_driver.upload:main'
        ],
    },
)
