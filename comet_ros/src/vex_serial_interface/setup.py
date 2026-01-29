from setuptools import find_packages, setup

package_name = 'vex_serial_interface'

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
    maintainer='Keval Shah',
    maintainer_email='kevals2005@gmail.com',
    description='Handles serial between Jetson and VEX Brain',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = vex_serial_interface.serial_interface:main'
        ],
    },
)
