from setuptools import setup

package_name = 'mks_servo_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='horseshitbot',
    maintainer_email='horseshitbot@todo.todo',
    description='ROS 2 driver for the MKS SERVO57D stepper-servo via Modbus RTU',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # name         = package.module:function
            'servo57d_node = mks_servo_driver.servo_node:main',
        ],
    },
)
