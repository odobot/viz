from setuptools import find_packages, setup

package_name = 'mpu_6050_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/imu_node.py']),
    ],
    install_requires=['setuptools', 'rospkg'],
    zip_safe=True,
    maintainer='slambot',
    maintainer_email='slambot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu_6050_driver = mpu_6050_driver.mpu_6050_driver:main'
        ],
    },
)
