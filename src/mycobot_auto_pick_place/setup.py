from setuptools import find_packages, setup

package_name = 'mycobot_auto_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ataullah',
    maintainer_email='shahataullah0314@gmail.com',
    description='Auto pick and place for MyCobot 280',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_node = mycobot_auto_pick_place.pick_place_node:main',
        ],
    },
)
