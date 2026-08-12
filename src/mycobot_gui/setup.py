from setuptools import find_packages, setup

package_name = 'mycobot_gui'

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
    maintainer='ataullah',
    maintainer_email='shahataullah0314@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'control_panel = mycobot_gui.control_panel:main',
            'spawn_object = mycobot_gui.spawn_object:main',
        ],
    },
)
