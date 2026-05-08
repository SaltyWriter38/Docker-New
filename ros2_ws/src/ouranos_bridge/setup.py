from setuptools import find_packages, setup

package_name = 'ouranos_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='jg2230@york.ac.uk',
    description='Bridges the Ouranos dashboard TCP server ⇄ ROS 2 topics.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = ouranos_bridge.bridge_node:main',
        ],
    },
)
