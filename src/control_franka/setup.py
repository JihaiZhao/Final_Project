from setuptools import find_packages, setup
import glob

package_name = 'control_franka'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jihai',
    maintainer_email='zhao.2946@buckeyemail.osu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_cartesian = control_franka.move_cartesian:main',
            'delay_node = control_franka.delay_node:delay_entry',
            'run = control_franka.run:run_entry',
        ],
    },
)
