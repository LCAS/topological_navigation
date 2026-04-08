from setuptools import find_packages, setup
from glob import glob

package_name = 'topological_nav_simulator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*')),
        ('share/' + package_name + '/config/', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Hroob',
    maintainer_email='ihroob@lincoln.ac.uk',
    description='Fake Nav2 action servers with virtual robot for testing topological navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_nav2_server = topological_nav_simulator.fake_nav2_server:main',
        ],
    },
)
