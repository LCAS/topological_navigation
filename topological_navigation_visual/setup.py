from setuptools import find_packages
from setuptools import setup
from glob import glob

package_name = 'topological_navigation_visual'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*', recursive=True)),
        ('share/' + package_name + '/config/', glob('config/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Hroob',
    maintainer_email='ihroob@lincoln.ac.uk',
    description='Visualization and interactive editing tools for topological maps',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Unified map visualization + interactive editor
            'topological_map_visualiser.py = topological_navigation_visual.scripts.topological_map_visualiser:main',

            # Route and occupancy visualization
            'topological_visual.py = topological_navigation_visual.scripts.topological_visual:main',

            # Policy visualization
            'policy_marker.py = topological_navigation_visual.policy_marker:main',
        ],
    },
)
