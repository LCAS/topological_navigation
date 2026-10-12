from setuptools import find_packages
from setuptools import setup
from glob import glob

package_name = 'topological_navigation'

setup(
    name=package_name,
    version='5.0.0',  # Major version bump - ROS1 code removed
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/', glob('config/*', recursive=True)),
        ('share/' + package_name + '/config/',
            glob('test/fixtures/mixed_actions_map.yaml')),
        ('share/' + package_name + '/launch/', glob('launch/*', recursive=True)),
        ('share/' + package_name + '/rviz/', glob('rviz/*', recursive=True)),
    ],
    install_requires=['setuptools', 'Pillow', 'PyYAML'],
    zip_safe=True,
    maintainer='Ibrahim Hroob',
    maintainer_email='ihroob@lincoln.ac.uk',
    description='ROS2 topological navigation package (ROS1 support removed in v4.0.0)',
    license='MIT',
    tests_require=['pytest', 'launch-pytest'],
    entry_points={
        'console_scripts': [
            # Core ROS2 Navigation Nodes
            'navigation2.py = topological_navigation.scripts.navigation2:main',
            'localisation2.py = topological_navigation.scripts.localisation2:main',
            'map_manager2.py = topological_navigation.scripts.map_manager2:main',

            # Supporting Utilities
            'manual_topomapping.py = topological_navigation.scripts.manual_topomapping:main',
            'map_recorder.py = topological_navigation.scripts.map_recorder:main',
            'validate_map.py = topological_navigation.validate_map:main',
            'topo_stats.py = topological_navigation.scripts.topo_stats:main',
            'map_analyser.py = topological_navigation.map_analyser:main',

            # Map conversion
            'convert_tmap.py = topological_navigation.convert_tmap:main',
            'topomap_to_map_image.py = topological_navigation.topomap_to_map_image:main',
        ],
    },

)
