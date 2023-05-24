from setuptools import setup

package_name = 'topological_navigation'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['sympy>=1.5.1'],
    zip_safe=True,
    maintainer='Adam Binch',
    maintainer_email='abinch@sagarobotics.com',
    description='The ros2 topological_navigation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_simple_policy.py = topological_navigation.scripts.get_simple_policy:main',
            'localisation.py = topological_navigation.scripts.localisation:main',
            'manual_edge_predictions.py = topological_navigation.scripts.manual_edge_predictions:main',
            'map_manager2.py = topological_navigation.scripts.map_manager2:main',
            'map_manager.py = topological_navigation.scripts.map_manager:main',
            'map_publisher.py = topological_navigation.scripts.map_publisher:main',
            'mean_based_prediction.py = topological_navigation.scripts.mean_based_prediction:main',
            'nav_client.py = topological_navigation.scripts.nav_client:main',
            'navigation.py = topological_navigation.scripts.navigation:main',
            'navstats_logger.py = topological_navigation.scripts.navstats_logger:main',
            'reconf_at_edges_server.py = topological_navigation.scripts.reconf_at_edges_server:main',
            'restrictions_manager.py = topological_navigation.scripts.restrictions_manager:main',
            'search_route.py = topological_navigation.scripts.search_route:main',
            'speed_based_prediction.py = topological_navigation.scripts.speed_based_prediction:main',
            'test_top_pred.py = topological_navigation.scripts.test_top_pred:main',
            'topological_prediction.py = topological_navigation.scripts.topological_prediction:main',
            'topological_transform_publisher.py = topological_navigation.scripts.topological_transform_publisher:main',
            'travel_time_estimator.py = topological_navigation.scripts.travel_time_estimator:main',
            'visualise_map2.py = topological_navigation.scripts.visualise_map2:main',
            'visualise_map.py = topological_navigation.scripts.visualise_map:main',
            'visualise_map_ros2.py = topological_navigation.scripts.visualise_map_ros2:main',
            'topomap_marker.py = topological_navigation.topomap_marker:main',
            'policy_marker.py = topological_navigation.policy_marker:main'
        ],
    },

)



