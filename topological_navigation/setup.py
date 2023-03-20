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
            'get_simple_policy.py = topological_navigation.get_simple_policy:main',
            'localisation.py = topological_navigation.localisation:main',
            'manual_edge_predictions.py = topological_navigation.manual_edge_predictions:main',
            'map_manager2.py = topological_navigation.map_manager2:main',
            'map_manager.py = topological_navigation.map_manager:main',
            'map_publisher.py = topological_navigation.map_publisher:main',
            'mean_based_prediction.py = topological_navigation.mean_based_prediction:main',
            'nav_client.py = topological_navigation.nav_client:main',
            'navigation.py = topological_navigation.navigation:main',
            'navstats_logger.py = topological_navigation.navstats_logger:main',
            'reconf_at_edges_server.py = topological_navigation.reconf_at_edges_server:main',
            'restrictions_manager.py = topological_navigation.restrictions_manager:main',
            'search_route.py = topological_navigation.search_route:main',
            'speed_based_prediction.py = topological_navigation.speed_based_prediction:main',
            'test_top_pred.py = topological_navigation.test_top_pred:main',
            'topological_prediction.py = topological_navigation.topological_prediction:main',
            'topological_transform_publisher.py = topological_navigation.topological_transform_publisher:main',
            'travel_time_estimator.py = topological_navigation.travel_time_estimator:main',
            'visualise_map2.py = topological_navigation.visualise_map2:main',
            'visualise_map.py = topological_navigation.visualise_map:main'
        ],
    },

)



