from setuptools import setup

package_name = 'topological_utils'

setup(
    name=package_name,
    version='3.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[''],
    zip_safe=True,
    maintainer='Marc Hanheide',
    maintainer_email='mhanheide@lincoln.ac.uk',
    description='The ros2 topological_utils package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_content.py = topological_navigation_utils.add_content:main.py',
            'add_edge.py = topological_navigation_utils.add_edge:main.py',
            'add_node.py = topological_navigation_utils.add_node:main.py',
            'add_node_tags.py = topological_navigation_utils.add_node_tags:main.py',
            'check_map = topological_navigation_utils.check_map.py',
            'crop_map.py = topological_navigation_utils.crop_map:main.py',
            'dummy_topological_navigation.py = topological_navigation_utils.dummy_topological_navigation:main.py',
            'draw_predicted_map.py = topological_navigation_utils.draw_predicted_map:main.py',
            'edge_length_analysis.py = topological_navigation_utils.edge_length_analysis:main.py',
            'edge_reconf_groups_to_tmap2.py = topological_navigation_utils.edge_reconf_groups_to_tmap2:main.py',
            'evaluate_predictions.py = topological_navigation_utils.evaluate_predictions:main.py',
            'insert_empty_map.py = topological_navigation_utils.insert_empty_map:main.py',
            'insert_map.py = topological_navigation_utils.insert_map:main.py',
            'joy_add_node.py = topological_navigation_utils.joy_add_node:main.py',
            'joy_add_waypoint.py = topological_navigation_utils.joy_add_waypoint:main.py',
            'list_maps = topological_navigation_utils.list_maps.py',
            'load_yaml_map.py = topological_navigation_utils.load_yaml_map:main.py',
            'load_json_map.py = topological_navigation_utils.load_json_map:main.py',
            'map_collection_change.py = topological_navigation_utils.map_collection_change:main.py',
            'map_export.py = topological_navigation_utils.map_export:main.py',
            'map_to_json.py = topological_navigation_utils.map_to_json:main.py',
            'map_to_yaml.py = topological_navigation_utils.map_to_yaml:main.py',
            'map_converter.py = topological_navigation_utils.map_converter:main.py',
            'migrate.py = topological_navigation_utils.migrate:main.py',
            'node_rm.py = topological_navigation_utils.node_rm:main.py',
            'node_metadata.py = topological_navigation_utils.node_metadata:main.py',
            'plot_topo_map.py = topological_navigation_utils.plot_topo_map:main.py',
            'plot_yaml.py = topological_navigation_utils.plot_yaml:main.py',
            'print_nav_stats.py = topological_navigation_utils.print_nav_stats:main.py',
            'rm_map_from_db.py = topological_navigation_utils.rm_map_from_db:main.py',
            'remove_node_tags.py = topological_navigation_utils.remove_node_tags:main.py',
            'rename_node = topological_navigation_utils.rename_node.py',
            'tmap_from_waypoints.py = topological_navigation_utils.tmap_from_waypoints:main.py',
            'tmap_to_yaml.py = topological_navigation_utils.tmap_to_yaml:main.py',
            'topological_map_update.py = topological_navigation_utils.topological_map_update:main.py',
            'visualise_map.py = topological_navigation_utils.visualise_map:main.py',
            'waypoints_to_yaml_tmap.py = topological_navigation_utils.waypoints_to_yaml_tmap:main.py'
        ],
    },

)

