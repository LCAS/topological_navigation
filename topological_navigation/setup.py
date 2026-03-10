from glob import glob

from setuptools import find_packages, setup


package_name = "topological_navigation"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Topological Navigation Team",
    maintainer_email="maintainers@example.com",
    description="Minimal wrapper around nav2_route with BT-aware segment execution.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "route_bt_wrapper = topological_navigation.route_bt_wrapper:main",
            "closest_node_publisher = topological_navigation.closest_node_publisher:main",
            "initial_pose_publisher = topological_navigation.initial_pose_publisher:main",
            "random_target_navigator = topological_navigation.random_target_navigator:main",
            "gazebo_spawn_waiter = topological_navigation.gazebo_spawn_waiter:main",
            "interactive_node_markers = topological_navigation.interactive_node_markers:main",
        ],
    },
)
