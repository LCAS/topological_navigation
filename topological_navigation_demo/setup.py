from glob import glob

from setuptools import find_packages, setup

package_name = "topological_navigation_demo"

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
    description="TurtleBot3 Gazebo demo for topological_navigation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_localizer = topological_navigation_demo.fake_localizer:main",
            "gazebo_spawn_waiter = topological_navigation_demo.gazebo_spawn_waiter:main",
            "initial_pose_publisher = topological_navigation_demo.initial_pose_publisher:main",
            "random_target_navigator = topological_navigation_demo.random_target_navigator:main",
        ],
    },
)
