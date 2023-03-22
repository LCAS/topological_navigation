from setuptools import setup
package_name = 'topological_rviz_tools'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[''],
    zip_safe=True,
    maintainer='Michal Staniaszek',
    maintainer_email='staniasm@cs.bham.ac.uk',
    description='The ros2 rviz-based topological map construction package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_topmap_interface.py = topological_rviz_tools.scripts.python_topmap_interface.py'
        ],
    },

)


