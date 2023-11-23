from setuptools import setup

package_name = 'bayesian_topological_localisation'

setup(
    name=package_name,
    version='3.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['sympy>=1.5.1'],
    zip_safe=True,
    maintainer='Francesco Del Duchetto',
    maintainer_email='fdelduchetto@lincoln.ac.uk',
    description='The ros2 bayesian_topological_localisation package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localisation_node.py = bayesian_topological_localisation.scripts.localisation_node:main'
        ],
    },

)
