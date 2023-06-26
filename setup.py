from setuptools import setup

package_name = 'ros2_prescyent'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abiver',
    maintainer_email='alexis.biver@inria.fr',
    description='ros2 module for prescyent',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_predict = ros2_prescyent.run_predictor_node:main',
        ],
    }
)
