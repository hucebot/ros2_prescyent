import os
from glob import glob
from setuptools import setup, find_packages

package_name = "ros2_prescyent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools", "prescyent"],
    zip_safe=True,
    maintainer="abiver",
    maintainer_email="alexis.biver@inria.fr",
    description="ros2 module for prescyent",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "prescyent_predict = ros2_prescyent.run_predictor_node:main",
            "prescyent_publish_poses = ros2_prescyent.example.publisher_node:main",
        ],
    },
)
