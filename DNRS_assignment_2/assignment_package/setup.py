import os
from glob import glob
from setuptools import find_packages, setup

package_name = "assignment_package"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "launch.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "urdf"), glob(os.path.join("urdf", "*"))),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ismilioshyn@gmail.com",
    description="Assignment package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "move_manipulator = assignment_package.move_manipulator:main",
        ],
    },
)
