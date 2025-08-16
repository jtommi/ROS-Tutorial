import os
from glob import glob

from setuptools import find_packages, setup

package_name = "my_bot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.py"),
        ),
        (
            os.path.join("share", package_name, "description"),
            glob("description/*.xacro"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thomas",
    maintainer_email="johanns.thomas@gmail.com",
    description="Some bot",
    license="GNU GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
