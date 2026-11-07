from glob import glob

from setuptools import setup

package_name = "larp_oak"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Team Larpcopter",
    maintainer_email="team@larpcopter.local",
    description="OAK-D S2 driver: full-res mono for VIO + on-device depth, and Jetson depth filtering.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "oak_node = larp_oak.oak_node:main",
            "depth_filter = larp_oak.depth_filter:main",
        ],
    },
)
