from glob import glob

from setuptools import find_packages, setup

package_name = "flight_executive"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team Larpcopter",
    maintainer_email="team@larpcopter.local",
    description="Simulation-only PX4 takeoff, hold, and trajectory tracking executive.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "flight_executive_node = flight_executive.flight_executive_node:main",
        ],
    },
)
