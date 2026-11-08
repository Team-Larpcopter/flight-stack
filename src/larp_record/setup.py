from setuptools import find_packages, setup

package_name = "larp_record"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team Larpcopter",
    maintainer_email="team@larpcopter.local",
    description="Record all ROS 2 topics to a rosbag, with images compressed.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "record = larp_record.record:main",
        ],
    },
)
