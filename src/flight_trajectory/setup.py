from setuptools import find_packages, setup

package_name = "flight_trajectory"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", ["config/fixed_course.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team Larpcopter",
    maintainer_email="team@larpcopter.local",
    description="Hard-coded course trajectories for the flight executive.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "course_trajectory_node = flight_trajectory.course_trajectory_node:main",
        ],
    },
)
