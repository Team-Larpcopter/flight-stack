from setuptools import setup

package_name = "larp_viz"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Team Larpcopter",
    maintainer_email="team@larpcopter.local",
    description="Low-bandwidth camera copies for Foxglove over Wi-Fi.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["viz_relay = larp_viz.viz_relay:main"]},
)
