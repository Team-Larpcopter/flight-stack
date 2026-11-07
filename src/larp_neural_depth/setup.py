from glob import glob
from setuptools import setup

package_name = "larp_neural_depth"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh") + glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Team Larpcopter",
    maintainer_email="team@larpcopter.local",
    description="Standalone Jetson TensorRT neural stereo depth estimation (Fast-FoundationStereo FP16) + Foxglove /viz output.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "neural_stereo_node = larp_neural_depth.neural_stereo_node:main",
        ]
    },
)
