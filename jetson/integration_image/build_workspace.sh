#!/usr/bin/env bash
# Inside the build image. No sensor access or running vehicle is required.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
workspace=$PWD
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
export CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS:-1}
export MAKEFLAGS="-j$CMAKE_BUILD_PARALLEL_LEVEL"
export VCPKG_MAX_CONCURRENCY="$CMAKE_BUILD_PARALLEL_LEVEL"
mkdir -p sdk-source
revision=623a3c02ca4440718fb45bfee5f30c6edff55d0c
archive=depthai-core-$revision.tar.gz
if [[ ! -f sdk-source/$archive ]]; then
  curl -fL --retry 3 "https://github.com/luxonis/depthai-core/archive/$revision.tar.gz" -o "sdk-source/$archive.part"
  mv "sdk-source/$archive.part" "sdk-source/$archive"
fi
(cd sdk-source && sha256sum -c ../jetson/integration_image/sdk-source.sha256)
if [[ ! -f sdk-source/depthai-core-v3.10.0/CMakeLists.txt ]]; then
  tar -xzf "sdk-source/$archive" -C sdk-source
  mv "sdk-source/depthai-core-$revision" sdk-source/depthai-core-v3.10.0
fi
bash jetson/integration_image/build_sdk.sh "$workspace/sdk-source/depthai-core-v3.10.0" "$workspace/build_sdk" "$workspace/sdk_install"
export CMAKE_PREFIX_PATH="$workspace/sdk_install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$workspace/sdk_install/lib:${LD_LIBRARY_PATH:-}"
# Native production path, including planners/executive; legacy Python DepthAI
# and CUDA depth experiments are not dependencies of this stack.
colcon build --base-paths src --executor sequential \
  --packages-up-to larp_hardware larp_autonomy flight_executive larp_px4_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
    -DENABLE_ARUCO_TAGS=OFF -DBoost_NO_BOOST_CMAKE=ON \
    -DBoost_NO_SYSTEM_PATHS=ON -DBOOST_ROOT=/usr \
    -DBOOST_INCLUDEDIR=/usr/include \
    "-DBOOST_LIBRARYDIR=/usr/lib/$(gcc -dumpmachine)"
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187
ctest --test-dir build/larp_hardware --output-on-failure
ctest --test-dir build/larp_navigation --output-on-failure
python3 -m pytest -q src/larp_autonomy/test src/larp_px4_bridge/test/test_core.py
ros2 pkg executables larp_hardware
ros2 pkg executables ov_srvins

bash jetson/integration_image/smoke_localization.sh
