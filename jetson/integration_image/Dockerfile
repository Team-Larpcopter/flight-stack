# Public multi-architecture Humble image; no organizer or local team image needed.
ARG BASE=ros:humble-ros-base-jammy
FROM ${BASE}
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git curl ca-certificates pkg-config unzip zip \
    autoconf autoconf-archive automake libtool patchelf \
    python3-colcon-common-extensions python3-pytest python3-numpy python3-scipy \
    python3-opencv python3-yaml libeigen3-dev libyaml-cpp-dev libopencv-dev \
    libboost-all-dev libusb-1.0-0-dev libudev-dev libpcl-dev \
    ros-humble-pcl-msgs ros-humble-pcl-conversions \
    ros-humble-cv-bridge ros-humble-image-transport ros-humble-message-filters \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
 && rm -rf /var/lib/apt/lists/*
# Also tolerate the historically used team base with inconsistent Boost headers.
RUN if ! grep -q '^#define BOOST_VERSION 107400$' /usr/include/boost/version.hpp; then \
      apt-get update && apt-get install -y --reinstall libboost1.74-dev \
      && rm -rf /var/lib/apt/lists/*; fi \
 && grep -q '^#define BOOST_VERSION 107400$' /usr/include/boost/version.hpp \
 && pkg-config --modversion opencv4 \
 && python3 -c 'import cv2; print(cv2.__version__)'

# Appended layer: keep all existing build layers/cache intact.
# PX4 v1.16 recommends Agent v2.4.3. Keep its Fast DDS libraries isolated
# from the ROS Humble middleware; only the agent wrapper loads them.
RUN apt-get update && apt-get install -y --no-install-recommends libasio-dev libtinyxml2-dev libssl-dev \
 && rm -rf /var/lib/apt/lists/* \
 && git init /tmp/xrce-agent \
 && git -C /tmp/xrce-agent remote add origin https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
 && git -C /tmp/xrce-agent fetch --depth 1 origin 73622810d984349b80bbac0ef55fc0b694d62222 \
 && git -C /tmp/xrce-agent checkout --detach FETCH_HEAD \
 && cmake -S /tmp/xrce-agent -B /tmp/xrce-agent/build \
      -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xrce-agent \
      -DUAGENT_BUILD_TESTS=OFF -DUAGENT_BUILD_EXECUTABLE=ON \
      -DUAGENT_P2P_PROFILE=OFF \
 && cmake --build /tmp/xrce-agent/build --parallel 1 \
 && cmake --install /tmp/xrce-agent/build \
 && printf '%s\n' '#!/bin/sh' \
      'export LD_LIBRARY_PATH="/opt/xrce-agent/lib:/opt/xrce-agent/lib64:${LD_LIBRARY_PATH:-}"' \
      'exec /opt/xrce-agent/bin/MicroXRCEAgent "$@"' > /usr/local/bin/MicroXRCEAgent \
 && chmod +x /usr/local/bin/MicroXRCEAgent \
 && (status=0; MicroXRCEAgent --help > /tmp/xrce-help.txt 2>&1 || status=$?; \
      test "$status" -eq 1 && grep -qi usage /tmp/xrce-help.txt) \
 && rm -rf /tmp/xrce-agent

# Visualization is appended so existing compiler/SDK dependency layers stay cached.
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-foxglove-bridge \
 && rm -rf /var/lib/apt/lists/*
