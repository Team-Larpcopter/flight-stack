# Public multi-architecture Humble image; no organizer or local team image needed.
ARG BASE=ros:humble-ros-base-jammy
FROM ${BASE}
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git curl ca-certificates pkg-config unzip zip \
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
