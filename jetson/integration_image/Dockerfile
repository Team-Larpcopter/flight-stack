ARG BASE=skybeta-team:0.3.2
FROM ${BASE}
SHELL ["/bin/bash", "-c"]
# The inherited image reports Boost 1.74 packages but contains 1.80 headers.
# Restore a consistent distro toolchain in this isolated image.
RUN apt-get update && apt-get install -y --reinstall libboost1.74-dev \
 && grep -q '^#define BOOST_VERSION 107400$' /usr/include/boost/version.hpp \
 && rm -rf /var/lib/apt/lists/*
# Native Air-FAR/ROG-Map/SUPER dependencies missing from the base team image.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-pcl-msgs ros-humble-pcl-conversions libpcl-dev \
 && rm -rf /var/lib/apt/lists/*
