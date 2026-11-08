# Assembly only: all application code was compiled on the ARM64-emulated host.
# This public ROS/Humble dependency image is already cached on the target.
ARG BASE=larp-flight-stack:humble
FROM ${BASE}
ADD gpu-runtime.tar.gz /
COPY engine /models/las2_m_320x480_d192_fp16.engine
# Stack updates leave the large GPU dependency layer cached.
ADD stack-runtime.tar.gz /
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV LD_LIBRARY_PATH=/flight-stack/sdk_install/lib:/usr/local/cuda-12.6/targets/aarch64-linux/lib:/usr/lib/aarch64-linux-gnu/nvidia
ENV PYTHONPATH=/usr/lib/python3.10/dist-packages
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
WORKDIR /flight-stack
ENTRYPOINT ["/flight-stack/jetson/offboard/entrypoint.sh"]
CMD ["bash"]
