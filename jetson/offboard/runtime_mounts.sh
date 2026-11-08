#!/usr/bin/env bash
# Source from a launcher after setting here and image.
bash "$here/load_image.sh"
runtime=$(realpath "$here/runtime")
runtime_mounts=(-v "$runtime/flight-stack:/flight-stack:ro"
  -v "$runtime/opt/ros/humble:/opt/ros/humble:ro"
  -v "$runtime/opt/xrce-agent:/opt/xrce-agent:ro"
  -v "$runtime/usr/local/bin/MicroXRCEAgent:/usr/local/bin/MicroXRCEAgent:ro")

# Older GPU caches omitted the parser imported by TensorRT's Python package.
# Reuse the matching JetPack host library without rebuilding the dependency image.
parser=/usr/lib/aarch64-linux-gnu/libnvonnxparser.so.10
if [[ -f $parser ]]; then
  runtime_mounts+=(-v "$(realpath "$parser"):$parser:ro")
fi

# Supplement older GPU caches from the existing working LAS installation.
for package in torchgen numpy numpy.libs; do
  extra=${GPU_CACHE_DIR:-$here/../gpu}/python/$package
  if [[ -d $extra ]]; then
    runtime_mounts+=(-v "$(realpath "$extra"):/usr/local/lib/python3.10/dist-packages/$package:ro")
  fi
done
