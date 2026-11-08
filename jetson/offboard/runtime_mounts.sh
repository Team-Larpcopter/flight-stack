#!/usr/bin/env bash
# Source from a launcher after setting here and image.
bash "$here/load_image.sh"
runtime=$(realpath "$here/runtime")
runtime_mounts=(-v "$runtime/flight-stack:/flight-stack:ro"
  -v "$runtime/opt/ros/humble:/opt/ros/humble:ro"
  -v "$runtime/opt/xrce-agent:/opt/xrce-agent:ro"
  -v "$runtime/usr/local/bin/MicroXRCEAgent:/usr/local/bin/MicroXRCEAgent:ro")
