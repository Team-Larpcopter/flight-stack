# Larpcopter team image, layered on the organisers' skybeta base.
#
# `larp image build` copies this file to ~/team_ws/Dockerfile and runs skybeta's
# `host/run.sh build`, which tags the result skybeta-team:<VERSION>. skybeta's
# run.sh (and so the boot-time stack service) then uses it automatically.
# `larp image off` moves the Dockerfile aside, so the stack goes back to the
# organisers' image on its next start. Nothing in ~/skybeta is edited.
#
# Build context is ~/team_ws.
ARG BASE
FROM ${BASE}

SHELL ["/bin/bash", "-c"]

# Build larp_oak (1280x800@30 mono8 VIO + 800x500@30 depth + CUDA pyramid depth
# filter) and larp_viz (5 Hz Foxglove relay) directly into /root/skybeta_ws so
# run_slam.sh (which sets SKYBETA_UNDERLAY_ONLY=1) finds larp_oak, then replace
# oakd_vslam_bringup's launch file (originally a --symlink-install link) with
# larp_vslam.launch.py.
COPY src/flight-stack/src/larp_oak /root/skybeta_ws/src/larp_oak
COPY src/flight-stack/src/larp_viz /root/skybeta_ws/src/larp_viz
RUN source /opt/ros/humble/setup.bash \
 && source /root/skybeta_ws/install/local_setup.bash \
 && cd /root/skybeta_ws \
 && colcon build --packages-select larp_oak larp_viz \
 && SHARE=/root/skybeta_ws/install/oakd_vslam_bringup/share/oakd_vslam_bringup \
 && rm -f "${SHARE}/launch/oakd_vslam.launch.py" \
 && install -m 0644 /root/skybeta_ws/src/larp_oak/launch/larp_vslam.launch.py "${SHARE}/launch/oakd_vslam.launch.py"

LABEL io.larpcopter.overrides="larp_oak (1280x800@30 VIO + 800x500 filtered depth) + larp_viz"
