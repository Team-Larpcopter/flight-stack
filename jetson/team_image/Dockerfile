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

# On-device stereo depth (jetson/team_image/oakd_vslam.yaml) and the matching VIO
# launch (cuVSLAM on the OAK's rectified mono8 pair). The originals are colcon
# --symlink-install links into the build tree, so remove each link first.
ARG SHARE=/root/skybeta_ws/install/oakd_vslam_bringup/share/oakd_vslam_bringup
COPY src/flight-stack/jetson/team_image/oakd_vslam.yaml /tmp/larp/oakd_vslam.yaml
COPY src/flight-stack/jetson/team_image/oakd_vslam.launch.py /tmp/larp/oakd_vslam.launch.py
RUN rm -f "${SHARE}/config/oakd_vslam.yaml" "${SHARE}/launch/oakd_vslam.launch.py" \
 && install -m 0644 /tmp/larp/oakd_vslam.yaml "${SHARE}/config/oakd_vslam.yaml" \
 && install -m 0644 /tmp/larp/oakd_vslam.launch.py "${SHARE}/launch/oakd_vslam.launch.py" \
 && rm -rf /tmp/larp

LABEL io.larpcopter.overrides="oakd_vslam.yaml oakd_vslam.launch.py (on-device depth)"
