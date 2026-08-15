FROM osrf/ros@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# The desktop image provides Gazebo/ROS GUI tools but does not contain the
# navigation stack.  Install the exact ROS Noetic packages required by the
# repository so move_base and TEB are present in every fresh fixed container.
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       ros-noetic-navigation \
       ros-noetic-teb-local-planner \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_MASTER_URI=http://127.0.0.1:11311 \
    ROS_HOSTNAME=localhost \
    GAZEBO_MASTER_URI=http://127.0.0.1:11345

