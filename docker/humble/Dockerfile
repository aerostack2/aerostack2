ARG ROS_DISTRO=humble
ARG IGNITION_VERSION=fortress

# Ubuntu 22.04 with nvidia-docker2 beta opengl support
FROM osrf/ros:${ROS_DISTRO}-desktop
USER root

ARG USERNAME=cvar
RUN useradd -ms /bin/bash $USERNAME

ENV ROS_DISTRO=${ROS_DISTRO}
ENV IGNITION_VERSION=${IGNITION_VERSION}

ENV AEROSTACK2_WORKSPACE=/home/${USERNAME}/aerostack2_ws

# Tools useful during development
RUN apt-get update -qq \
    && apt-get install --no-install-recommends -y -qq \
        git \
        python3-rosdep \
        python3-pip \
        python3-colcon-common-extensions \
        tmux \
        tmuxinator \
        ros-${ROS_DISTRO}-aerostack2 \
        ros-${ROS_DISTRO}-as2-ign-gazebo-assets \
        ros-${ROS_DISTRO}-as2-platform-ign-gazebo \
    && rm -rf /var/lib/apt/lists/*

# RUN sudo -H pip install setuptools==58.2.0 #fix of humble error
# RUN sudo -H pip install PySimpleGUI

WORKDIR ${AEROSTACK2_WORKSPACE}
RUN git clone https://github.com/aerostack2/aerostack2_viz.git src/aerostack2_viz
RUN apt-get update && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install'
# RUN . /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 6

USER ${USERNAME}

ENV AEROSTACK2_PATH=${AEROSTACK2_WORKSPACE}/src/aerostack2
ENV AEROSTACK2_SIMULATION_DRONE_ID=drone_sim_0

WORKDIR /home/${USERNAME}
COPY ["entrypoint.sh", "./"]
ENTRYPOINT ["./entrypoint.sh"]
CMD ["bash"]
