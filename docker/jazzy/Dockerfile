# Ubuntu 24.04 with ROS 2 Jazzy (Gazebo Harmonic) and aerostack2 from the
# local checkout (build context is the repository root).
FROM osrf/ros:jazzy-desktop
USER root

ARG USERNAME=cvar
RUN useradd -ms /bin/bash $USERNAME

ENV ROS_DISTRO=jazzy
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
        # Gazebo Harmonic + ros_gz for Jazzy (ROS vendor packages)
        ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${AEROSTACK2_WORKSPACE}
COPY . ./src/aerostack2
RUN git clone https://github.com/aerostack2/aerostack2_viz.git src/aerostack2_viz
RUN apt-get update && \
    rosdep update --rosdistro jazzy && \
    rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install'

USER ${USERNAME}

ENV AEROSTACK2_PATH=${AEROSTACK2_WORKSPACE}/src/aerostack2
ENV AEROSTACK2_SIMULATION_DRONE_ID=drone_sim_0

WORKDIR /home/${USERNAME}
COPY ["docker/entrypoint.sh", "./"]
ENTRYPOINT ["./entrypoint.sh"]
CMD ["bash"]
