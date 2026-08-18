# ROS 2 Jazzy + Gazebo Harmonic. Build context is the repository root.
FROM osrf/ros:jazzy-desktop

ENV ROS_DISTRO=jazzy
ENV AEROSTACK2_WORKSPACE=/root/aerostack2_ws
ENV AEROSTACK2_PATH=/root/aerostack2_ws/src/aerostack2

RUN apt-get update -qq \
    && apt-get install --no-install-recommends -y -qq \
        git \
        python3-rosdep \
        python3-pip \
        python3-colcon-common-extensions \
        tmux \
        tmuxinator \
        ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Do not clone aerostack2_viz; those packages already live in this repo.
WORKDIR ${AEROSTACK2_WORKSPACE}
COPY . ./src/aerostack2
RUN apt-get update && \
    rosdep update --rosdistro jazzy && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "export AEROSTACK2_PATH=$AEROSTACK2_PATH" >> ~/.bashrc
RUN echo 'source $AEROSTACK2_PATH/as2_cli/setup_env.bash' >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
