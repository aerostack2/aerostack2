# Build context is the repository root.
ARG ROS_DISTRO=jazzy
# ros-base ships amd64 and arm64; osrf/ros desktop is amd64 only. rosdep pulls rviz2 and Gazebo.
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV AEROSTACK2_WORKSPACE=/root/aerostack2_ws
ENV AEROSTACK2_PATH=/root/aerostack2_ws/src/aerostack2
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN apt-get update -qq \
    && apt-get install --no-install-recommends -y -qq \
        git \
        python3-rosdep \
        python3-pip \
        python3-colcon-common-extensions \
        tmux \
        tmuxinator \
        ros-${ROS_DISTRO}-ros-gz \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir PySimpleGUI-4-foss

WORKDIR ${AEROSTACK2_WORKSPACE}
COPY . ./src/aerostack2
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

# behaviortree_cpp 4.9.1 installs under lib/<multiarch>/; drop once humble ships 4.10.0.
# https://github.com/BehaviorTree/BehaviorTree.CPP/issues/1175
RUN find /opt/ros/$ROS_DISTRO/lib -path '*-linux-gnu/libbehaviortree_cpp.so' \
        -exec ln -sf {} /opt/ros/$ROS_DISTRO/lib/ \;

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "export AEROSTACK2_PATH=$AEROSTACK2_PATH" >> ~/.bashrc \
    && echo 'source $AEROSTACK2_PATH/as2_cli/setup_env.bash' >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
