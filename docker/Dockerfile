# Ubuntu 22.04 with ros2 desktop
ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop

#create an user with a /home/cvar directory
ARG USERNAME=cvar
ARG PASSWD=test
RUN apt update && \
  apt-get install apt-utils sudo software-properties-common -y && \
  apt upgrade -y

## create an user
RUN useradd --create-home --shell /bin/bash $USERNAME && echo "$USERNAME:$PASSWD" | chpasswd && adduser $USERNAME sudo
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# Setups
ARG AEROSTACK2_BRANCH=docker_humble

ENV ROS_DISTRO=${ROS_DISTRO}
ENV AEROSTACK2_WORKSPACE=/home/${USERNAME}/aerostack2_ws
ENV AEROSTACK2_PATH=${AEROSTACK2_WORKSPACE}/src/aerostack2

# Tools useful during development
RUN sudo apt-get update && \
  sudo apt install git python3-rosdep python3-pip python3-colcon-common-extensions python3-rospkg ros-dev-tools tmux tmuxinator -y

# Cone Aerostack2 and dependencies
WORKDIR ${AEROSTACK2_WORKSPACE}
RUN git clone https://github.com/aerostack2/aerostack2.git -b ${AEROSTACK2_BRANCH} ${AEROSTACK2_PATH}
WORKDIR ${AEROSTACK2_PATH}
RUN vcs import < ${AEROSTACK2_PATH}/installers/thirdparty/as2_thirdparties.repos
RUN vcs import < ${AEROSTACK2_PATH}/installers/projects.repos
RUN vcs import < ${AEROSTACK2_PATH}/installers/thirdparty/px4.repos --recursive --shallow || true

# Set PX4 autopilot to not compile
RUN touch ${AEROSTACK2_PATH}/thirdparties/px4/thirdparties/COLCON_IGNORE

# Setup ROS 2
COPY setup_ros.sh /home/${USERNAME}/setup_ros.sh
RUN sudo chmod +x /home/${USERNAME}/setup_ros.sh && \
  /home/${USERNAME}/setup_ros.sh && \
  rm /home/${USERNAME}/setup_ros.sh

# Rosdep install
WORKDIR ${AEROSTACK2_WORKSPACE}
RUN sudo apt-get update && \
  sudo rm /etc/ros/rosdep/sources.list.d/20-default.list && \
  sudo rosdep init && \
  rosdep update --rosdistro ${ROS_DISTRO} && \
  rosdep install --from-paths src --ignore-src -r -y

# Compile aerostack2
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Compile px4
COPY install_px4.sh /home/${USERNAME}/install_px4.sh
RUN sudo chmod +x /home/${USERNAME}/install_px4.sh && \
  /home/${USERNAME}/install_px4.sh && \
  rm /home/${USERNAME}/install_px4.sh
