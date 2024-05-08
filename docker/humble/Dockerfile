FROM osrf/ros:humble-desktop

WORKDIR /root/

RUN apt-get update
RUN apt-get install apt-utils software-properties-common -y

RUN apt install git tmux tmuxinator -y
## ROS2 utils

RUN apt-get install python3-rosdep  \
                python3-pip     \
                python3-colcon-common-extensions \
                python3-colcon-mixin \
                ros-dev-tools -y

RUN apt-get install python3-flake8-builtins  \
                python3-flake8-comprehensions \
                python3-flake8-docstrings \
                python3-flake8-import-order \
                python3-flake8-quotes -y

RUN apt-get install cppcheck lcov -y

RUN colcon mixin update default
RUN rm -rf log # remove log folder

RUN pip3 install colcon-lcov-result cpplint
RUN pip3 install pysimplegui==4.60

RUN mkdir -p /root/aerostack2_ws/src/
WORKDIR /root/aerostack2_ws/src/
RUN git clone https://github.com/aerostack2/aerostack2.git -b main

WORKDIR /root/aerostack2_ws
RUN rosdep update
RUN rosdep fix-permissions
RUN rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo 'export AEROSTACK2_PATH=/root/aerostack2_ws/src/aerostack2' >> ~/.bashrc
RUN echo 'source $AEROSTACK2_PATH/as2_cli/setup_env.bash' >> ~/.bashrc
