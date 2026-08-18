FROM osrf/ros:humble-desktop

ENV ROS_DISTRO=humble
ENV AEROSTACK2_WORKSPACE=/root/aerostack2_ws
ENV AEROSTACK2_PATH=/root/aerostack2_ws/src/aerostack2

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

RUN apt-get install python3-flake8 \
                python3-flake8-builtins  \
                python3-flake8-comprehensions \
                python3-flake8-docstrings \
                python3-flake8-import-order \
                python3-flake8-quotes -y

RUN pip3 install pylint
RUN pip3 install flake8==4.0.1
RUN pip3 install pycodestyle==2.8
RUN pip3 install cmakelint cpplint

RUN apt-get install cppcheck lcov -y

RUN colcon mixin update default
RUN rm -rf log # remove log folder

RUN pip3 install colcon-lcov-result cpplint cmakelint
RUN pip3 install PySimpleGUI-4-foss

RUN mkdir -p /root/aerostack2_ws/src/
WORKDIR /root/aerostack2_ws/src/
COPY . ./aerostack2

WORKDIR /root/aerostack2_ws
RUN rosdep update
RUN rosdep fix-permissions
RUN rosdep install --from-paths src --ignore-src -r -y

# behaviortree_cpp 4.9.1 installs under lib/<multiarch>/; drop once humble ships 4.10.0.
# https://github.com/BehaviorTree/BehaviorTree.CPP/issues/1175
RUN find /opt/ros/$ROS_DISTRO/lib -path '*-linux-gnu/libbehaviortree_cpp.so' \
        -exec ln -sf {} /opt/ros/$ROS_DISTRO/lib/ \;

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "export AEROSTACK2_PATH=$AEROSTACK2_PATH" >> ~/.bashrc
RUN echo 'source $AEROSTACK2_PATH/as2_cli/setup_env.bash' >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
