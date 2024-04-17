# Ubuntu 20.04 with nvidia-docker2 beta opengl support
FROM osrf/ros:galactic-desktop
USER root

ARG DEBIAN_FRONTEND=noninteractive

# Tools useful during development
RUN apt-get update -qq \
    && apt-get install --no-install-recommends -y -qq \
    build-essential \
    atop \
    cmake \
    cppcheck \
    expect \
    gdb \
    git \
    gnutls-bin \
    libbluetooth-dev \
    libccd-dev \
    libcwiid-dev \
    libfcl-dev \
    libgoogle-glog-dev \
    libspnav-dev \
    libusb-dev \
    python3-dbg \
    python3-empy \
    python3-numpy \
    python3-setuptools \
    python3-pip \
    python3-venv \
    software-properties-common \
    vim \
    net-tools \
    iputils-ping \
    xvfb \
    curl 

# NVIDIA_STUFF
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#         nvidia-cuda-toolkit \
#     && rm -rf /var/lib/apt/lists/* \
#     && apt-get clean

# Commands below run as the developer user
USER $USERNAME

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

## TODO: needed? Installation scripts are not launched directly and as2 utils not used...
# install aerostack2
RUN mkdir -p /home/${USERNAME}/aerostack2_ws/src/mbzirc
RUN cd /home/$USERNAME/aerostack2_ws/src \
    && git clone https://github.com/aerostack2/aerostack2.git 

ENV AEROSTACK2_PATH /home/$USERNAME/aerostack2_ws/src/aerostack2
ENV AEROSTACK2_WORKSPACE /home/$USERNAME/aerostack2_ws
ENV AEROSTACK2_PROJECTS /home/$USERNAME/aerostack2_ws/src/aerostack2/projects        
ENV AEROSTACK2_SIMULATION_DRONE_ID drone_sim_0

# check ignition version
ENV IGNITION_VERSION fortress

ENV PATH $PATH:$AEROSTACK2_PATH/scripts/
RUN echo 'alias as2="source $AEROSTACK2_PATH/scripts/as2.bash"' >> ~/.bashrc    
RUN echo 'source $AEROSTACK2_PATH/scripts/bash_utils/as2_autocompletion.bash ' >> ~/.bashrc    

USER root 
WORKDIR /home/$USERNAME/aerostack2_ws/src/aerostack2
# run with bash
RUN /bin/bash -c 'bash installers/install_core_dependencies.bash'

USER $USERNAME
WORKDIR /home/$USERNAME/aerostack2_ws/src/aerostack2
RUN /bin/bash -c 'sed -i "s|git@github.com:|https://github.com/|" installers/as2_rolling.repos'
RUN echo $(pwd) && vcs import --recursive < ./installers/as2_rolling.repos
RUN /bin/bash -c 'python3 installers/install_package_dependencies.py --repos ./installers/as2_rolling.repos -p .'

# RUN as2 build
WORKDIR /home/${USERNAME}/aerostack2_ws

RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash \
    && colcon build --symlink-install && echo "Build successfully "'

RUN /bin/sh -c 'echo ". /opt/ros/galactic/setup.bash" >> ~/.bashrc ' \
    && /bin/sh -c 'echo ". ~/aerostack2_ws/install/setup.bash" >> ~/.bashrc'

ENTRYPOINT ["/bin/bash"]

# NVIDIA_STUFF
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
# ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/local/cuda-10.1/compat/
