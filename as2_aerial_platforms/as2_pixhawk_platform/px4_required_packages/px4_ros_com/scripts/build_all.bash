#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_all.bash [option...] \t This script builds px4_ros_com workspaces for ROS 2 and ROS (1)" >&2
  echo
  echo -e "\t--ros1_ws_dir \t\t Location of the ROS (1) workspace where one has cloned px4_ros_com 'ros1' branch. Default: $HOME/px4_ros_com_ros1"
  echo -e "\t--ros1_distro \t\t Set ROS (1) distro name (melodic|noetic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros2_distro \t\t Set ROS 2 distro name (dashing|eloquent|foxy|galactic|rolling). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros1_path \t\t Set ROS (1) environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS_DISTRO/"
  echo -e "\t--ros2_path \t\t Set ROS 2 environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS_DISTRO/"
  echo -e "\t--verbose \t\t Add more verbosity to the console output"
  echo
  exit 0
fi

SCRIPT_DIR=$0
if [[ ${SCRIPT_DIR:0:1} != '/' ]]; then
  SCRIPT_DIR=$(dirname $(realpath -s "$PWD/$0"))
fi

# parse the arguments
while [ $# -gt 0 ]; do
  if [[ $1 == *"--"* ]]; then
    v="${1/--/}"
    if [ ! -z $2 ]; then
      declare $v="$2"
    else
      declare $v=1
    fi
  fi
  shift
done

# Check FastRTPSGen version
fastrtpsgen_version_out=""
if [[ -z $FASTRTPSGEN_DIR ]]; then
  fastrtpsgen_version_out="$FASTRTPSGEN_DIR/$(fastrtpsgen -version)"
else
  fastrtpsgen_version_out=$(fastrtpsgen -version)
fi
if [[ -z $fastrtpsgen_version_out ]]; then
  echo "FastRTPSGen not found! Please build and install FastRTPSGen..."
  exit 1
else
  fastrtpsgen_version="${fastrtpsgen_version_out: -5:-2}"
  if ! [[ $fastrtpsgen_version =~ ^[0-9]+([.][0-9]+)?$ ]] ; then
    fastrtpsgen_version="1.0"
    echo "FastRTPSGen version: ${fastrtpsgen_version}"
  else
    echo "FastRTPSGen version: ${fastrtpsgen_version_out: -5}"
  fi
fi

unset ROS_DISTRO
# One can pass the ROS_DISTRO's using the '--ros1_distro' and '--ros2_distro' args
if [ -z $ros1_distro ] && [ -z $ros2_distro]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename and
  # ROS install directory
  # The distros are order by priority (according to being LTS vs non-LTS)
  case "$(lsb_release -s -c)" in
  "bionic")
    if [ -d "/opt/ros/melodic" ]; then
      export ROS1_DISTRO="melodic"
    else
      if [ -z $ros1_path ]; then
        echo "- No ROS (1) distro installed or not installed in the default directory."
        echo "  If you are using a ROS (1) version installed from source, please set the install location with '--ros1_path' arg! (ex: ~/ros_src/melodic/install). Otherwise, please install ROS Melodic following http://wiki.ros.org/melodic/Installation"
        exit 1
      else
        # source the ROS (1) environment (from arg)
        source $ros1_path
        export ROS1_DISTRO="$(rosversion -d)"
      fi
    fi
    if [ -d "/opt/ros/dashing" ]; then
      export ROS2_DISTRO="dashing"
    elif [ -d "/opt/ros/eloquent" ]; then
      export ROS2_DISTRO="eloquent"
    else
      if [ -z $ros2_path ]; then
        echo "- No ROS 2 distro installed or not installed in the default directory."
        echo "  If you are using a ROS 2 version installed from source, please set the install location with '--ros1_path' arg! (ex: ~/ros_src/eloquent/install). Otherwise, please install ROS 2 Dashing following https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Binary/"
        exit 1
      else
        # source the ROS2 environment (from arg)
        source $ros2_path
        export ROS2_DISTRO="$(rosversion -d)"
      fi
    fi
    ;;
  "focal")
    if [ -d "/opt/ros/noetic" ]; then
      export ROS1_DISTRO="noetic"
    else
      if [ -z $ros1_path ]; then
        echo "- No ROS (1) distro installed or not installed in the default directory."
        echo "  If you are using a ROS (1) version installed from source, please set the install location with '--ros1_path' arg! (ex: ~/ros_src/noetic/install). Otherwise, please install ROS Noetic following http://wiki.ros.org/noetic/Installation"
        exit 1
      else
        # source the ROS (1) environment (from arg)
        source $ros1_path
        export ROS1_DISTRO="$(rosversion -d)"
      fi
    fi
    if [ -d "/opt/ros/foxy" ]; then
      export ROS2_DISTRO="foxy"
    elif [ -d "/opt/ros/galactic" ]; then
      ROS_DISTRO="galactic"
    elif [ -d "/opt/ros/rolling" ]; then
      ROS_DISTRO="rolling"
    elif [ -d "/opt/ros/humble" ]; then
      ROS_DISTRO="humble"
    else
      if [ -z $ros2_path ]; then
        echo "- No ROS 2 distro installed or not installed in the default directory."
        echo "  If you are using a ROS 2 version installed from source, please set the install location with '--ros1_path' arg! (ex: ~/ros_src/foxy/install). Otherwise, please install ROS 2 Foxy following https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Binary/"
        exit 1
      else
        # source the ROS2 environment (from arg)
        source $ros2_path
        export ROS2_DISTRO="$(rosversion -d)"
      fi
    fi
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
else
  if [ -z $ros1_path ]; then
    echo "- Warning: You set a ROS (1) manually to be used."
    echo "  This assumes you want to use another ROS (1) version installed on your system. Please set the install location with '--ros1_path' arg! (ex: --ros_path ~/ros_src/kinetic/install/setup.bash)"
    exit 1
  else
    export ROS1_DISTRO="$ros1_distro"
  fi
  if [ -z $ros2_path ]; then
    echo "- Warning: You set a ROS 2 manually to be used."
    echo "  This assumes you want to use another ROS 2 version installed on your system. Please set the install location with '--ros2_path' arg! (ex: --ros_path ~/ros_src/eloquent/install/setup.bash)"
    exit 1
  else
    export ROS2_DISTRO="$ros2_distro"
  fi
fi

# setup the required path variables
export ROS2_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
export ROS2_WS_SRC_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")" && pwd)
export ROS2_WS_DIR=$(cd "$(dirname "$ROS2_WS_SRC_DIR")" && pwd)

# by default set to $HOME/px4_ros_com_ros1 but user can use command line arg instead
export ROS1_WS_DIR=${ros1_ws_dir:-"$(cd "$HOME/px4_ros_com_ros1" && pwd)"}

# clone ros1_bridge to the workspace dir
if [ -z $no_ros1_bridge ] && [ ! -d "$ROS2_WS_SRC_DIR/ros1_bridge" ]; then
  cd $ROS2_WS_SRC_DIR && git clone https://github.com/ros2/ros1_bridge.git -b $ROS2_DISTRO
fi

# Check if gnome-terminal exists
if [ -x "$(command -v gnome-terminal)" ]; then
  SHELL_TERM="gnome-terminal --tab -- /bin/bash -c"
  echo $SHELL_TERM
# Check if xterm exists
elif [ -x "$(command -v xterm)" ]; then
  SHELL_TERM="xterm -e"
  echo $SHELL_TERM
fi

$SHELL_TERM \
  '''
    unset ROS_DISTRO
    # source the ROS1 environment
    if [ -z $ros1_path ]; then
      source /opt/ros/$ROS1_DISTRO/setup.bash
    else
      source $ros1_path
    fi

    # check if the ROS1 side of px4_ros_com was built and source it. Otherwise, build it
    printf "\n************* Building ROS1 workspace *************\n\n"
    # build the ROS1 workspace of the px4_ros_com package
    cd $ROS1_WS_DIR && catkin init && catkin build

    # source the ROS1 workspace environment so to have it ready to use
    source $ROS1_WS_DIR/devel/setup.bash

    printf "\nROS1 workspace ready...\n\n"
    exec /bin/bash
  ''' &

printf "\n************* Building ROS2 workspace *************\n\n"

# source the ROS2 environment
if [ -z $ros2_path ]; then
  source /opt/ros/$ROS2_DISTRO/setup.bash
else
  source $ros2_path
fi
# build px4_ros_com package, except the ros1_bridge
[ ! -v $verbose ] && colcon_output=$(echo "--event-handlers console_direct+")
cd $ROS2_WS_DIR && colcon build --packages-skip ros1_bridge $colcon_output

$SHELL_TERM \
  '''
    unset ROS_DISTRO
    # check if the ROS1 workspace of px4_ros_com and px4_msgs was built and source it.
    if [ -z $ROS1_WS_DIR ]; then
      echo "ROS1 workspace does not exist!"
      exec /bin/bash
      exit 1
    else
      if [ -f $ROS1_WS_DIR/devel/setup.bash ]; then
        unset ROS_DISTRO && source $ROS1_WS_DIR/devel/setup.bash
      elif [ -f $ROS1_WS_DIR/install/setup.bash ]; then
        unset ROS_DISTRO && source $ROS1_WS_DIR/install/setup.bash
      else
        echo "ROS1 workspace not built."
        exec /bin/bash
        exit 1
      fi
    fi

    # source the ROS2 workspace
    unset ROS_DISTRO && source $ROS2_WS_DIR/install/setup.bash

    printf "\n************* Building ros1_bridge *************\n\n"
    # build the ros1_bridge only
    [ ! -v $verbose ] && colcon_output=$(echo "--event-handlers console_direct+")
    cd $ROS2_WS_DIR && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install --packages-select ros1_bridge --cmake-force-configure $colcon_output

    # source the ROS2 workspace environment so to have it ready to use
    unset ROS_DISTRO && source $ROS2_WS_DIR/install/setup.bash

    printf "\nros1_bridge workspace ready...\n\n"
    exec /bin/bash
  ''' &

# source the ROS2 workspace environment so to have it ready to use
source $ROS2_WS_DIR/install/setup.bash

printf "\nROS2 workspace ready...\n\n"

$SHELL_TERM \
  '''
  # source the ROS2 workspace environment so to have it ready to use
  source $ROS2_WS_DIR/install/setup.bash

  printf "To start the microRTPS bridge agent, use \"micrortps_agent [options]\"\n\n"
  exec /bin/bash
  ''' &
