#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_ros1_bridge.bash [option...] \t This script build the ros1_bridge package." >&2
  echo
  echo "NOTE: The script requires both ROS 2 and ROS (1) workspaces to be built previously." >&2
  echo
  echo -e "\t--ros1_ws_dir \t Location of the ROS( 1) workspace where one has cloned px4_ros_com 'ros1' branch. Default: $HOME/px4_ros_com_ros1"
  echo -e "\t--ros1_distro \t Set ROS (1) distro name (melodic|noetic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros2_distro \t Set ROS2  distro name (dashing|eloquent|foxy|galactic|rolling). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros1_path \t\t Set ROS(1) environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS_DISTRO/"
  echo -e "\t--ros2_path \t\t Set ROS2 environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS_DISTRO/"
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

unset ROS_DISTRO
# One can pass the ROS_DISTRO's using the '--ros1_distro' and '--ros2_distro' args
if [ -z $ros1_distro ] && [ -z $ros2_distro]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename and
  # ROS install directory.
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
  ROS1_DISTRO="$ros1_distro"
  if [ -z $ros1_path ]; then
    echo "- Warning: You set a ROS (1) manually to be used."
    echo "  This assumes you want to use another ROS (1) version installed on your system. Please set the install location with '--ros1_path' arg! (ex: --ros_path ~/ros_src/kinetic/install/setup.bash)"
    exit 1
  else
    # source the ROS (1) environment (from arg)
    source $ros1_path
  fi
  ROS2_DISTRO="$ros2_distro"
  if [ -z $ros2_path ]; then
    echo "- Warning: You set a ROS 2 manually to be used."
    echo "  This assumes you want to use another ROS 2 version installed on your system. Please set the install location with '--ros2_path' arg! (ex: --ros_path ~/ros_src/eloquent/install/setup.bash)"
    exit 1
  else
    # source the ROS 2 environment (from arg)
    source $ros2_path
  fi
fi

# ROS2 dirs
ROS2_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS2_WS_SRC_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")" && pwd)
ROS2_WS_DIR=$(cd "$(dirname "$ROS2_WS_SRC_DIR")" && pwd)

# ROS1 dirs (one can pass the ROS1 workspace dir using '--ros1_ws_dir <ws_dir>')
ROS1_WS_DIR=${ros1_ws_dir:-"$(cd "$HOME/px4_ros_com_ros1" && pwd)"}

# check if the ROS1 workspace of px4_ros_com was built and source it.
if [ -z $ROS1_WS_DIR ]; then
  echo "ROS1 workspace does not exist!"
  exit 1
else
  if [ -f $ROS1_WS_DIR/devel/setup.bash ]; then
    unset ROS_DISTRO && source $ROS1_WS_DIR/devel/setup.bash
  elif [ -f $ROS1_WS_DIR/install/setup.bash ]; then
    unset ROS_DISTRO && source $ROS1_WS_DIR/install/setup.bash
  else
    echo "ROS1 workspace not built."
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

cd $SCRIPT_DIR
