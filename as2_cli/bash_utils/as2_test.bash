#!/bin/bash


usage() {
    echo "usage: $0 $CMD  [-h] [-d] [--ros2-only] [<pkg>]

AS2 test 

Perform test on a package

positional arguments:
  pkg          build up to PKG

optional arguments:
  -h, --help   show this help message and exit" 1>&2; exit 1;
}


# check if $ROS_VERSION is set

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS_DISTRO is not set"
    exit 1
fi

colcon_test() {
    pkg=$@
    if [[ -z $pkg ]]; then
        source /opt/ros/$ROS_DISTRO/setup$TERM_EXTENSION && cd ${AEROSTACK2_WORKSPACE} && colcon test --event-handlers console_direct+ 
    else
        source /opt/ros/$ROS_DISTRO/setup$TERM_EXTENSION; source ${AEROSTACK2_WORKSPACE}/install/setup$TERM_EXTENSION && cd ${AEROSTACK2_WORKSPACE} && colcon test --packages-select ${pkg}  --event-handlers console_direct+
    fi
}

for opt in "${OPTS_ARGS[@]}"; do
  echo $opt
    # filter spaces and ignore empty strings
    [ -z "$opt" ] && continue
    # check if the option ends with spaces and remove them
    opt="${opt%\ }"
    case $opt in
    -h | --help )
        usage
        exit 1
        ;;
    -* | --* )
        echo "invalid option: $opt"
        usage
        exit 1
        ;;
    esac
done

# positional args 
colcon_test ${POS_ARGS[@]}
