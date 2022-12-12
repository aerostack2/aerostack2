#!/bin/bash

usage() {
    echo "usage: as2 clean <pkg> [-a] [-h]

AS2 clean workspace

optional arguments:
    -a ,--all             clean all packages
    -h, --help            show this help message and exit" 2>&2; exit 1;
}

clean_all=0

pkgs_list_as2_=$(${AEROSTACK2_PATH}/as2_cli/bash_utils/as2_core_function.bash list --list-format)

function check_if_package_exists() {
    pkg=$1
    for p in $pkgs_list_as2_; do
        if [ "$p" == "$pkg" ]; then
            return 0
        fi
    done
    return 1
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
        -a | --all)
            clean_all=1
        ;;
        -* | --* )
            echo "invalid option: $opt"
            usage
            exit 1
        ;;
    esac
done

# check if $AEROSTACK2_WORKSPACE is set
if [ -z "$AEROSTACK2_WORKSPACE" ]; then
    echo "AEROSTACK2_WORKSPACE is not set"
    exit 1
fi

if [ $clean_all -eq 1 ]; then
    # ask for confirmation
    read -p "Are you sure? " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Cleaning all"
        cd ${AEROSTACK2_WORKSPACE} && rm -rf build install log > /dev/null
    else
        echo "Canceled"
        exit 1
    fi
else
    # echo all ${POS_ARGS[@]} together
    if [ ${#POS_ARGS[@]} -eq 0 ]; then
        echo "No package specified"
        usage
        exit 1
    fi
    echo "Cleaning ${POS_ARGS[@]}"
    
    # ask for confirmation
    read -p "Are you sure? " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        for arg in "${POS_ARGS[@]}"; do
            # echo "Cleaning $arg"
            
            #check if package exists
            if ! check_if_package_exists $arg; then
                echo "Package $arg not found" > /dev/stderr
                exit 1
            fi
            
            if [ -d "${AEROSTACK2_WORKSPACE}/build/${arg}" ]; then
                rm -rf ${AEROSTACK2_WORKSPACE}/build/${arg}
            fi
            if [ -d "${AEROSTACK2_WORKSPACE}/install/${arg}" ]; then
                rm -rf ${AEROSTACK2_WORKSPACE}/install/${arg}
            fi
            rm -rf ${AEROSTACK2_WORKSPACE}/log
            
        done
    else
        echo "Canceled"
        exit 1
    fi
    
    echo "Done"
fi

