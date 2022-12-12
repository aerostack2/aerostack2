#!/bin/bash

declare endColour="\033[0m\e[0m"
declare greenColour="\e[0;32m\033[1m"
declare redColour="\e[0;31m\033[1m"
declare blueColour="\e[0;34m\033[1m"
declare yellowColour="\e[0;33m\033[1m"
declare purpleColour="\e[0;35m\033[1m"
declare turquoiseColour="\e[0;36m\033[1m"
declare grayColour="\e[0;37m\033[1m"

usage() {
    echo "usage: $0 [-h] [-y] {cd,build,clean,test,project,run} ...

Aerostack2 toolbox for ease the use of the AS2 pipeline

optional arguments:
  -h, --help            show this help message and exit
  -y, --yes             answer yes to all questions

AS2 commands:
  {cd,build,clean,test,project,run}  action to do
    cd                  change directory
    list                list all packages
    build               build help
    clean               clean workspace
    project             list and install aerostack2 projects
    test                run tests
    run" 1>&2; exit 1;
}

# put arguments in an env variables

source $AEROSTACK2_PATH/as2_cli/bash_utils/argparser.bash
arg_parse $@


case $CMD in
    build )
        # echo "building... with args: ${OPT_ARGS[@]}"
        source $AEROSTACK2_PATH/as2_cli/bash_utils/as2_build.bash
    ;;
    test )
        # echo "building... with args: ${OPT_ARGS[@]}"
        source $AEROSTACK2_PATH/as2_cli/bash_utils/as2_test.bash
    ;;
    list )
        # echo "building... with args: ${OPT_ARGS[@]}"
        source $AEROSTACK2_PATH/as2_cli/bash_utils/as2_list.bash
    ;;
    clean )
        # source as2_clean ${OPT_ARGS[@]} $*
        source $AEROSTACK2_PATH/as2_cli/bash_utils/as2_clean.bash
    ;;
    project )
        # source as2_clean ${OPT_ARGS[@]} $*
        # parse the arguments without the first one into the var $ARGS
        source $AEROSTACK2_PATH/as2_cli/bash_utils/as2_projects.bash ${ALL_ARGS[@]:1}
    ;;
    * )
        usage
    ;;
esac

unset CMD OPT_ARGS SHORT_OPTS LONG_OPTS POS_ARGS ALL_ARGS # clean up
