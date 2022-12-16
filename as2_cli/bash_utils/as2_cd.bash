#!/bin/bash

source $AEROSTACK2_PATH/as2_cli/bash_utils/argparser.bash
arg_parse "$@"

AEROSTACK_BASE_FOLDERS="['projects']"


if [[ $TERM_EXTENSION == ".zsh" ]]; then
    pkg=${POS_ARGS[1]}
else
    pkg=${POS_ARGS[0]}
fi

route=$("${AEROSTACK2_PATH}"/as2_cli/bash_utils/as2_core_function.bash list -v --list-format | sed -e 's/ /\n/g' | grep -E "^$pkg\$" -m 1 -A1| tail -n 1)

if [ -z "${pkg}" ]; then
    route=$AEROSTACK2_PATH
# check if the package is inside the $AEROSTACK_BASE_FOLDERS folders
elif [[ $AEROSTACK_BASE_FOLDERS == *"$pkg"* ]]; then
    route=$AEROSTACK2_PATH/$pkg
    
# check if this can be a project,
# projects are folders that are inside $AEROSTACK2_PATH/projects paths
# if the $AEROSTACK2_PATH/projects exists and it contains any folder
elif [[ -d $AEROSTACK2_PATH/projects/$pkg ]]; then
      route=$AEROSTACK2_PATH/projects/$pkg
fi 

if [ -z "$route" ]; then
  pkg="as2_${pkg}"
  route=$("${AEROSTACK2_PATH}"/as2_cli/bash_utils/as2_core_function.bash list -v --list-format | sed -e 's/ /\n/g' | grep -E "^$pkg\$" -m 1 -A1| tail -n 1)
  # if [ -n "$route" ]; then
    # echo "Changing to $pkg folder"
  # fi
fi

if [ -z "$route" ]; then
  echo "package $pkg not found" >&2
else
  cd "$route"
fi
    
unset route pkg CMD OPT_ARGS SHORT_OPTS LONG_OPTS POS_ARGS ALL_ARGS # clean up

