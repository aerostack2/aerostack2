#!/bin/bash

if [[ $1 == "cd" ]]; then
    source $AEROSTACK2_PATH/as2_cli/bash_utils/as2_cd.bash $@
else
    bash $AEROSTACK2_PATH/as2_cli/bash_utils/as2_core_function.bash $@
fi
