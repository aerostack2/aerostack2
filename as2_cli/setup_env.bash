#!/bin/bash

if [ "$ZSH_VERSION" = "" ]; then
    TERM_EXTENSION=".bash"
else
    TERM_EXTENSION=".zsh"
fi

# check if AEROSTACK2_PATH is set
if [ -z "$AEROSTACK2_PATH" ]; then
    echo "AEROSTACK2_PATH env var is unset. Please set it to the path of the AEROSTACK2_PATH folder"
else
        
        export AEROSTACK2_WORKSPACE=$(dirname $(dirname ${AEROSTACK2_PATH}))
        export AEROSTACK2_PROJECTS="$AEROSTACK2_PATH/projects/"
        export PATH=$PATH:$AEROSTACK2_PATH/as2_cli/
        
        ENV_VARIABLES_FILE="$AEROSTACK2_PATH/as2_cli/env_variables.bash"
        if test -f "$ENV_VARIABLES_FILE"; then
            source $ENV_VARIABLES_FILE
        else
            echo "export AEROSTACK2_SIMULATION_DRONE_ID=drone_sim_${USER}_0" >> $ENV_VARIABLES_FILE
            source $ENV_VARIABLES_FILE
        fi
        
        alias as2="source $AEROSTACK2_PATH/as2_cli/as2.bash"
        # enable custom AS2 bash completions
        
        if [[ -f "$AEROSTACK2_WORKSPACE/install/setup$TERM_EXTENSION" && ! -z "$ROS_DISTRO" ]]; then
            source $AEROSTACK2_WORKSPACE/install/setup$TERM_EXTENSION
        fi
        source $AEROSTACK2_PATH/as2_cli/bash_utils/as2_autocompletion.bash
    
fi
