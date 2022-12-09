#!/bin/bash

if [ "$ZSH_VERSION" = "" ]; then
    TERM_EXTENSION=".bash"
else
    TERM_EXTENSION=".zsh"
fi

function arg_parse() {

POS_ARGS=()
SHORT_OPTS=()
LONG_OPTS=()
OPTS_ARGS=()
ALL_ARGS=()
# for each argument check if it is an option
for arg in "$@"; do
    ALL_ARGS+=("$arg")
    if [ "${arg:0:1}" = "-" ]; then
        # check if its long or short option
        if [ "${arg:1:1}" = "-" ]; then
            LONG_OPTS+=("${arg}")
            OPTS_ARGS+=("${arg}")
        else
            # foreach character in short option desglose it in its parts
            for char in $(echo "${arg:1}" | sed 's/./& /g'); do
                SHORT_OPTS+=("-${char}")
                OPTS_ARGS+=("-${char}")
            done
        fi
    else
        POS_ARGS+=("$arg")
    fi
done

CMD="${POS_ARGS[0]}"
POS_ARGS=("${POS_ARGS[@]:1}")

# echo "POS_ARGS: ${POS_ARGS[@]}"
# echo "SHORT_OPTS: ${SHORT_OPTS[@]}"
# echo "LONG_OPTS: ${LONG_OPTS[@]}"
# echo "OPTS_ARGS: ${OPTS_ARGS[@]}"
}


