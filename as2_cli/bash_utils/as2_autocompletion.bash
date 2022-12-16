# as2 completion

as2_pkgs=$(${AEROSTACK2_PATH}/as2_cli/as2.bash --list-format list )
as2_projects=$(${AEROSTACK2_PATH}/as2_cli/as2.bash --list-format --projects list  )
as2_basic_folders="projects"

_as2_completion()
{
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    
    as2_commands='cd build list clean run project test'
    opts="--help -h -y"
    
    case $cur in
        -*)
            if [[ ${as2_commands} =~ ${prev} ]] ; then
                COMPREPLY=( $(compgen -W "${opts}" -- ${cur}) )
                return 0
            fi
        ;;
        *)
            if [[ ${prev} =~ "as2" ]] ; then
                COMPREPLY=( $(compgen -W "${as2_commands}" ${cur}) )
                return 0
            fi
            if [[ ${prev} =~ 'build' ]] ; then
                COMPREPLY=( $(compgen -W "${as2_pkgs}" ${cur}) )
                return 0
            fi
            if [[ ${prev} =~ 'test' ]] ; then
                COMPREPLY=( $(compgen -W "${as2_pkgs}" ${cur}) )
                return 0
            fi
            if [[ ${prev} =~ 'cd' ]] ; then
                COMPREPLY=( $(compgen -W "${as2_pkgs}" ${cur}) )
                COMPREPLY=( ${COMPREPLY[@]} $(compgen -W "${as2_projects}" -- ${cur}) )
                COMPREPLY=( ${COMPREPLY[@]} $(compgen -W "${as2_basic_folders}" -- ${cur}) )
                return 0
            fi
            if [[ ${prev} =~ 'clean' ]] ; then
                COMPREPLY=( $(compgen -W "${as2_pkgs}" ${cur}) )
                # append the opts to the COMPREPLY array
                opts="--all --help"
                COMPREPLY=( ${COMPREPLY[@]} $(compgen -W "${opts}" -- ${cur}) )
                return 0
            fi
        ;;
    esac
}

complete -F _as2_completion as2
complete -F _as2_completion as2.bash
complete -F _as2_completion ${AEROSTACK2_PATH}/as2_cli/as2.bash
