
function _roscomplete_launch {
    arg="${COMP_WORDS[COMP_CWORD]}"
    COMPREPLY=()
    if [[ ${arg} =~ \-\-.* ]]; then
        COMPREPLY=(${COMPREPLY[@]} $(compgen -W "--files --args --nodes --find-node --child --local --screen --server_uri --run_id --wait --port --core --pid --dump-params" -- ${arg}))

    else
        _roscomplete_search_dir "( -type f -regex .*\.launch$ -o -type f -regex .*\.test$ )"
        if [[ $COMP_CWORD == 1 ]]; then
           COMPREPLY=($(compgen -o plusdirs -f -X "!*.launch" -- ${arg}) ${COMPREPLY[@]} $(compgen -o plusdirs -f -X "!*.test" -- ${arg}) ${COMPREPLY[@]})
        fi
        # complete roslaunch arguments for a launch file
        if [[ ${#COMP_WORDS[@]} -ge 2 ]]; then
            ROSLAUNCH_COMPLETE=$(which roslaunch-complete)
            if [[ -x ${ROSLAUNCH_COMPLETE} ]]; then
                # Call roslaunch-complete instead of roslaunch to get arg completion
                _roslaunch_args=$(${ROSLAUNCH_COMPLETE} ${COMP_WORDS[@]:1:2})
                # roslaunch-complete should be very silent about
                # errors and return 0 if it produced usable completion.
                if [[ $? == 0 ]]; then
                    COMPREPLY=($(compgen -W "${_roslaunch_args}" -- "${arg}") ${COMPREPLY[@]})
                    # FIXME maybe leave ${COMPREPLY[@]} out if we completed successfully.
                fi
            fi
        fi
    fi
}

