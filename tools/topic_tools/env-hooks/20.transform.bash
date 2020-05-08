function _roscomplete_node_transform
{
    local arg opts
    COMPREPLY=()
    arg="${COMP_WORDS[COMP_CWORD]}"
    local cword=$COMP_CWORD
    for a in $(seq $((COMP_CWORD-1))); do
        if [ -z "${COMP_WORDS[a]//-*}" ]; then
            ((cword--))
        fi
    done
    local words=(${COMP_WORDS[@]//-*})

    if [[ $cword == 3 ]]; then
        opts=`rostopic list 2> /dev/null`
        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
    elif [[ $cword == 5 ]]; then
        opts=`rosmsg list 2> /dev/null`
        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
    fi
}

_sav_transform_roscomplete_rosrun=$(complete | { grep -w rosrun || test $? = 1; } | awk '{print $3}')

function is_transform_node
{
    local words=(${COMP_WORDS[@]//-*})
    [ ${#words[@]} -gt 2 ] && \
    [ "${words[1]}" = "topic_tools" ] && \
    [ "${words[2]}" = "transform" ]
}

function _roscomplete_rosrun_transform
{
    if is_transform_node; then
        _roscomplete_node_transform
    elif [[ "$_sav_transform_roscomplete_rosrun" != "_roscomplete_rosrun_transform" ]]; then

        eval "$_sav_transform_roscomplete_rosrun"
    fi
}

complete -F "_roscomplete_rosrun_transform" "rosrun"
