# Loaded on demand by bash-completion for the four installed SRM commands.
_srm_complete()
{
    local cur prev cword
    local -a words
    _get_comp_words_by_ref -n = cur prev words cword
    local -a COMP_WORDS=("${words[@]}")
    local COMP_CWORD=$cword
    local command=${COMP_WORDS[0]##*/} action= options= choices= paths= index=1
    if [[ $cur == --*=* ]]; then prev=${cur%%=*}; cur=${cur#*=}; fi
    COMPREPLY=()
    if [[ $command == srm-nav ]]; then
        if ((COMP_CWORD == 1)); then
            mapfile -t COMPREPLY < <(compgen -W 'status topics hz tf sensors standby field localize drive check-standby start prepare cancel reset start-here return-home mission-status result standby-down bag-info bag-check map-check preview zero map pulse ros help -h --help' -- "$cur")
            return 0
        fi
        action=${COMP_WORDS[1]}
        index=2
        case $action in
            map|pulse) command=srm-$action ;;
            ros)
                # Reuse the installed ros2 completer with its original command context.
                local line=${COMP_LINE#*"${COMP_WORDS[0]}"}
                line=${line#*ros}
                local COMP_POINT=$((COMP_POINT - ${#COMP_LINE} + 4 + ${#line}))
                local COMP_LINE="ros2$line" COMP_CWORD=$((COMP_CWORD - 1))
                local -a words=(ros2 "${COMP_WORDS[@]:2}")
                local -a COMP_WORDS=("${words[@]}")
                if declare -F _comp_command_offset >/dev/null; then
                    _comp_command_offset 0
                else
                    _command_offset 0
                fi
                return 0 ;;
            field)
                options='-h --help --map-config --output --parking --parking-here --profile --response-model --allow-bypass --no-bypass'
                paths=directories ;;
            preview) options='-h --help --output --all'; paths=directories ;;
            drive) options='--allow-bypass --no-bypass'; paths=files ;;
            sensors|standby|localize|check-standby|result|bag-info|bag-check|map-check)
                paths=directories ;;
            return-home|zero) paths=files ;;
        esac
    fi
    case $command in
        srm-map)
            options='-h --help --duration --bag --vehicle-config --output --arena-config --profile --response-model' ;;
        srm-pulse)
            options='-h --help --device --output --axis --value --seconds --legacy20 --wheels-lifted --stop-available'
            choices='x y'
            local i
            for ((i=index; i<COMP_CWORD; i++)); do
                case ${COMP_WORDS[i]} in
                    --device|--output|--axis|--value|--seconds) ((i+=1)) ;;
                    x|y) choices= ;;
                esac
            done ;;
        srm-rviz) options='-h --help' ;;
    esac
    case $prev in
        --profile)
            choices='safe candidate'
            [[ $command != srm-nav || $action != field ]] || choices+=' gold-reference'
            mapfile -t COMPREPLY < <(compgen -W "$choices" -- "$cur")
            return 0 ;;
        --axis)
            mapfile -t COMPREPLY < <(compgen -W 'x y' -- "$cur")
            return 0 ;;
        --duration|--value|--seconds|--parking|--parking=*) return 0 ;;
        --bag|--vehicle-config|--map-config) paths=directories ;;
        --output)
            if [[ $command == srm-map || $action == field || $command == srm-pulse ]]; then
                paths=directories
            else
                paths=files
            fi ;;
        --arena-config|--response-model|--device) paths=files ;;
    esac
    # --parking consumes two numeric coordinates, including negative values.
    if ((COMP_CWORD >= 2)) && [[ ${COMP_WORDS[COMP_CWORD-2]} == --parking ]]; then
        return 0
    fi
    if [[ $cur == -* ]]; then
        mapfile -t COMPREPLY < <(compgen -W "$options" -- "$cur")
        return 0
    fi
    case $paths in
        directories) _filedir -d ;;
        files) _filedir ;;
    esac
    if [[ -z $cur ]]; then choices+=" $options"; fi
    local -a matches=()
    mapfile -t matches < <(compgen -W "$choices" -- "$cur")
    COMPREPLY+=("${matches[@]}")
    return 0
}

complete -o filenames -F _srm_complete srm-nav srm-map srm-pulse srm-rviz
