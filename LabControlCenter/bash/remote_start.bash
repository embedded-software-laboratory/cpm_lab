#!/bin/bash
#Get command line arguments
for i in "$@"
do
case $i in
    --script_path=*)
    SCRIPT_PATH="${i#*=}"
    shift # past argument=value
    ;;
    --script_arguments=*)
    SCRIPT_ARGS="${i#*=}"
    shift # past argument=value
    ;;
    --middleware_arguments=*)
    MIDDLEWARE_ARGS="${i#*=}"
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done

# Kill potential previous sessions
tmux kill-session -t "middleware"
tmux kill-session -t "script"

# Start middleware
tmux new-session -d -s "middleware" "cd /tmp/scripts/;bash tmux_middleware.bash --middleware_arguments='${MIDDLEWARE_ARGS}' &> tmux_middleware.txt"

# Start script
tmux new-session -d -s "script" "cd /tmp/scripts/;bash tmux_script.bash --script_path=${SCRIPT_PATH} --script_arguments='${SCRIPT_ARGS}' &> tmux_script.txt"
