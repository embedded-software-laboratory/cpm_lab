#!/bin/bash
# This file is used on the remote system (NUC)
# DESCRIPTION: Kill previous sessions of the middleware and the selected script, then launch the currently selected script + middleware in a new tmux session (using tmux_....bash scripts)
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


# Kill potential previous sessions - DO NOT change these session names, unless you intend to change them in the whole software repo
rm -rf ~/dev/lcc_script_logs;mkdir -p ~/dev/lcc_script_logs

# Start middleware
tmux new-session -d -s "middleware" "cd /tmp/scripts/;bash tmux_middleware.bash --middleware_arguments='${MIDDLEWARE_ARGS}' &> ~/dev/lcc_script_logs/tmux_middleware.txt"

# Start script
tmux new-session -d -s "script" "cd /tmp/scripts/;bash tmux_script.bash --script_path=${SCRIPT_PATH} --script_arguments='${SCRIPT_ARGS}' &> ~/dev/lcc_script_logs/tmux_script.txt"
