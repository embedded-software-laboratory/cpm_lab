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

# Get log directory relative to this script's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
RELATIVE_LOG_DIR="${DIR}../../../lcc_script_logs"
ABSOLUTE_LOG_DIR="$(realpath "${RELATIVE_LOG_DIR}")"

# Kill potential previous sessions - DO NOT change these session names, unless you intend to change them in the whole software repo
tmux kill-session -t "middleware"
tmux kill-session -t "script"

# Clear previous log files, no longer done
# rm -rf '${ABSOLUTE_LOG_DIR}';mkdir -p '${ABSOLUTE_LOG_DIR}'

# Start middleware
tmux new-session -d -s "middleware" "cd /tmp/scripts/;bash tmux_middleware.bash --middleware_arguments='${MIDDLEWARE_ARGS}' &> '${ABSOLUTE_LOG_DIR}'/tmux_middleware.txt"

# Start script
tmux new-session -d -s "script" "cd /tmp/scripts/;bash tmux_script.bash --script_path=${SCRIPT_PATH} --script_arguments='${SCRIPT_ARGS}' &> '${ABSOLUTE_LOG_DIR}'/tmux_script.txt"
