#!/bin/bash
# IP and SCRIPT_PATH must be set, SCRIPT_ARGS and MIDDLEWARE_ARGS are not mandatory. If MIDDLEWARE_ARGS is set, the middleware is used, else it isn't (you must set a node id for the middleware).
# SCRIPT_PATH must contain the script name + type-ending as well
# This file is called by the LCC if distributed / remote deployment is selected
# DESCRIPTION: The given script is uploaded to the guest account on the specified NUC (IP), all other scripts within this folder (lab_control_center/bash) are uploaded as well.
#   Then, remote_start.bash is executed on the remote system.
#Get command line arguments
for i in "$@"
do
case $i in
    --ip=*)
    IP="${i#*=}"
    shift # past argument=value
    ;;
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

# Get directory of the script (use before first use of cd)
LCC_BASH_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"

# Create scripts directory in remote /tmp folder
ssh guest@${IP} << 'EOF'
    cd /tmp
    rm -rf ./scripts
    mkdir scripts
EOF

#Omit ../software and the script name to get the path relative to the software directory
[[ $SCRIPT_PATH =~ (.*)(^|/)(software/)(.*) ]];
RELATIVE_SCRIPT_PATH="${BASH_REMATCH[4]}"
RELATIVE_PATH="/${RELATIVE_SCRIPT_PATH%/*}" #Get string before last / (omit name of script)
PARENT_PATH="${BASH_REMATCH[1]}${BASH_REMATCH[2]}${BASH_REMATCH[3]}"
echo "${PARENT_PATH}"
echo "${RELATIVE_PATH}"

cd ${PARENT_PATH}
tar czvf - ./${RELATIVE_PATH} | ssh guest@${IP} "cd ~/dev/software/;tar xzvf -"

# Copy further file modification orders to the NUC
scp ${LCC_BASH_DIR}remote_start.bash guest@${IP}:/tmp/scripts
scp ${LCC_BASH_DIR}environment_variables.bash guest@${IP}:/tmp/scripts
scp ${LCC_BASH_DIR}tmux_middleware.bash guest@${IP}:/tmp/scripts
scp ${LCC_BASH_DIR}tmux_script.bash guest@${IP}:/tmp/scripts

# Let the NUC handle the rest
sshpass ssh -t guest@${IP} 'bash /tmp/scripts/remote_start.bash' "--script_path=~/dev/software/${RELATIVE_SCRIPT_PATH} --script_arguments='${SCRIPT_ARGS}' --middleware_arguments='${MIDDLEWARE_ARGS}'"