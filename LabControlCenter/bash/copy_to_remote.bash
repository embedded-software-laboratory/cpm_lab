#!/bin/bash
# IP and SCRIPT_PATH must be set, SCRIPT_ARGS and MIDDLEWARE_ARGS are not mandatory. If MIDDLEWARE_ARGS is set, the middleware is used, else it isn't (you must set a node id for the middleware).
# SCRIPT_PATH must contain the script name + type-ending as well
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

# Create scripts directory in remote /tmp folder
ssh guest@${IP} << 'EOF'
    cd /tmp
    rm -rf ./scripts
    mkdir scripts
EOF

# Create .tar that contains all relevant files and copy to host
cd /tmp
mkdir scripts
SCRIPT_FOLDER="${SCRIPT_PATH%/*}" #Get string before last /
/bin/cp -R ${SCRIPT_FOLDER} ./scripts
chmod a+rwx ./scripts
tar czvf - nuc_package.tar.gz scripts | ssh guest@${IP} "cd /tmp/scripts;tar xzvf -"

# Copy further file modification orders to the NUC
scp ~/dev/software/LabControlCenter/bash/remote_start.bash guest@${IP}:/tmp/scripts
scp ~/dev/software/LabControlCenter/bash/environment_variables.bash guest@${IP}:/tmp/scripts
scp ~/dev/software/LabControlCenter/bash/tmux_middleware.bash guest@${IP}:/tmp/scripts
scp ~/dev/software/LabControlCenter/bash/tmux_script.bash guest@${IP}:/tmp/scripts

# Let the NUC handle the rest
if ! [ -z "${MIDDLEWARE_ARGS}" ] 
then
    sshpass ssh -t guest@${IP} 'bash /tmp/scripts/remote_start.bash' "--script_path=${SCRIPT_PATH} --script_arguments='${SCRIPT_ARGS}' --middleware_arguments='${MIDDLEWARE_ARGS}'"
else
    sshpass ssh -t guest@${IP} 'bash /tmp/scripts/remote_start.bash' "--script_path=${SCRIPT_PATH} --script_arguments='${SCRIPT_ARGS}'"
fi