#!/bin/bash
# IP and SCRIPT_PATH must be set, SCRIPT_ARGS and MIDDLEWARE_ARGS are not mandatory. If MIDDLEWARE_ID is set, the middleware is used, else it isn't.
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

cd /tmp

# Create software directory in remote /tmp folder
ssh guest@${IP} << 'EOF'
    cd /tmp
    rm -rf ./software
    mkdir software
EOF

# Create .tar that contains all relevant files and copy to host
mkdir nuc_program_package
pushd nuc_program_package
if [ -z "${MIDDLEWARE_ID}" ] 
then
    tar czvf - nuc_package.tar.gz ${SCRIPT_PATH} ~/dev/cpm_base/cpm_lib/build/libcpm.so ~/dev/cpm_base/dds_idl | ssh guest@${IP} "cd /tmp/software;tar xzvf -"
else
    tar czvf - nuc_package.tar.gz ${SCRIPT_PATH} ~/dev/software/hlc ~/dev/cpm_base/cpm_lib/build/libcpm.so ~/dev/cpm_base/dds_idl | ssh guest@${IP} "cd /tmp/software;tar xzvf -"
fi
popd

# Copy further file modification orders to the NUC
scp ~/dev/software/LabControlCenter/bash/remote_start.bash guest@${IP}:/tmp/software
scp ~/dev/software/LabControlCenter/bash/environment_variables.bash guest@${IP}:/tmp/software
scp ~/dev/software/LabControlCenter/bash/tmux_middleware.bash guest@${IP}:/tmp/software
scp ~/dev/software/LabControlCenter/bash/tmux_script.bash guest@${IP}:/tmp/software

# Let the NUC handle the rest
if [ -z "${MIDDLEWARE_ID}" ] 
then
    sshpass ssh -t guest@${IP} 'bash /tmp/software/remote_start.bash' "--script_path=${SCRIPT_PATH} --script_arguments=${SCRIPT_ARGS} --middleware_arguments=${MIDDLEWARE_ARGS}"
else
    sshpass ssh -t guest@${IP} 'bash /tmp/software/remote_start.bash' "--script_path=${SCRIPT_PATH} --script_arguments=${SCRIPT_ARGS}"
fi