#!/bin/bash

cd /tmp

# Create .tar that contains all relevant files
mkdir nuc_program_package
pushd nuc_program_package
tar -czvf nuc_package.tar.gz ~/dev/software/hlc ~/dev/cpm_base/cpm_lib/build/libcpm.so ~/dev/cpm_base/dds_idl
popd

# Create software directory in remote /tmp folder
# TODO

# Copy .tar over to the NUC, as well as extraction orders
scp /tmp/nuc_program_package guest@$IP_TEMPLATE:/tmp/software
scp ~/dev/software/LabControlCenter/ui/setup/extract_in_remote.bash guest@$IP_TEMPLATE:/tmp/software #TODO: Less specific, other location

# Let the NUC handle the rest
sshpass ssh -t guest@$IP_TEMPLATE 'bash /tmp/software/extract_in_remote.bash' "${script_path} ${script_name} ${id} ${simulated_time}"