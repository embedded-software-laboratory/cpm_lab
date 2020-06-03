# This file is supposed to be used to create a package that is downloaded by the NUC AFTER IT HAS BOOTED
# -> Only files that are probably relevant to all Matlab scripts are uploaded at this points
# Scripts are still supposed to be uploaded using the LCC's UI (or your own upload script)

# Publish NUC package via http/apache for the HLCs to download

# exit when any command fails
set -e

cd /tmp
mkdir matlab_package
cp ~/dev/software/hlc/matlab/init_script.m ./matlab_package
cp ~/dev/software/hlc/matlab/QOS_READY_TRIGGER.xml ./matlab_package
tar -czvf matlab_package.tar.gz matlab_package
rm -f /var/www/html/nuc/matlab_package.tar.gz
cp ./matlab_package.tar.gz /var/www/html/nuc