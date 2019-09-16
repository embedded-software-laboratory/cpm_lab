#!/bin/bash

# This script should be installed on each NUC under "/root/".
# It downloads the NUC software from the master PC
# and runs it.


# Add the following line to "/etc/rc.local" to enable auto start. Install tmux first.
# tmux new-session -d -s "apache_test_start" "bash /root/apache_test_start.bash"

cd /tmp
rm nuc_package.tar.gz
wget http://192.168.1.249/nuc/nuc_package.tar.gz
tar -xzvf nuc_package.tar.gz

# bash start.bash