# Ping to make sure that an internet connection is available
# Write to /dev/null to suppress output
until ping -c1 ntp1.rwth-aachen.de >/dev/null 2>&1; do sleep 0.1; done

# Now enforce time sync
sudo service ntp stop
sudo ntpd -gq
sudo service ntp restart

# If you do not want sudo: Place in /etc/rc.local
# The service only starts if the file is formatted properly, like this, with your code in between:

#!/bin/sh
#
# rc.local
#
# Some explanation

# Your code

exit 0

# Also, you need to make sure that the file is executable: sudo chmod +x /etc/rc.local

# Another problem: We want the lab / hlc program to only start if the time sync was finished
# Solutions for the rc.local as well as lab_autostart script can be found below:

# For communication between processes, proposed solution:
# -------------------------------------------------------
# -------------------------------------------------------
# -------------------------------------------------------

#!/bin/bash

# First, set a wrong date (for testing purposes)
date +%T -s "14:14:00"

# Ping to make sure that an internet connection is available
# Write to /dev/null to suppress output
until ping -c1 ntp1.rwth-aachen.de >/dev/null 2>&1; do sleep 0.1; done

# Now enforce time sync
service ntp stop
ntpd -gq
service ntp restart

# Then, setup communication with the NUC startup script
# We could also run this as another user from here
# But that would be more complicated
# (The script needs to be run from guest)
# We tell the script that we are finished with the clock sync
# It is sufficient to just create a pipe here - we will check for its existence
# in the other script using -p
nuc_ntp_pipe=/tmp/nuc_ntp_pipe
nuc_lab_pipe=/tmp/nuc_lab_pipe

mkfifo /tmp/nuc_ntp_pipe
mkfifo /tmp/nuc_lab_pipe

chmod a+rwx /tmp/nuc_ntp_pipe
chmod a+rwx /tmp/nuc_lab_pipe

# delete the named pipe on exit
trap "rm $nuc_ntp_pipe $nuc_lab_pipe" EXIT

# Communication: 1 to tell that time sync is done, expect 1 as answer
echo "1" > $nuc_ntp_pipe
while read ans < $nuc_lab_pipe; do
        if [[ $ans -eq 1 ]]; then
                break
        fi
        sleep 1
done

echo "Done" >> /tmp/test.txt

exit 0

# Proposed change for autostart, add at start after export statements:
# -------------------------------------------------------
# -------------------------------------------------------
# -------------------------------------------------------

# Wait for clock sync before doing anything else, because starting any program before a clock sync would cause problems
# The clock sync is performed in rc.local of the sudo user
# We communicate via pipes, alternatively starting this script as another user would have been possible as well
# Check for existing pipe, wait until it has been created
nuc_ntp_pipe=/tmp/nuc_ntp_pipe
nuc_lab_pipe=/tmp/nuc_lab_pipe

while [[ ! (-p $nuc_ntp_pipe && -p $nuc_lab_pipe ) ]]; do
        sleep 1
done

# Read msg from pipe, which is sent after the time sync, then answer
# 1 -> time sync done, answer with 1 on other pipe
while read sy < $nuc_ntp_pipe; do
	if [[ $sy -eq 1 ]]; then
		break
	fi
	sleep 1
done
echo "1" > $nuc_lab_pipe
