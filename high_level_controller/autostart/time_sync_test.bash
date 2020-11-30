# Ping to make sure that an internet connection is available
# Write to /dev/null to suppress output
until ping -c1 ntp1.rwth-aachen.de >/dev/null 2>&1; do sleep 0.1; done

# Now enforce time sync
sudo service ntp stop
sudo ntpd -gq
sudo service ntp restart

# If you do not want sudo: https://askubuntu.com/questions/889632/startup-script-with-sudo-in-ubuntu-16-10 - place in /etc/rc.local as startup script