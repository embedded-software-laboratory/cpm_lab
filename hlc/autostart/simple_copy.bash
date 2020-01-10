# scp /home/cpm/dev/software/hlc/autostart/lab_autostart.bash  controller@192.168.1.215:/home/controller/dev/autostart
# scp /home/cpm/dev/software/hlc/autostart/build/autostart   controller@192.168.1.215:/home/controller/dev/autostart
# scp /home/cpm/dev/cpm_base/cpm_lib/build/libcpm.so    controller@192.168.1.215:/home/controller/dev/autostart/cpm

scp ~/dev/software/hlc/autostart/lab_autostart.bash  guest@192.168.1.205:/home/guest/autostart
scp ~/dev/software/hlc/autostart/lab_autostart.bash  guest@192.168.1.210:/home/guest/autostart
scp ~/dev/software/hlc/autostart/lab_autostart.bash  guest@192.168.1.211:/home/guest/autostart
scp ~/dev/software/hlc/autostart/lab_autostart.bash  guest@192.168.1.213:/home/guest/autostart
scp ~/dev/software/hlc/autostart/lab_autostart.bash  guest@192.168.1.214:/home/guest/autostart
scp ~/dev/software/hlc/autostart/lab_autostart.bash  guest@192.168.1.215:/home/guest/autostart

# @reboot ~/dev/autostart/lab_autostart.bash