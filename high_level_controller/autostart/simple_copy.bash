# scp /home/cpm/dev/software/high_level_controller/autostart/lab_autostart.bash  controller@192.168.1.215:/home/controller/dev/autostart
# scp /home/cpm/dev/software/high_level_controller/autostart/build/autostart   controller@192.168.1.215:/home/controller/dev/autostart
# scp /home/cpm/dev/software/cpm_lib/build/libcpm.so    controller@192.168.1.215:/home/controller/dev/autostart/cpm

scp ~/dev/software/high_level_controller/autostart/lab_autostart.bash  guest@192.168.1.201:/home/guest/autostart

# @reboot ~/dev/autostart/lab_autostart.bash