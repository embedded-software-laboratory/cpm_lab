# scp /home/cpm/dev/software/high_level_controller/autostart/lab_autostart.bash  controller@192.168.1.215:/home/controller/dev/autostart
# scp /home/cpm/dev/software/high_level_controller/autostart/build/autostart   controller@192.168.1.215:/home/controller/dev/autostart
# scp /home/cpm/dev/software/cpm_lib/build/libcpm.so    controller@192.168.1.215:/home/controller/dev/autostart/cpm

# Get autostart folder directory relative to this script's directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"
ABSOLUTE_AUTOSTART_DIR="$(realpath "${DIR}")"

scp ${ABSOLUTE_AUTOSTART_DIR}/lab_autostart.bash  guest@192.168.1.202:/home/guest/autostart
scp ${ABSOLUTE_AUTOSTART_DIR}/lab_autostart.bash  guest@192.168.1.219:/home/guest/autostart
scp ${ABSOLUTE_AUTOSTART_DIR}/lab_autostart.bash  guest@192.168.1.201:/home/guest/autostart
scp ${ABSOLUTE_AUTOSTART_DIR}/lab_autostart.bash  guest@192.168.1.203:/home/guest/autostart


# @reboot ~/dev/autostart/lab_autostart.bash