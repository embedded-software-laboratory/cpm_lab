#!/bin/bash
exit_script() {
    tmux kill-session -t "LabControlCenter"
    tmux kill-session -t "BaslerLedDetection"
    tmux kill-session -t "ips_pipeline"
    trap - SIGINT SIGTERM # clear the trap
}

trap exit_script SIGINT SIGTERM

tmux new-session -d -s "LabControlCenter" "(cd LabControlCenter;./build/LabControlCenter -autostart_param >stdout.txt 2>stderr.txt)"
tmux new-session -d -s "BaslerLedDetection" "(cd ips2;./build/BaslerLedDetection >stdout_led_detection.txt 2>stderr_led_detection.txt)"
tmux new-session -d -s "ips_pipeline" "(cd ips2;./build/ips_pipeline >stdout_ips.txt 2>stderr_ips.txt)"

sleep infinity