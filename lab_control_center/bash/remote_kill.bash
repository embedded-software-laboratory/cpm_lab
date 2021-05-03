
#!/bin/bash
# IP must be set
# This file is called by the LCC when distributed / remote deployment was selected and kill has been pressed
# DESCRIPTION: Kill the middleware and script sessions that were started on the NUCs remotely
for i in "$@"
do
case $i in
    --ip=*)
    IP="${i#*=}"
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done

trap exit_script SIGINT SIGTERM

# Ping to make sure that the NUC is available - we want blocking behaviour in case it is not, this is handled by the C++ program
# Write to /dev/null to suppress output
until ping -c1 ${IP} >/dev/null 2>&1; do sleep 0.1; done

ssh guest@${IP} << 'EOF'
    tmux kill-session -t "middleware"
    tmux kill-session -t "script"
EOF