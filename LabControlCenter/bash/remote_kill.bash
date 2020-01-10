
#!/bin/bash
# IP must be set
# This file is called by the LCC when remote deployment was selected and kill has been pressed
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

ssh guest@${IP} << 'EOF'
    tmux kill-session -t "middleware"
    tmux kill-session -t "script"
EOF