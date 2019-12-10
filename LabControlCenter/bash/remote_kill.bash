
#!/bin/bash
# IP must be set
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