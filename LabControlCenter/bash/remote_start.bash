#!/bin/bash
#Get command line arguments
for i in "$@"
do
case $i in
    --script_path=*)
    SCRIPT_PATH="${i#*=}"
    shift # past argument=value
    ;;
    --script_arguments=*)
    SCRIPT_ARGS="${i#*=}"
    shift # past argument=value
    ;;
    --middleware_id=*)
    MIDDLEWARE_ID="${i#*=}"
    shift # past argument=value
    ;;
    --middleware_arguments=*)
    MIDDLEWARE_ARGS="${i#*=}"
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done

cd /tmp/software/home/cpm/dev/software

# Set correct IP in local communication script
my_ip=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
/bin/cp -rf ./hlc/middleware/QOS_LOCAL_COMMUNICATION.xml.template ./hlc/middleware/build/QOS_LOCAL_COMMUNICATION.xml
sed -i -e "s/TEMPLATE_IP/${my_ip}/g" ./hlc/middleware/build/QOS_LOCAL_COMMUNICATION.xml
/bin/cp -rf ./hlc/middleware/build/QOS_LOCAL_COMMUNICATION.xml ./hlc/$script_path/

# Start middleware
if ! [ -z "${MIDDLEWARE_ID}" ] 
then
    tmux new-session -d -s "middleware" "cd /tmp/software/;bash tmux_middleware.bash --middleware_id=${MIDDLEWARE_ID} --middleware_arguments=${MIDDLEWARE_ARGS} &> tmux_middleware.txt"
fi

# Start script
tmux new-session -d -s "middleware" "cd /tmp/software/;bash tmux_script.bash --script_path=${SCRIPT_PATH} --script_arguments=${SCRIPT_ARGS} &> tmux_script.txt"
