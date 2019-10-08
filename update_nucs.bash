#!/bin/bash
#Get command line arguments
for i in "$@"
do
case $i in
    -vi=*|--vehicle_ids=*)
    vehicle_ids="${i#*=}"
    shift # past argument=value
    ;;
    -va=*|--vehicle_amount=*)
    vehicle_amount="${i#*=}"
    shift # past argument=value
    ;;
    -pw=*|--password=*)
    password="${i#*=}"
    shift # past argument=value
    ;;
    *)
          # unknown option
    ;;
esac
done

#Check for existence of required command line arguments
if ( [ -z "$vehicle_ids" ] && [ -z "$vehicle_amount" ] ) || [ -z "$password" ]
then
      echo "Invalid use, enter vehicle IDs or amount and a password"
      exit 1
fi

#If vehicle amount was set, create vehicle id list in vehicle_ids from that, style: 1,...,vehicle_amount
if !([ -z "$vehicle_amount" ])
then
    vehicle_ids=$(seq -s, 1 1 ${vehicle_amount})
fi

IFS=,
for val in $vehicle_ids;
do
    ip=$(printf "192.168.1.2%02d" ${val})
    echo $ip
    sshpass -p $password ssh -t controller@$ip "echo ${password} | sudo -S apt-get update;sudo apt-get upgrade"
done