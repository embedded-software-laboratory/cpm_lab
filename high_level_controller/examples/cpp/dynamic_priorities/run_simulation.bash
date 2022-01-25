#!/bin/bash


export IP_SELF=$(ip route get 8.8.8.8 | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')
export DDS_INITIAL_PEER=rtps@udpv4://$IP_SELF:25598

# heptrack can be used to analyse memory usage and detect leaks
# heaptrack ./build/simulation --n=5 --hlc_mode=2 --steps=100 

./build/simulation --n=15 --hlc_mode=2
